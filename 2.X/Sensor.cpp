/*
 * Copyright (C) 2023 Robert Bosch GmbH. All rights reserved.
 * Copyright (C) 2023 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "Sensor.h"

#include <log/log.h>
#include <utils/SystemClock.h>

#include <cmath>

#include "iioHwctl.h"

namespace android {
namespace hardware {
namespace sensors {
namespace V2_X {
namespace implementation {

using ::android::hardware::sensors::V1_0::EventPayload;
using ::android::hardware::sensors::V1_0::MetaDataEventType;
using ::android::hardware::sensors::V1_0::OperationMode;
using ::android::hardware::sensors::V1_0::Result;
using ::android::hardware::sensors::V1_0::SensorFlagBits;
using ::android::hardware::sensors::V1_0::SensorStatus;
using ::android::hardware::sensors::V2_1::Event;
using ::android::hardware::sensors::V2_1::SensorInfo;

Sensor::Sensor(ISensorsEventCallback* callback)
  : mIsEnabled(false), mSamplingPeriodNs(0), mLastSampleTimeNs(0), mCallback(callback) {
  mRunThread = std::thread(startThread, this);
}

Sensor::~Sensor() {
  std::unique_lock<std::mutex> lock(mRunMutex);
  mStopThread = true;
  mIsEnabled = false;
  mWaitCV.notify_all();
  lock.release();
  mRunThread.join();
}

const SensorInfo& Sensor::getSensorInfo() const { return mSensorInfo; }

void Sensor::batch(int64_t samplingPeriodNs) {
  if (samplingPeriodNs < mSensorInfo.minDelay * 1000LL) {
    samplingPeriodNs = mSensorInfo.minDelay * 1000LL;
  } else if (samplingPeriodNs > mSensorInfo.maxDelay * 1000LL) {
    samplingPeriodNs = mSensorInfo.maxDelay * 1000LL;
  }

  if (mSamplingPeriodNs != samplingPeriodNs) {
    mSamplingPeriodNs = samplingPeriodNs;
    // Wake up the 'run' thread to check if a new event should be generated now
    mWaitCV.notify_all();
  }
}

void Sensor::activate(bool enable) {
  ALOGD("Sensor activate %s %d", mSensorInfo.name.c_str(), enable);
  if (mIsEnabled != enable) {
    std::unique_lock<std::mutex> lock(mRunMutex);
    mIsEnabled = enable;
    mWaitCV.notify_all();
  }
}

Result Sensor::flush() {
  // Only generate a flush complete event if the sensor is enabled and if the
  // sensor is not a one-shot sensor.
  if (!mIsEnabled || (mSensorInfo.flags & static_cast<uint32_t>(SensorFlagBits::ONE_SHOT_MODE))) {
    return Result::BAD_VALUE;
  }

  // Note: If a sensor supports batching, write all of the currently batched
  // events for the sensor to the Event FMQ prior to writing the flush complete
  // event.
  Event ev;
  ev.sensorHandle = mSensorInfo.sensorHandle;
  ev.sensorType = SensorType::META_DATA;
  ev.u.meta.what = MetaDataEventType::META_DATA_FLUSH_COMPLETE;
  std::vector<Event> evs{ev};
  mCallback->postEvents(evs, isWakeUpSensor());

  return Result::OK;
}

void Sensor::startThread(Sensor* sensor) { sensor->run(); }

void Sensor::run() {
  std::unique_lock<std::mutex> runLock(mRunMutex);
  constexpr int64_t kNanosecondsInSeconds = 1000 * 1000 * 1000;

  while (!mStopThread) {
    if (!mIsEnabled) {
      mWaitCV.wait(runLock, [&] { return (mIsEnabled || mStopThread); });
    } else {
      timespec curTime;
      clock_gettime(CLOCK_BOOTTIME, &curTime);
      int64_t now = (curTime.tv_sec * kNanosecondsInSeconds) + curTime.tv_nsec;
      int64_t nextSampleTime = mLastSampleTimeNs + mSamplingPeriodNs;

      if (now >= nextSampleTime) {
        mLastSampleTimeNs = now;
        nextSampleTime = mLastSampleTimeNs + mSamplingPeriodNs;
        mCallback->postEvents(readEvents(), isWakeUpSensor());
      }

      mWaitCV.wait_for(runLock, std::chrono::nanoseconds(nextSampleTime - now));
    }
  }
}

bool Sensor::isWakeUpSensor() { return mSensorInfo.flags & static_cast<uint32_t>(SensorFlagBits::WAKE_UP); }

std::vector<Event> Sensor::readEvents() {
  std::vector<Event> events;
  Event event;
  event.sensorHandle = mSensorInfo.sensorHandle;
  event.sensorType = mSensorInfo.type;
  event.timestamp = ::android::elapsedRealtimeNano();
  memset(&event.u, 0, sizeof(event.u));
  readEventPayload(event.u);
  events.push_back(event);
  return events;
}

Result Sensor::setOperationMode(OperationMode mode) {
  if (mode == OperationMode::NORMAL) {
    return Result::OK;
  } else {
    return Result::BAD_VALUE;
  }
}

bool Sensor::supportsDataInjection() const {
  return mSensorInfo.flags & static_cast<uint32_t>(SensorFlagBits::DATA_INJECTION);
}

Result Sensor::injectEvent(const Event& event) {
  (void)(event);
  return Result::INVALID_OPERATION;
}

void Sensor::readEventPayload(EventPayload& payload) {
  std::string values;
  if (0 == ::rb::hardware::sensors::hwctl::readFromFile(&mIioFileName, values)) {
    fillPayload(payload, values);
  } else {
    ALOGE("Sensor readEventPayload failed");
  }
}

void Sensor::fillPayload(EventPayload& payload, std::string& data) {
  std::istringstream iss(data);
  std::vector<std::string> results(std::istream_iterator<std::string>{iss}, std::istream_iterator<std::string>());
  if (results.size() > 2) {
    payload.vec3.x = ::atof(results[0].c_str()) * mSensorInfo.resolution;
    payload.vec3.y = ::atof(results[1].c_str()) * mSensorInfo.resolution;
    payload.vec3.z = ::atof(results[2].c_str()) * mSensorInfo.resolution;
    payload.vec3.status = SensorStatus::ACCURACY_HIGH;
  } else {
    ALOGE("Sensor fillPayload failed");
  }
}

}  // namespace implementation
}  // namespace V2_X
}  // namespace sensors
}  // namespace hardware
}  // namespace android
