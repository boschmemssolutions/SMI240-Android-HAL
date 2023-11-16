/*
 * Copyright (C) 2023 Robert Bosch GmbH
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

#include "sensors-impl/Sensor.h"

#include <log/log.h>

#include <cmath>

#include "iioHwctl.h"
#include "utils/SystemClock.h"

using ::ndk::ScopedAStatus;

namespace aidl {
namespace android {
namespace hardware {
namespace sensors {

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
  if (samplingPeriodNs < mSensorInfo.minDelayUs * 1000LL) {
    samplingPeriodNs = mSensorInfo.minDelayUs * 1000LL;
  } else if (samplingPeriodNs > mSensorInfo.maxDelayUs * 1000LL) {
    samplingPeriodNs = mSensorInfo.maxDelayUs * 1000LL;
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

ScopedAStatus Sensor::flush() {
  // Only generate a flush complete event if the sensor is enabled and if the
  // sensor is not a one-shot sensor.
  if (!mIsEnabled || (mSensorInfo.flags & static_cast<uint32_t>(SensorInfo::SENSOR_FLAG_BITS_ONE_SHOT_MODE))) {
    return ScopedAStatus::fromServiceSpecificError(static_cast<int32_t>(BnSensors::ERROR_BAD_VALUE));
  }

  // Note: If a sensor supports batching, write all of the currently batched
  // events for the sensor to the Event FMQ prior to writing the flush complete
  // event.
  Event ev;
  ev.sensorHandle = mSensorInfo.sensorHandle;
  ev.sensorType = SensorType::META_DATA;
  EventPayload::MetaData meta = {
    .what = MetaDataEventType::META_DATA_FLUSH_COMPLETE,
  };
  ev.payload.set<EventPayload::Tag::meta>(meta);
  std::vector<Event> evs{ev};
  mCallback->postEvents(evs, isWakeUpSensor());

  return ScopedAStatus::ok();
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

bool Sensor::isWakeUpSensor() {
  return mSensorInfo.flags & static_cast<uint32_t>(SensorInfo::SENSOR_FLAG_BITS_WAKE_UP);
}

std::vector<Event> Sensor::readEvents() {
  std::vector<Event> events;
  Event event;
  event.sensorHandle = mSensorInfo.sensorHandle;
  event.sensorType = mSensorInfo.type;
  event.timestamp = ::android::elapsedRealtimeNano();
  memset(&event.payload, 0, sizeof(event.payload));
  readEventPayload(event.payload);
  events.push_back(event);
  return events;
}

ScopedAStatus Sensor::setOperationMode(OperationMode mode) {
  if (mode == OperationMode::NORMAL) {
    return ScopedAStatus::ok();
  } else {
    return ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
  }
}

bool Sensor::supportsDataInjection() const {
  return mSensorInfo.flags & static_cast<uint32_t>(SensorInfo::SENSOR_FLAG_BITS_DATA_INJECTION);
}

ScopedAStatus Sensor::injectEvent(const Event& event) {
  if (event.sensorType == SensorType::ADDITIONAL_INFO) {
    return ScopedAStatus::ok();
    // When in OperationMode::NORMAL, SensorType::ADDITIONAL_INFO is used to
    // push operation environment data into the device.
  }

  if (!supportsDataInjection()) {
    return ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
  }

  return ScopedAStatus::fromServiceSpecificError(static_cast<int32_t>(BnSensors::ERROR_BAD_VALUE));
}

void Sensor::readEventPayload(EventPayload& payload) {
  std::string values;
  if (0 == ::rb::hardware::sensors::hwctl::readFromFile(&mIioFileName, values)) {
    fillPayload(payload, values);
  }
}

void Sensor::fillPayload(EventPayload& payload, std::string& data) {
  std::istringstream iss(data);
  std::vector<std::string> results(std::istream_iterator<std::string>{iss}, std::istream_iterator<std::string>());

  if (results.size() > 2) {
    EventPayload::Vec3 vec3 = {
      .x = std::stof(results[0]) * mSensorInfo.resolution,
      .y = std::stof(results[1]) * mSensorInfo.resolution,
      .z = std::stof(results[2]) * mSensorInfo.resolution,
      .status = SensorStatus::ACCURACY_HIGH,
    };
    payload.set<EventPayload::Tag::vec3>(vec3);
  }
}

}  // namespace sensors
}  // namespace hardware
}  // namespace android
}  // namespace aidl
