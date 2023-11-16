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
 
#pragma once

#include <aidl/android/hardware/sensors/BnSensors.h>

#include <string>
#include <thread>

namespace aidl {
namespace android {
namespace hardware {
namespace sensors {

class ISensorsEventCallback {
public:
  using Event = ::aidl::android::hardware::sensors::Event;

  virtual ~ISensorsEventCallback(){};
  virtual void postEvents(const std::vector<Event>& events, bool wakeup) = 0;
};

class Sensor {
public:
  using OperationMode = ::aidl::android::hardware::sensors::ISensors::OperationMode;
  using Event = ::aidl::android::hardware::sensors::Event;
  using EventPayload = ::aidl::android::hardware::sensors::Event::EventPayload;
  using SensorInfo = ::aidl::android::hardware::sensors::SensorInfo;
  using SensorType = ::aidl::android::hardware::sensors::SensorType;
  using MetaDataEventType = ::aidl::android::hardware::sensors::Event::EventPayload::MetaData::MetaDataEventType;

  Sensor(ISensorsEventCallback* callback);
  virtual ~Sensor();

  const SensorInfo& getSensorInfo() const;
  void batch(int64_t samplingPeriodNs);
  virtual void activate(bool enable);
  ndk::ScopedAStatus flush();

  ndk::ScopedAStatus setOperationMode(OperationMode mode);
  bool supportsDataInjection() const;
  ndk::ScopedAStatus injectEvent(const Event& event);

protected:
  void run();
  virtual std::vector<Event> readEvents();
  static void startThread(Sensor* sensor);

  bool isWakeUpSensor();

  void readEventPayload(EventPayload&);
  void fillPayload(EventPayload& payload, std::string& data);

  bool mIsEnabled;
  int64_t mSamplingPeriodNs;
  int64_t mLastSampleTimeNs;
  SensorInfo mSensorInfo;
  int32_t* mMinDelay = &mSensorInfo.minDelayUs;
  int32_t* mMaxDelay = &mSensorInfo.maxDelayUs;

  std::atomic_bool mStopThread;
  std::condition_variable mWaitCV;
  std::mutex mRunMutex;
  std::thread mRunThread;

  ISensorsEventCallback* mCallback;

  std::string mIioFileName;
};

}  // namespace sensors
}  // namespace hardware
}  // namespace android
}  // namespace aidl
