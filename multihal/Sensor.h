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

#include <android/hardware/sensors/2.1/types.h>

#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

using ::android::hardware::sensors::V1_0::EventPayload;
using ::android::hardware::sensors::V1_0::OperationMode;
using ::android::hardware::sensors::V1_0::Result;
using ::android::hardware::sensors::V2_1::Event;
using ::android::hardware::sensors::V2_1::SensorInfo;
using ::android::hardware::sensors::V2_1::SensorType;

namespace android {
namespace hardware {
namespace sensors {
namespace V2_1 {
namespace subhal {
namespace implementation {

class ISensorsEventCallback {
public:
  virtual ~ISensorsEventCallback(){};
  virtual void postEvents(const std::vector<Event>& events, bool wakeup) = 0;
};

class Sensor {
public:
  Sensor(ISensorsEventCallback* callback);
  virtual ~Sensor();

  const SensorInfo& getSensorInfo() const;
  void batch(int64_t samplingPeriodNs);
  virtual void activate(bool enable);
  Result flush();

  bool supportsDataInjection() const;
  Result injectEvent(const Event& event);

protected:
  void run();
  virtual std::vector<Event> readEvents();
  static void startThread(Sensor* sensor);

  bool isWakeUpSensor();

  void readEventPayload(EventPayload&);

  bool mIsEnabled;
  int64_t mSamplingPeriodNs;
  int64_t mLastSampleTimeNs;
  SensorInfo mSensorInfo;
  int32_t* mMinDelay = &mSensorInfo.minDelay;
  int32_t* mMaxDelay = &mSensorInfo.maxDelay;

  std::atomic_bool mStopThread;
  std::condition_variable mWaitCV;
  std::mutex mRunMutex;
  std::thread mRunThread;

  ISensorsEventCallback* mCallback;

  std::string mIioFileName;
};

}  // namespace implementation
}  // namespace subhal
}  // namespace V2_1
}  // namespace sensors
}  // namespace hardware
}  // namespace android
