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

#define LOG_TAG "android.hardware.sensors@aidl-service.bosch"

#include <android-base/logging.h>
#include <android/binder_manager.h>
#include <android/binder_process.h>

#include "sensors-impl/SensorsHalAidl.h"

using aidl::android::hardware::sensors::SensorsHalAidl;

int main() {
  ALOGD("Entering main function of Bosch Sensors AIDL Service");
  ABinderProcess_setThreadPoolMaxThreadCount(0);
  // Make a default sensors service
  auto sensor = ndk::SharedRefBase::make<SensorsHalAidl>();
  const std::string sensorName = std::string() + SensorsHalAidl::descriptor + "/default";
  ALOGD("Add Service %s", sensorName.c_str());
  binder_status_t status = AServiceManager_addService(sensor->asBinder().get(), sensorName.c_str());
  CHECK_EQ(status, STATUS_OK);

  ABinderProcess_joinThreadPool();
  return EXIT_FAILURE;  // should not reach
}
