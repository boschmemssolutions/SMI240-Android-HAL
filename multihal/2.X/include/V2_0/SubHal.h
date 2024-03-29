/*
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

#include <android/hardware/sensors/1.0/types.h>
#include <android/hardware/sensors/2.0/ISensors.h>

#include <vector>

#include "ScopedWakelock.h"

// Indicates the current version of the multiHAL interface formatted as (HAL major version) << 24 |
// (HAL minor version) << 16 | (multiHAL version)
#define SUB_HAL_2_0_VERSION 0x02000000

namespace android {
namespace hardware {
namespace sensors {
namespace V2_0 {
namespace implementation {

using ::android::hardware::sensors::V1_0::Event;
using ::android::hardware::sensors::V1_0::Result;
using ::android::hardware::sensors::V1_0::SensorInfo;

/**
 * Interface that contains several callbacks into the HalProxy class to communicate dynamic sensor
 * changes and sensor events to the framework and acquire wake locks. The HalProxy will ensure
 * callbacks occurring at the same time from multiple sub-HALs are synchronized in a safe, efficient
 * manner.
 */
class IHalProxyCallback : public ISensorsCallback {
public:
  /**
   * Thread-safe callback used to post events to the HalProxy. Sub-HALs should invoke this
   * whenever new sensor events need to be delivered to the sensors framework. Once invoked, the
   * HalProxy will attempt to send events to the sensors framework using a blocking write with a
   * 5 second timeout. This write may be done asynchronously if the queue used to communicate
   * with the framework is full to avoid blocking sub-HALs for the length of the timeout. If the
   * write fails, the events will be dropped and any wake locks held will be released.
   *
   * The provided ScopedWakelock must be locked if the events are from wakeup sensors. If it's
   * not locked accordingly, the HalProxy will crash as this indicates the sub-HAL isn't compliant
   * with the sensors HAL 2.0 specification. Additionally, since ScopedWakelock isn't copyable,
   * the HalProxy will take ownership of the wake lock given when this method is invoked. Once the
   * method returns, the HalProxy will handle holding the wake lock, if necessary, until the
   * framework has successfully processed any wakeup events.
   *
   * No return type is used for this callback to avoid sub-HALs trying to resend events when
   * writes fail. Writes should only fail when the framework is under inordinate stress which will
   * likely result in a framework restart so retrying will likely only result in overloading the
   * HalProxy. Sub-HALs should always assume that the write was a success and perform any
   * necessary cleanup. Additionally, the HalProxy will ensure it logs any errors (through ADB and
   * bug reports) it encounters during delivery to ensure it's obvious that a failure occurred.
   *
   * @param events the events that should be sent to the sensors framework
   * @param wakelock ScopedWakelock that should be locked to send events from wake sensors and
   *     unlocked otherwise.
   */
  virtual void postEvents(const std::vector<Event>& events, ScopedWakelock wakelock) = 0;

  /**
   * Initializes a ScopedWakelock on the stack that, when locked, will increment the reference
   * count for the sub-HAL's wake lock managed inside the HalProxy. See the ScopedWakelock class
   * definition for how it should be used.
   *
   * @param lock whether the ScopedWakelock should be locked before it's returned.
   * @return the created ScopedWakelock
   */
  virtual ScopedWakelock createScopedWakelock(bool lock) = 0;
};

/**
 * ISensorsSubHal is an interface that sub-HALs must implement in order to be compliant with
 * multihal 2.0 and in order for the HalProxy to successfully load and communicate with the sub-HAL.
 *
 * Any vendor wishing to implement this interface and support multihal 2.0 will need to create a
 * dynamic library that exposes sensorsHalGetSubHal (defined below). This library will be loaded by
 * the HalProxy when the sensors HAL is initialized and then the HalProxy will retrieve the vendor's
 * implementation of sensorsHalGetSubHal.
 *
 * With the exception of the initialize method, ISensorsSubHal will implement the ISensors.hal spec.
 * Any sensor handles given to the HalProxy, either through getSensorsList() or the
 * onDynamicSensors(Dis)Connected callbacks, will be translated to avoid clashing with other sub-HAL
 * handles. To achieve this, the HalProxy will use the upper byte to store the sub-HAL index and
 * sub-HALs can continue to use the lower 3 bytes of the handle.
 */
class ISensorsSubHal : public ISensors {
public:
  // The ISensors version of initialize isn't used for multihal. Instead, sub-HALs must implement
  // the version below to allow communciation logic to centralized in the HalProxy
  Return<Result> initialize(const ::android::hardware::MQDescriptorSync<Event>& /* eventQueueDescriptor */,
                            const ::android::hardware::MQDescriptorSync<uint32_t>& /* wakeLockDescriptor */,
                            const sp<ISensorsCallback>& /* sensorsCallback */) final {
    return Result::INVALID_OPERATION;
  }

  /**
   * Method defined in ::android::hidl::base::V1_0::IBase.
   *
   * This method should write debug information to hidl_handle that is useful for debugging
   * issues. Suggestions include:
   * - Sensor info including handle values and any other state available in the SensorInfo class
   * - List of active sensors and their current sampling period and reporting latency
   * - Information about pending flush requests
   * - Current operating mode
   * - Currently registered direct channel info
   * - A history of any of the above
   */
  virtual Return<void> debug(const hidl_handle& fd, const hidl_vec<hidl_string>& args) = 0;

  /**
   * @return A human-readable name for use in wake locks and logging.
   */
  virtual const std::string getName() = 0;

  /**
   * This is the first method invoked on the sub-HAL after it's allocated through
   * sensorsHalGetSubHal() by the HalProxy. Sub-HALs should use this to initialize any state and
   * retain the callback given in order to communicate with the HalProxy. Method will be called
   * anytime the sensors framework restarts. Therefore, this method will be responsible for
   * reseting the state of the subhal and cleaning up and reallocating any previously allocated
   * data. Initialize should ensure that the subhal has reset its operation mode to NORMAL state
   * as well.
   *
   * @param halProxyCallback callback used to inform the HalProxy when a dynamic sensor's state
   *     changes, new sensor events should be sent to the framework, and when a new ScopedWakelock
   *     should be created.
   * @return result OK on success
   */
  virtual Return<Result> initialize(const sp<IHalProxyCallback>& halProxyCallback) = 0;
};

}  // namespace implementation
}  // namespace V2_0
}  // namespace sensors
}  // namespace hardware
}  // namespace android

using ::android::hardware::sensors::V2_0::implementation::ISensorsSubHal;

/**
 * Function that must be exported so the HalProxy class can invoke it on the sub-HAL dynamic
 * library. This function will only be invoked once at initialization time.
 *
 * NOTE: The supported sensors HAL version must match SUB_HAL_2_0_VERSION exactly or the HalProxy
 *     will fail to initialize.
 *
 * @param uint32_t when this function returns, this parameter must contain the HAL version that
 *     this sub-HAL supports. To support this version of multi-HAL, this must be set to
 *     SUB_HAL_2_0_VERSION.
 * @return A statically allocated, valid ISensorsSubHal implementation.
 */
__attribute__((visibility("default"))) extern "C" ISensorsSubHal* sensorsHalGetSubHal(uint32_t* version);
