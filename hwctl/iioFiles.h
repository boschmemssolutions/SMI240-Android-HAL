/*
 * Copyright (C) 2023 Robert Bosch GmbH
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

#include <string>

namespace rb {
namespace hardware {
namespace sensors {
namespace hwctl {

const std::string SMI240ACC = "/sys/bus/iio/devices/iio:device0/in_accel_x&y&z_raw";
const std::string SMI240GYRO = "/sys/bus/iio/devices/iio:device0/in_anglvel_x&y&z_raw";

}  // namespace hwctl
}  // namespace sensors
}  // namespace hardware
}  // namespace rb
