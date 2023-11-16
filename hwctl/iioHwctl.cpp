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

#include "iioHwctl.h"

#include <log/log.h>

#include <fstream>
#include <string>

namespace rb {
namespace hardware {
namespace sensors {
namespace hwctl {

int32_t readFromFile(std::string* filename, std::string& result) {
  std::ifstream ifs(filename->c_str());

  if (!ifs.is_open()) {
    ALOGE("Failed to read file %s", filename->c_str());
    return -1;
  }
  result.assign((std::istreambuf_iterator<char>(ifs)), (std::istreambuf_iterator<char>()));

  return 0;
}

}  // namespace hwctl
}  // namespace sensors
}  // namespace hardware
}  // namespace rb
