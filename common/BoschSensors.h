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

#ifndef ANDROID_HARDWARE_BOSCH_SENSORS_H
#define ANDROID_HARDWARE_BOSCH_SENSORS_H

#include <stdint.h>

#include <cmath>

#include "iioFiles.h"

namespace bosch {
namespace sensors {

template <class Base, class EventCallback, typename SensorType>
class Smi240Accel : public Base {
public:
  Smi240Accel(int32_t sensorHandle, EventCallback* callback);

private:
  constexpr float gravityToAcceleration(float gravity) { return gravity * 9.80665f; }
};

template <class Base, class EventCallback, typename SensorType>
class Smi240Gyro : public Base {
public:
  Smi240Gyro(int32_t sensorHandle, EventCallback* callback);

private:
  constexpr float degreeToRad(float degree) { return degree * M_PI / 180.0f; }
};

template <class Base, class EventCallback, typename SensorType>
Smi240Accel<Base, EventCallback, SensorType>::Smi240Accel(int32_t sensorHandle, EventCallback* callback)
  : Base(callback) {
  Base::mSensorInfo.sensorHandle = sensorHandle;
  Base::mSensorInfo.name = "BOSCH SMI240 Accelerometer Sensor";
  Base::mSensorInfo.vendor = "Robert Bosch GmbH";
  Base::mSensorInfo.version = 1;
  Base::mSensorInfo.type = SensorType::ACCELEROMETER;
  Base::mSensorInfo.typeAsString = "android.sensor.accelerometer";
  Base::mSensorInfo.maxRange = gravityToAcceleration(16);
  Base::mSensorInfo.resolution = gravityToAcceleration(1.0f / 2000.0f);
  Base::mSensorInfo.power = 5.0f;
  Base::mSensorInfo.fifoReservedEventCount = 0;
  Base::mSensorInfo.fifoMaxEventCount = 0;
  Base::mSensorInfo.requiredPermission = "";
  Base::mSensorInfo.flags = 0;

  *Base::mMinDelay = 10000;
  *Base::mMaxDelay = 200000;

  Base::mIioFileName = ::rb::hardware::sensors::hwctl::SMI240ACC;
};

template <class Base, class EventCallback, typename SensorType>
Smi240Gyro<Base, EventCallback, SensorType>::Smi240Gyro(int32_t sensorHandle, EventCallback* callback)
  : Base(callback) {
  Base::mSensorInfo.sensorHandle = sensorHandle;
  Base::mSensorInfo.name = "BOSCH SMI240 Gyroscope Sensor";
  Base::mSensorInfo.vendor = "Robert Bosch GmbH";
  Base::mSensorInfo.version = 1;
  Base::mSensorInfo.type = SensorType::GYROSCOPE;
  Base::mSensorInfo.typeAsString = "android.sensor.gyroscope";
  Base::mSensorInfo.maxRange = degreeToRad(300);
  Base::mSensorInfo.resolution = degreeToRad(1.0f / 100.0f);
  Base::mSensorInfo.power = 5.0f;
  Base::mSensorInfo.fifoReservedEventCount = 0;
  Base::mSensorInfo.fifoMaxEventCount = 0;
  Base::mSensorInfo.requiredPermission = "";
  Base::mSensorInfo.flags = 0;

  *Base::mMinDelay = 10000;
  *Base::mMaxDelay = 200000;

  Base::mIioFileName = ::rb::hardware::sensors::hwctl::SMI240GYRO;
};

}  // namespace sensors
}  // namespace bosch

#endif  // ANDROID_HARDWARE_BOSCH_SENSORS_H
