# Android HAL for smi240 Sensor

## Table of Contents
 - [Which HAL type to use](#Type)
 - [HIDL,AIDL and Multi HAL](#Hal)
 - [Legacy HAL](#legacyHal)
 - [License](#License)
 - [Architecture](#Architecture)

## HAL Type <a name=Type></a>
Different HAL typea are provided in this repository. 

- To use HIDL,AIDL and Multi HAL, all the folders except "legacy" folder are required. [->](#Hal)
- To usr legacy HAL only the legacy folder is required. [->](#legacyHal)


## HIDL,AIDL and Multi HAL <a name=Hal></a>

smi240 HIDL,AIDL and Multi Hal is module running in user space, under framework of Android Hardware Abstraction Layer.
It provides capability to bridge smi240 sensor driver in kernel and Android APPs.

smi240 HAL supports data synchronization feature of smi240 sensor only and shall be used with [smi240 kernel driver](https://github.com/boschmemssolutions/SMI240-Linux-Driver-IIO) based on Linux Input Framework.

## Build

Modify the device makefile (i.e. *android-platform/device/brcm/rpi4/device.mk*) by adding these lines:

```make
# For sensor hal 2.0
PRODUCT_PACKAGES += android.hardware.sensors@2.0-service.bosch
# For sensor hal 2.1
PRODUCT_PACKAGES += android.hardware.sensors@2.1-service.bosch
# For sensor hal AIDL
PRODUCT_PACKAGES += android.hardware.sensors@aidl-service.bosch
# For sensor multi hal 2.X
PRODUCT_PACKAGES += android.hardware.sensors@2.X-service.multihal bosch.sensor.multihal
PRODUCT_COPY_FILES += hardware/bosch/sensors/multihal/hals.conf:$(TARGET_COPY_OUT_VENDOR)/etc/sensors/hals.conf
# For sensor multi hal AIDL
PRODUCT_PACKAGES += android.hardware.sensors-service.multihal bosch.sensor.multihal
PRODUCT_COPY_FILES += hardware/bosch/sensors/multihal/hals.conf:$(TARGET_COPY_OUT_VENDOR)/etc/sensors/hals.conf
```

Modify the sepolicy file_contexts file (i.e. *android-platform/device/brcm/rpi4/sepolicy/file_contexts*) by adding these lines:

```make
# For sensor hal 2.0
/vendor/bin/hw/android\.hardware\.sensors@2\.0-service\.bosch u:object_r:hal_sensors_default_exec:s0
# For sensor hal 2.1
/vendor/bin/hw/android\.hardware\.sensors@2\.1-service\.bosch u:object_r:hal_sensors_default_exec:s0
# For sensor hal AIDL
/vendor/bin/hw/android\.hardware\.sensors@aidl-service\.bosch u:object_r:hal_sensors_default_exec:s0
# For sensor multi hal 2.X
/vendor/bin/hw/android\.hardware\.sensors@2\.X-service\.multihal u:object_r:hal_sensors_default_exec:s0
# For sensor multi hal AIDL
/vendor/bin/hw/android\.hardware\.sensors-service\.multihal u:object_r:hal_sensors_default_exec:s0
```


## Legacy HAL <a name=legacyHal></a>

smi240 legacy hal is a custom specific implmentaion, which runs on a specific Linux Plattform of the custom instead on Andorid.
For more information please refer to [HAL sensors-1.0](https://source.android.com/devices/sensors/hal-interface)

smi240 HAL supports data synchronization feature of smi240 sensor only and shall be used with [smi240 kernel driver](https://github.com/boschmemssolutions/SMI240-Linux-Driver-IIO) based on Linux Input Framework.

## Usage

smi240 HAL is a shared lib, compatable with android sensor 1.0 specification.
There are two ways to compile/build the lib.

### build with Android SDK

This way, the HAL lib is dynamically linked to other Android libs in its SDK.
For convenient testing, it can also be compiled as an executable to directly run on command line.

Integrate this repository into your build setup into location
```
 android/hardware/libhardware/modules/
```
extend your build setup with smi240-hal and compile it.

For more information please refer to [android build system](https://source.android.com/setup/build/building)

### build independantly without Android SDK

The essecial header files from Android have already been placed in the repo and adapted for compiling the lib. The local libs instead of Android SDK libs would be used to link to the HAL lib in the build process.
Go to the folder where Makefile is located, and first clean the folder by issuing command:
```
make clean
```
Then simply issue command:
```
make
```
the Hal lib will be built in seconds.
Note: the Makefile is written for cross compiling on the host and with toolchain specified. If a different environment is used, a little change is needed to fit into new conditions.

## License <a name=License></a>
See [LICENSE](LICENSE) file

## Architecture <a name=Architecture></a>
```
                 Android APPS
                      |
                 Android Framework
                      |
                  smi240 HAL
                      |
-------------------------------------------------------
                 |          |
               sysfs       dev
                 \          /
               input-subsystem
	              |
                smi240_driver
                      |
-------------------------------------------------------
                  Hardware
```
