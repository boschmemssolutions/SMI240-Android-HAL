// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright (C) 2023 Robert Bosch GmbH. All rights reserved.
 * Copyright (C) 2011~2015 Bosch Sensortec GmbH All Rights Reserved
 * Copyright (C) 2008 The Android Open Source Project
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

#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <string.h>
#include <sys/cdefs.h>
#include <sys/stat.h>
#include <sys/timerfd.h>
#include <sys/types.h>
#include <unistd.h>

#if !defined(PLTF_LINUX_ENABLED)
#include <android/log.h>
/*Android utils headers*/
#include <utils/SystemClock.h>
#include <utils/Timers.h>
#endif

#include "BoschSensor.h"
#include "axis_remap.h"
#include "sensord_algo.h"
#include "sensord_cfg.h"
#include "sensord_hwcntl.h"
#include "sensord_hwcntl_iio.h"
#include "sensord_pltf.h"
#include "util_misc.h"

/* SMI240 ODR */
#define SMI240_ODR_5HZ 5
#define SMI240_ODR_50HZ 50
#define SMI240_ODR_100HZ 100
#define SMI240_ODR_200HZ 200
#define SMI240_ODR_400HZ 400

#define SMI240_ACCEL_RANGE_2G 2
#define SMI240_ACCEL_RANGE_4G 4
#define SMI240_ACCEL_RANGE_8G 8
#define SMI240_ACCEL_RANGE_16G 16

#define SMI240_GYRO_RANGE_125DPS 125
#define SMI240_GYRO_RANGE_250DPS 250
#define SMI240_GYRO_RANGE_500DPS 500
#define SMI240_GYRO_RANGE_1000DPS 1000
#define SMI240_GYRO_RANGE_2000DPS 2000

#define US_PER_SEC 1000000
/*the sampling interval offset depends on hardware performance
 * which needs to be tuned on target hardware runing the HAL.
 * For raspi 3 and max SPI freq set to 10000000, the offset is
 * set to 600us to compensate time used to sample data*/
#define SAMPLING_INTERVAL_OFFSET 600
static uint32_t sampling_interval = US_PER_SEC;

[[maybe_unused]] static int32_t gyro_scan_size;
static int acc_fd = -1;

[[maybe_unused]] static int gyr_fd = -1;
[[maybe_unused]] static int gyr_device_num = 0;

/**
 *
 * @param p_sSensorList
 * @return
 */
static uint32_t ap_get_sensorlist(struct sensor_t const **p_sSensorList) {
  uint64_t avail_sens_regval = 0;
  uint32_t sensor_amount = 0;
  int32_t i;
  int32_t j;

  if (0 == bosch_sensorlist.list_len) {
    switch (accl_range) {
      case ACC_CHIP_RANGCONF_2G:
      case ACC_CHIP_RANGCONF_4G:
      case ACC_CHIP_RANGCONF_8G:
      case ACC_CHIP_RANGCONF_16G:
        bosch_all_sensors[SENSORLIST_INX_ACCELEROMETER].maxRange =
            accl_range * GRAVITY_EARTH;
        bosch_all_sensors[SENSORLIST_INX_LINEAR_ACCELERATION].maxRange =
            accl_range * GRAVITY_EARTH;
        bosch_all_sensors[SENSORLIST_INX_GRAVITY].maxRange =
            accl_range * GRAVITY_EARTH;
        bosch_all_sensors[SENSORLIST_INX_WAKEUP_ACCELEROMETER].maxRange =
            accl_range * GRAVITY_EARTH;
        bosch_all_sensors[SENSORLIST_INX_WAKEUP_LINEAR_ACCELERATION].maxRange =
            accl_range * GRAVITY_EARTH;
        bosch_all_sensors[SENSORLIST_INX_WAKEUP_GRAVITY].maxRange =
            accl_range * GRAVITY_EARTH;
        break;
      default:
        PWARN("Invalid accl_range: %d", accl_range);
        break;
    }

    // bosch_all_sensors[SENSORLIST_INX_GYROSCOPE].minDelay = 20000;
    // bosch_all_sensors[SENSORLIST_INX_LINEAR_ACCELERATION].minDelay = 20000;
    // bosch_all_sensors[SENSORLIST_INX_GYROSCOPE_UNCALIBRATED].minDelay =
    // 20000;

    avail_sens_regval = ((1 << SENSORLIST_INX_ACCELEROMETER) |
                         (1 << SENSORLIST_INX_GYROSCOPE_UNCALIBRATED));

    sensor_amount = sensord_popcount_64(avail_sens_regval);

    bosch_sensorlist.list =
        (struct sensor_t *)malloc(sensor_amount * sizeof(struct sensor_t));
    if (NULL == bosch_sensorlist.list) {
      PERR("fail to malloc %d * sizeof(struct sensor_t)(=%d)", sensor_amount,
           sizeof(struct sensor_t));
      return 0;
    }

    bosch_sensorlist.bsx_list_index =
        (int32_t *)malloc(sensor_amount * sizeof(int32_t));
    if (NULL == bosch_sensorlist.bsx_list_index) {
      PERR("fail to malloc %d * 4", sensor_amount);
      free(bosch_sensorlist.list);
      return 0;
    }

    for (i = 0, j = 0; i < SENSORLIST_INX_END; i++, avail_sens_regval >>= 1) {
      if (0x0 == avail_sens_regval) {
        break;
      }

      if (avail_sens_regval & 0x1) {
        memcpy(&(bosch_sensorlist.list[j]), &(bosch_all_sensors[i]),
               sizeof(struct sensor_t));
        bosch_sensorlist.bsx_list_index[j] = i;
        j++;
      }
    }

    bosch_sensorlist.list_len = sensor_amount;
  }

  *p_sSensorList = bosch_sensorlist.list;
  return bosch_sensorlist.list_len;
}

static inline int32_t SMI240_convert_ODR(int32_t bsx_list_inx, float Hz) {
  if (Hz > 200) {
    return SMI240_ODR_400HZ;
  }
  if (Hz > 100 && Hz <= 200) {
    return SMI240_ODR_200HZ;
  }
  if (Hz > 50 && Hz <= 100) {
    return SMI240_ODR_100HZ;
  }
  if (Hz > 5 && Hz <= 50) {
    return SMI240_ODR_50HZ;
  }
  if (Hz <= 5) {
    return SMI240_ODR_5HZ;
  }

  return 0;
}

static void ap_config_phyACC(bsx_f32_t sample_rate, uint16_t fifo_data_len) {
  int32_t odr_Hz;

  if (ACC_CHIP_SMI240 == accl_chip) {
    if (SAMPLE_RATE_DISABLED == sample_rate) {
      PDEBUG("shutdown acc");
    } else {
      PDEBUG("set acc active");
      PINFO("set physical ACC rate %f", sample_rate);
      odr_Hz = SMI240_convert_ODR(SENSORLIST_INX_ACCELEROMETER, sample_rate);
      PDEBUG("set acc odr: %f", odr_Hz);
      sampling_interval = US_PER_SEC / odr_Hz;
    }
  }

  return;
}

static void ap_config_phyGYR(bsx_f32_t sample_rate, uint16_t fifo_data_len) {
  if (GYR_CHIP_SMI240 == gyro_chip) {
    if (SAMPLE_RATE_DISABLED == sample_rate) {
      PDEBUG("shutdown gyro");
    } else {
      PDEBUG("set gyro active");
      PINFO("set physical GYRO rate aligned with ACC");
    }
  }

  return;
}

static void ap_config_physensor(bsx_u32_t input_id, bsx_f32_t sample_rate,
                                uint16_t fifo_data_len) {
  int32_t ret = 0;

  if (BSX_PHYSICAL_SENSOR_ID_INVALID ==
      input_id) {  // TODO: library may has bug
    return;
  }

  switch (input_id) {
    case BSX_INPUT_ID_ACCELERATION:
      ap_config_phyACC(sample_rate, fifo_data_len);
      break;
    case BSX_INPUT_ID_ANGULARRATE:
      ap_config_phyGYR(sample_rate, fifo_data_len);
      break;
    default:
      PWARN("unknown input id: %d", input_id);
      ret = 0;
      break;
  }

  if (ret < 0) {
    PERR("write_sysfs() fail");
  }

  return;
}

/**
 *
 */
static void ap_send_config(int32_t bsx_list_inx) {
  const struct sensor_t *p_sensor;
  BSX_SENSOR_CONFIG *p_config;
  bsx_sensor_configuration_t bsx_config_output[2];
  int32_t bsx_supplier_id;
  int32_t list_inx_base;
  bsx_u32_t input_id;

  if (bsx_list_inx <= SENSORLIST_INX_AMBIENT_IAQ) {
    list_inx_base = SENSORLIST_INX_GAS_RESIST;
    p_config = BSX_sensor_config_nonwk;
  } else {
    list_inx_base = SENSORLIST_INX_WAKEUP_SIGNI_PRESSURE;
    p_config = BSX_sensor_config_wk;
  }

  bsx_supplier_id = convert_BSX_ListInx(bsx_list_inx);
  if (BSX_VIRTUAL_SENSOR_ID_INVALID == bsx_supplier_id) {
    PWARN("invalid list index: %d, when matching supplier id", bsx_list_inx);
    return;
  }

  {
    bsx_config_output[0].sensor_id = bsx_supplier_id;

    p_sensor = &(bosch_all_sensors[bsx_list_inx]);
    if (SENSOR_FLAG_ON_CHANGE_MODE == (p_sensor->flags & REPORTING_MODE_MASK)) {
      bsx_config_output[0].sample_rate =
          p_config[bsx_list_inx - list_inx_base].delay_onchange_Hz;
    } else {
      CONVERT_DATARATE_CODE(p_config[bsx_list_inx - list_inx_base].data_rate,
                            bsx_config_output[0].sample_rate);
    }

    /**
     * For test app, AR should be defined as a on change sensor,
     * But for BSX4 Library, only can send rate 0 on AR
     */
    if (SENSORLIST_INX_WAKEUP_ACTIVITY == bsx_list_inx) {
      bsx_config_output[0].sample_rate = 0;
    }
  }

  {
    switch (bsx_list_inx) {
      case SENSORLIST_INX_ACCELEROMETER:
        input_id = BSX_INPUT_ID_ACCELERATION;
        break;
      case SENSORLIST_INX_GYROSCOPE_UNCALIBRATED:
        input_id = BSX_INPUT_ID_ANGULARRATE;
        break;
      default:
        PERR("wrong list index: %d", bsx_list_inx);
        return;
    }

    ap_config_physensor(input_id, bsx_config_output[0].sample_rate,
                        p_config[bsx_list_inx - list_inx_base].fifo_data_len);
  }

  return;
}

/**
 *
 * @param bsx_list_inx
 */
static void ap_send_disable_config(int32_t bsx_list_inx) {
  int32_t bsx_supplier_id;
  bsx_u32_t input_id;

  bsx_supplier_id = convert_BSX_ListInx(bsx_list_inx);
  if (BSX_VIRTUAL_SENSOR_ID_INVALID == bsx_supplier_id) {
    PWARN("invalid list index: %d, when matching supplier id", bsx_list_inx);
    return;
  }

  {
    switch (bsx_list_inx) {
      case SENSORLIST_INX_ACCELEROMETER:
        input_id = BSX_INPUT_ID_ACCELERATION;
        break;
      case SENSORLIST_INX_GYROSCOPE_UNCALIBRATED:
        input_id = BSX_INPUT_ID_ANGULARRATE;
        break;
      default:
        PERR("wrong list index: %d", bsx_list_inx);
        return;
    }

    ap_config_physensor(input_id, SAMPLE_RATE_DISABLED, 0);
  }
  return;
}

static int32_t ap_activate(int32_t handle, int32_t enabled) {
  struct sensor_t *p_sensor;
  int32_t bsx_list_inx;
  int32_t ret;

  if (BSX_SENSOR_ID_INVALID == handle) {
    /*private sensors which not supported in library and Android yet*/
    return 0;
  }

  PDEBUG("ap_activate(handle: %d, enabled: %d)", handle, enabled);

  get_sensor_t(handle, &p_sensor, &bsx_list_inx);
  if (NULL == p_sensor) {
    PWARN("invalid id: %d", handle);
    return -EINVAL;
  }

  /*To adapt BSX4 algorithm's way of configuration string,
   * activate_configref_resort() is employed*/
  ret = activate_configref_resort(bsx_list_inx, enabled);
  ret = 1;  // force to control sensor irrespective of previous state
  if (ret) {
    if (enabled) {
      ap_send_config(bsx_list_inx);
    } else {
      ap_send_disable_config(bsx_list_inx);
    }
  }

  return 0;
}

static int32_t ap_batch(int32_t handle, int32_t flags,
                        int64_t sampling_period_ns,
                        int64_t max_report_latency_ns) {
  (void)flags;  // Deprecated in SENSORS_DEVICE_API_VERSION_1_3
  struct sensor_t *p_sensor;
  int32_t bsx_list_inx;
  int32_t ret;
  float delay_Hz_onchange = 0;

  // Sensors whoes fifoMaxEventCount==0 can be called batching according to spec
  get_sensor_t(handle, &p_sensor, &bsx_list_inx);
  if (NULL == p_sensor) {
    PWARN("invalid id: %d", handle);
    /*If maxReportingLatency is set to 0, this function must succeed*/
    if (0 == max_report_latency_ns) {
      return 0;
    } else {
      return -EINVAL;
    }
  }

  /*One-shot sensors are sometimes referred to as trigger sensors.
   The sampling_period_ns and max_report_latency_ns parameters
   passed to the batch function is ignored.
   Events from one-shot events cannot be stored in hardware FIFOs:
   the events must be reported as soon as they are generated.
   */
  if (SENSOR_FLAG_ONE_SHOT_MODE == (p_sensor->flags & REPORTING_MODE_MASK)) {
    return 0;
  }

  /*For special reporting-mode sensors, process each one according to spec*/
  if (SENSOR_FLAG_SPECIAL_REPORTING_MODE ==
      (p_sensor->flags & REPORTING_MODE_MASK)) {
    //...
    return 0;
  }

  PINFO(
      "batch(handle: %d, sampling_period_ns = %lld, max_report_latency = %lld)",
      handle, sampling_period_ns, max_report_latency_ns);

  /*
   For continuous and on-change sensors
   1. if sampling_period_ns is less than sensor_t.minDelay,
   then the HAL implementation must silently clamp it to max(sensor_t.minDelay,
   1ms). Android does not support the generation of events at more than 1000Hz.
   2. if sampling_period_ns is greater than sensor_t.maxDelay,
   then the HAL implementation must silently truncate it to sensor_t.maxDelay.
   */
  if (sampling_period_ns < (int64_t)(p_sensor->minDelay) * 1000LL) {
    sampling_period_ns = (int64_t)(p_sensor->minDelay) * 1000LL;
  } else if (sampling_period_ns > (int64_t)(p_sensor->maxDelay) * 1000LL) {
    sampling_period_ns = (int64_t)(p_sensor->maxDelay) * 1000LL;
  }

  /**
   * store sampling period as delay(us) for on change type sensor
   */
  if (SENSOR_FLAG_ON_CHANGE_MODE == (p_sensor->flags & REPORTING_MODE_MASK)) {
    delay_Hz_onchange = (float)1000000000LL / (float)sampling_period_ns;
    sampling_period_ns = 0;
  }

  /*In Android's perspective, a sensor can be configured no matter if it's
   active. To adapt BSX4 algorithm's way of configuration string,
   batch_configref_resort() is employed*/
  ret = batch_configref_resort(bsx_list_inx, sampling_period_ns,
                               max_report_latency_ns, delay_Hz_onchange);
  if (ret) {
    ap_send_config(bsx_list_inx);
  }

  return 0;
}

static int32_t ap_flush(BoschSensor *boschsensor, int32_t handle) {
  struct sensor_t *p_sensor;
  int32_t bsx_list_inx;

  get_sensor_t(handle, &p_sensor, &bsx_list_inx);
  if (NULL == p_sensor) {
    PWARN("invalid id: %d", handle);
    return -EINVAL;
  }

  /*flush does not apply to one-shot sensors:
   if sensor_handle refers to a one-shot sensor,
   flush must return -EINVAL and not generate any flush complete metadata event.
   */
  if (SENSOR_FLAG_ONE_SHOT_MODE == (p_sensor->flags & REPORTING_MODE_MASK)) {
    PWARN("invalid flags for id: %d", handle);
    return -EINVAL;
  }

  // Now the driver can not support this specification, so has to work around
  (void)boschsensor->send_flush_event(handle);

  /*If the specified sensor has no FIFO (no buffering possible),
   or if the FIFO was empty at the time of the call,
   flush must still succeed and send a flush complete event for that sensor.
   This applies to all sensors
   (except one-shot sensors, and already filted in former code).
   */

  return 0;
}

#define DIGITS_32BIT_INT 10
static void sysfs_extract_numbers(char *s, int32_t *x, int32_t *y, int32_t *z) {
  char *p, *stop;

  // 32bit signed int has 10 decimal digits
  // the format is fixed from sysfs node:
  // 1st is x value, 2nd for y, 3rd for z
  p = s;
  if (isdigit(*p) || (*p == '-'))
    *x = strtol(p, &stop, DIGITS_32BIT_INT);
  else
    PERR("data error");
  p = ++stop;

  if (isdigit(*p) || (*p == '-'))
    *y = strtol(p, &stop, DIGITS_32BIT_INT);
  else
    PERR("data error");
  p = ++stop;

  if (isdigit(*p) || (*p == '-'))
    *z = strtol(p, &stop, DIGITS_32BIT_INT);
  else
    PERR("data error");
}

static void ap_hw_poll_smi240acc(BoschSimpleList *dest_list_acc) {
  int32_t ret, x, y, z;
  char data[100];
  HW_DATA_UNION *p_hwdata;
  struct timespec timestamp;

  while ((ret = read(acc_fd, data, sizeof(data))) > 0) {
    data[ret] = '\0';

    clock_gettime(CLOCK_MONOTONIC, &timestamp);
    sysfs_extract_numbers(data, &x, &y, &z);
    PNOTE("acc data: x %d, y %d, z %d", x, y, z);

    p_hwdata = (HW_DATA_UNION *)calloc(1, sizeof(HW_DATA_UNION));
    if (NULL == p_hwdata) {
      PERR("malloc fail");
      continue;
    }

    p_hwdata->id = SENSOR_TYPE_ACCELEROMETER;
    p_hwdata->x = x;
    p_hwdata->y = y;
    p_hwdata->z = z;
    p_hwdata->timestamp = timestamp.tv_sec * 1000000000LL + timestamp.tv_nsec;

    ret = dest_list_acc->list_add_rear((void *)p_hwdata);
    if (ret) {
      PERR("list_add_rear() fail, ret = %d", ret);
      if (-1 == ret) {
        free(p_hwdata);
      }
    }
  }

  if (lseek(acc_fd, 0, SEEK_SET) == -1) {
    PERR("lseek error");
    close(acc_fd);
  }

  return;
}

static void ap_hw_poll_smi240gyro(BoschSimpleList *dest_list) {
  int32_t ret, x, y, z;
  char data[100];
  HW_DATA_UNION *p_hwdata;
  struct timespec timestamp;

  while ((ret = read(gyr_fd, data, sizeof(data))) > 0) {
    data[ret] = '\0';

    clock_gettime(CLOCK_MONOTONIC, &timestamp);
    sysfs_extract_numbers(data, &x, &y, &z);
    PNOTE("gyro data: x %d, y %d, z %d", x, y, z);

    p_hwdata = (HW_DATA_UNION *)calloc(1, sizeof(HW_DATA_UNION));
    if (NULL == p_hwdata) {
      PERR("malloc fail");
      continue;
    }

    p_hwdata->id = SENSOR_TYPE_GYROSCOPE_UNCALIBRATED;
    p_hwdata->x_uncalib = x;
    p_hwdata->y_uncalib = y;
    p_hwdata->z_uncalib = z;
    p_hwdata->timestamp = timestamp.tv_sec * 1000000000LL + timestamp.tv_nsec;

    hw_remap_sensor_data(&(p_hwdata->x_uncalib), &(p_hwdata->y_uncalib),
                         &(p_hwdata->z_uncalib), g_place_g);

    ret = dest_list->list_add_rear((void *)p_hwdata);
    if (ret) {
      PERR("list_add_rear() fail, ret = %d", ret);
      if (-1 == ret) {
        free(p_hwdata);
      }
    }
  }

  if (lseek(gyr_fd, 0, SEEK_SET) == -1) {
    PERR("lseek error");
    close(gyr_fd);
  }

  return;
}

/*
static void dump_samples(BoschSimpleList *al, BoschSimpleList *gl,
BoschSimpleList *ml)
{
    HW_DATA_UNION *p_hwdata;
    struct list_node *pnode;
    uint32_t i;

    pnode = al->head;
    for (i = 0; i < al->list_len; ++i) {
        p_hwdata = (HW_DATA_UNION *)(pnode->p_data);
        PNOTE("**ACCL tm = %lld", p_hwdata->timestamp);
        pnode = pnode->next;
    }

    pnode = gl->head;
    for (i = 0; i < gl->list_len; ++i) {
        p_hwdata = (HW_DATA_UNION *)(pnode->p_data);
        PNOTE("--GYRO tm = %lld", p_hwdata->timestamp);
        pnode = pnode->next;
    }

    PNOTE("==============================");

    return;
}
*/

static uint32_t IMU_hw_deliver_sensordata(BoschSensor *boschsensor) {
  int32_t ret;
  uint32_t j;
  struct pollfd poll_fds[2];

  if (ACC_CHIP_SMI240 == accl_chip) {
    poll_fds[0].fd = acc_fd;
    poll_fds[0].events = POLLIN;
  }

  if (GYR_CHIP_SMI240 == gyro_chip) {
    poll_fds[1].fd = gyr_fd;
    poll_fds[1].events = POLLIN;
  }

  for (j = 0; j < ARRAY_ELEMENTS(poll_fds); j++) {
    switch (j) {
      case 0:
        ap_hw_poll_smi240acc(boschsensor->tmplist_hwcntl_acclraw);
        break;

      case 1:
        /* this is an undefined place holder */
        ap_hw_poll_smi240gyro(boschsensor->tmplist_hwcntl_gyroraw);
        break;
    }
  }

#if 1
  if (boschsensor->tmplist_hwcntl_acclraw->list_len +
      boschsensor->tmplist_hwcntl_gyroraw->list_len) {
    pthread_mutex_lock(&(boschsensor->shmem_hwcntl.mutex));

    ret = boschsensor->shmem_hwcntl.p_list->list_mount_rear(
        boschsensor->tmplist_hwcntl_acclraw);
    if (ret) {
      PWARN("list mount fail");
    }

    ret = boschsensor->shmem_hwcntl.p_list->list_mount_rear(
        boschsensor->tmplist_hwcntl_gyroraw);
    if (ret) {
      PWARN("list mount fail");
    }

    pthread_cond_signal(&(boschsensor->shmem_hwcntl.cond));
    pthread_mutex_unlock(&(boschsensor->shmem_hwcntl.mutex));
  }
#endif
  usleep(sampling_interval - SAMPLING_INTERVAL_OFFSET);
  return 0;
}

static void ap_show_ver() {
  const char *accl_chip_name = NULL;
  const char *gyro_chip_name = NULL;
  const char *solution_name = NULL;
  char data_log_buf[256] = {0};

  switch (accl_chip) {
    case ACC_CHIP_SMI240:
      accl_chip_name = "SMI240ACC";
      break;
  }

  switch (gyro_chip) {
    case GYR_CHIP_SMI240:
      gyro_chip_name = "SMI240GYRO";
      break;
  }

  switch (solution_type) {
    case SOLUTION_MDOF:
      solution_name = "MDOF";
      break;
    case SOLUTION_ECOMPASS:
      solution_name = "ECOMPASS";
      gyro_chip_name = NULL;
      break;
    case SOLUTION_IMU:
      solution_name = "IMU";
      break;
    case SOLUTION_M4G:
      solution_name = "M4G";
      gyro_chip_name = NULL;
      break;
    case SOLUTION_ACC:
      solution_name = "ACC";
      gyro_chip_name = NULL;
      break;
    default:
      solution_name = "Unknown";
      accl_chip_name = NULL;
      gyro_chip_name = NULL;
      break;
  }

  PNOTE(
      "\n **************************\n \
            * HAL version: %d.%d.%d.%d\n \
            * build time: %s, %s\n \
            * solution type: %s\n \
            * accl_chip: %s\n \
            * gyro_chip: %s\n \
            **************************",
      HAL_ver[0], HAL_ver[1], HAL_ver[2], HAL_ver[3], __DATE__, __TIME__,
      solution_name, accl_chip_name, gyro_chip_name);

  if (bsx_datalog) {
    sprintf(data_log_buf, "<component>HAL version: %d.%d.%d.%d</component>\n",
            HAL_ver[0], HAL_ver[1], HAL_ver[2], HAL_ver[3]);
    bsx_datalog_algo(data_log_buf);
    sprintf(data_log_buf, "<component>build time: %s, %s</component>\n",
            __DATE__, __TIME__);
    bsx_datalog_algo(data_log_buf);
    sprintf(data_log_buf, "<component>solution type: %s</component>\n\n",
            solution_name);
    bsx_datalog_algo(data_log_buf);

    if (accl_chip_name) {
      sprintf(data_log_buf, "<device name=\"%s\" bus=\"IIC\">\n",
              accl_chip_name);
      bsx_datalog_algo(data_log_buf);
      sprintf(data_log_buf, "</device>\n");
      bsx_datalog_algo(data_log_buf);
    }

    if (gyro_chip_name) {
      sprintf(data_log_buf, "<device name=\"%s\" bus=\"IIC\">\n",
              gyro_chip_name);
      bsx_datalog_algo(data_log_buf);
      sprintf(data_log_buf, "</device>\n\n");
      bsx_datalog_algo(data_log_buf);
    }
  }

  return;
}

static int32_t ap_hwcntl_init_ACC() {
  if (ACC_CHIP_SMI240 == accl_chip) {
    acc_fd = open("/sys/bus/iio/devices/iio:device0/in_accel_x&y&z_raw",
                  O_RDONLY | O_NONBLOCK);
    if (-1 == acc_fd) {
      PERR("Failed to open file\n");
      return -ENODEV;
    }
    PNOTE("open smi240 iio acc device successfully\n");
  }

  return 0;
}

static int32_t ap_hwcntl_init_GYRO() {
  if (GYR_CHIP_SMI240 == gyro_chip) {
    gyr_fd = open("/sys/bus/iio/devices/iio:device0/in_anglvel_x&y&z_raw",
                  O_RDONLY | O_NONBLOCK);
    ;

    if (-1 == gyr_fd) {
      PERR("Failed to open dir\n");
      return -ENODEV;
    }

    PNOTE("open smi240 iio gyro device successfully\n");
  }

  return 0;
}

int32_t hwcntl_init(BoschSensor *boschsensor) {
  int32_t ret = 0;

  ap_show_ver();

  boschsensor->pfun_get_sensorlist = ap_get_sensorlist;
  boschsensor->pfun_activate = ap_activate;
  boschsensor->pfun_batch = ap_batch;
  boschsensor->pfun_flush = ap_flush;
  if (SOLUTION_IMU == solution_type) {
    boschsensor->pfun_hw_deliver_sensordata = IMU_hw_deliver_sensordata;
    ret = ap_hwcntl_init_ACC();
    ret = ap_hwcntl_init_GYRO();
  } else {
    PERR("Unkown solution type: %d", solution_type);
  }

  return ret;
}
