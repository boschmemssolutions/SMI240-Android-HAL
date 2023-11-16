/* SPDX-License-Identifier: Apache-2.0 */
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

#ifndef __SENSORD_ALGO_H
#define __SENSORD_ALGO_H

#include "BoschSensor.h"
#ifdef __cplusplus
extern "C" {
#endif

#include "bsx_android.h"
#include "bsx_constant.h"
#include "bsx_datatypes.h"
#include "bsx_physical_sensor_identifier.h"
#include "bsx_property_set_identifier.h"
#include "bsx_return_value_identifier.h"
#include "bsx_virtual_sensor_identifier.h"

#ifdef __cplusplus
}
#endif

#define SAMPLE_RATE_DISABLED 65535.f
#define BST_DLOG_ID_START 256
#define BST_DLOG_ID_SUBSCRIBE_OUT BST_DLOG_ID_START
#define BST_DLOG_ID_SUBSCRIBE_IN (BST_DLOG_ID_START + 1)
#define BST_DLOG_ID_DOSTEP (BST_DLOG_ID_START + 2)
#define BST_DLOG_ID_ABANDON (BST_DLOG_ID_START + 3)
#define BST_DLOG_ID_NEWSAMPLE (BST_DLOG_ID_START + 4)

extern void sensord_algo_process(BoschSensor *boschsensor);

#endif
