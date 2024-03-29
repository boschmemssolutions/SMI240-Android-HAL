/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Copyright (C) 2023 Robert Bosch GmbH. All rights reserved.
 * Copyright (C) 2011~2015 Bosch Sensortec GmbH All Rights Reserved
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

/*!@addtogroup bsx_external
 * @{*/

#ifndef __BSX_VECTOR_INDEX_IDENTIFIER_H__
#define __BSX_VECTOR_INDEX_IDENTIFIER_H__

#ifdef __cplusplus
extern "C" {
#endif

/*! @brief BSX vector indices */
typedef enum {
  BSX_ID_ROTATION_QUATERNION_X = (0),
  BSX_ID_ROTATION_QUATERNION_Y = (1),
  BSX_ID_ROTATION_QUATERNION_Z = (2),
  BSX_ID_ROTATION_QUATERNION_W = (3),
  BSX_ID_ROTATION_STATUS = (4),
  BSX_ID_ORIENTATION_EULERANGLE_YAW = (0),
  BSX_ID_ORIENTATION_EULERANGLE_PITCH = (1),
  BSX_ID_ORIENTATION_EULERANGLE_ROLL = (2),
  BSX_ID_ORIENTATION_EULERANGLE_HEADING = (3),
  BSX_ID_ORIENTATION_STATUS = (4),
  BSX_ID_MOTION_VECTOR_X = (0),
  BSX_ID_MOTION_VECTOR_Y = (1),
  BSX_ID_MOTION_VECTOR_Z = (2),
  BSX_ID_MOTION_STATUS = (3)
} bsx_vector_index_identifier_t;

#ifdef __cplusplus
}
#endif

#endif /*__BSX_VECTOR_INDEX_IDENTIFIER_H__*/
/*! @}*/
