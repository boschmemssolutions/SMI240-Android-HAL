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

#ifndef __SENSORD_HWCNTL_IIO_H
#define __SENSORD_HWCNTL_IIO_H

#define IIO_NAME_MAXLEN 30
#define MAX_FILENAME_LEN 256

static inline int wr_sysfs_twoint(const char *filename, char *basedir, int val1,
                                  int val2) {
  FILE *fp;
  char fname_buf[MAX_FILENAME_LEN + 1];

  snprintf(fname_buf, MAX_FILENAME_LEN, "%s/%s", basedir, filename);

  fp = fopen(fname_buf, "w");
  if (NULL == fp) {
    return -errno;
  }

  fprintf(fp, "%d %d", val1, val2);
  fclose(fp);

  return 0;
}

static inline int wr_sysfs_oneint(const char *filename, char *basedir,
                                  int val) {
  FILE *fp;
  char fname_buf[MAX_FILENAME_LEN + 1];

  snprintf(fname_buf, MAX_FILENAME_LEN, "%s/%s", basedir, filename);

  fp = fopen(fname_buf, "w");
  if (NULL == fp) {
    return -errno;
  }

  fprintf(fp, "%d", val);
  fclose(fp);

  return 0;
}

static inline int wr_sysfs_str(const char *filename, char *basedir,
                               const char *str) {
  FILE *fp;
  char fname_buf[MAX_FILENAME_LEN + 1];

  snprintf(fname_buf, MAX_FILENAME_LEN, "%s/%s", basedir, filename);

  fp = fopen(fname_buf, "w");
  if (NULL == fp) {
    return -errno;
  }

  fprintf(fp, "%s", str);
  fclose(fp);

  return 0;
}

static inline int rd_sysfs_oneint(const char *filename, char *basedir,
                                  int *pval) {
  FILE *fp;
  char fname_buf[MAX_FILENAME_LEN + 1];
  int ret;

  snprintf(fname_buf, MAX_FILENAME_LEN, "%s/%s", basedir, filename);

  fp = fopen(fname_buf, "r");
  if (NULL == fp) {
    return -errno;
  }

  ret = fscanf(fp, "%d\n", pval);
  fclose(fp);

  if (ret <= 0) {
    return -errno;
  }

  return 0;
}

#endif /* __SENSORD_HWCNTL_IIO_H */
