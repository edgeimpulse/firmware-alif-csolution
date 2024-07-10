/*
 * Copyright (c) 2024 EdgeImpulse Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef EI_INERTIAL_SENSOR
#define EI_INERTIAL_SENSOR

/* Include ----------------------------------------------------------------- */
#include "firmware-sdk/ei_fusion.h"

/** Number of axis used and sample data format */
typedef float sample_format_t;

#define INERTIAL_AXIS_SAMPLED   6

/* Function prototypes ----------------------------------------------------- */
bool ei_inertial_init(void);
float *ei_fusion_inertial_read_data_icm42670(int n_samples);
float* ei_fusion_inertial_read_data_bmi323(int n_samples);

static const ei_device_fusion_sensor_t inertial_sensor_icm42670 = {
    "TDK icm42670 (Inertial)",
    // number of sensor module axis
    INERTIAL_AXIS_SAMPLED,
    // sampling frequencies
    { 20.0f, 62.5f, 100.0f },
    // axis name and units payload (must be same order as read in)
    { { "accX", "m/s2" },
      { "accY", "m/s2" },
      { "accZ", "m/s2" },
      { "gyrX", "dps" },
      { "gyrY", "dps" },
      { "gyrZ", "dps" } },
    // reference to read data function
    &ei_fusion_inertial_read_data_icm42670,
    0
};

static const ei_device_fusion_sensor_t inertial_sensor_bmi323 = {
    "Bosch bmi323 (Inertial)",
    // number of sensor module axis
    INERTIAL_AXIS_SAMPLED,
    // sampling frequencies
    { 20.0f, 62.5f, 100.0f },
    // axis name and units payload (must be same order as read in)
    { { "accX_bosch", "m/s2" },
      { "accY_bosch", "m/s2" },
      { "accZ_bosch", "m/s2" },
      { "gyrX_bosch", "dps" },
      { "gyrY_bosch", "dps" },
      { "gyrZ_bosch", "dps" } },
    // reference to read data function
    &ei_fusion_inertial_read_data_bmi323,
    0
};

#endif