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

/* Include ----------------------------------------------------------------- */
#include "ei_inertial.h"
#include "peripheral/inertial/bmi323_icm42670.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

/* Constant defines -------------------------------------------------------- */
#define CONVERT_G_TO_MS2 (9.80665f)
#define ACC_RAW_SCALING  (32767.5f)

#define ACC_SCALE_FACTOR (2.0f * CONVERT_G_TO_MS2) / ACC_RAW_SCALING
#define CONVERT_ADC_GYR (float)(250.0f / 32768.0f)

/* Private variables ------------------------------------------------------- */
static float imu_data[INERTIAL_AXIS_SAMPLED];
static float imu_data_bmi323[INERTIAL_AXIS_SAMPLED];

/**
 * @brief      Setup payload header
 *
 * @return     true
 */
bool ei_inertial_init(void)
{
    if (bmi323_icm42670_init()) {
        ei_printf("Failed to initialize inertial sensor\r\n");
        return false;
    }
    else {
        ei_add_sensor_to_fusion_list(inertial_sensor_icm42670);
        ei_add_sensor_to_fusion_list(inertial_sensor_bmi323);
        return true;
    }

    
}

/**
 * @brief
 *
 * @param n_samples
 * @return float*
 */
float *ei_fusion_inertial_read_data_icm42670(int n_samples)
{
    xyz_accel_gyro_imu_s icm_data;

    memset(imu_data, 0, sizeof(imu_data));

    getICM42670PAllSensors(&icm_data);
    imu_data[0] = (float)icm_data.acc_x * ACC_SCALE_FACTOR;
    imu_data[1] = (float)icm_data.acc_y * ACC_SCALE_FACTOR;
    imu_data[2] = (float)icm_data.acc_z * ACC_SCALE_FACTOR;

    if ((n_samples > 3)) {
        imu_data[3] = (float)icm_data.gyro_x * CONVERT_ADC_GYR;
        imu_data[4] = (float)icm_data.gyro_y * CONVERT_ADC_GYR;
        imu_data[5] = (float)icm_data.gyro_z * CONVERT_ADC_GYR;
    }

    return imu_data;
}

/**
 * @brief 
 * 
 * @param n_samples 
 * @return float* 
 */
float* ei_fusion_inertial_read_data_bmi323(int n_samples)
{
    xyz_accel_gyro_imu_s bmi_data;

    memset(imu_data_bmi323, 0, sizeof(imu_data_bmi323));

    getBMI323AllSensors(&bmi_data);
    imu_data_bmi323[0] = (float)bmi_data.acc_x * ACC_SCALE_FACTOR;
    imu_data_bmi323[1] = (float)bmi_data.acc_y * ACC_SCALE_FACTOR;
    imu_data_bmi323[2] = (float)bmi_data.acc_z * ACC_SCALE_FACTOR;

    if ((n_samples > 3)) {
        imu_data_bmi323[3] = (float)bmi_data.gyro_x * CONVERT_ADC_GYR;
        imu_data_bmi323[4] = (float)bmi_data.gyro_y * CONVERT_ADC_GYR;
        imu_data_bmi323[5] = (float)bmi_data.gyro_z * CONVERT_ADC_GYR;
    }

    return imu_data_bmi323;
}
