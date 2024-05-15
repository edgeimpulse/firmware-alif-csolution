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

/* Includes ---------------------------------------------------------------- */
#include "ei_device_alif_e7.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "ei_utils.h"
#include "peripheral/ei_uart.h"

/** Data Output Baudrate */
const ei_device_data_output_baudrate_t ei_dev_max_data_output_baudrate = {
    ei_xstr(MAX_BAUD),
    MAX_BAUD,
};

const ei_device_data_output_baudrate_t ei_dev_default_data_output_baudrate = {
    ei_xstr(DEFAULT_BAUD),
    DEFAULT_BAUD,
};

static ei_device_sensor_t sensor_list[] = {
    { 
        .name = "Microphone",
        .frequencies = { 16000.0 },
        .max_sample_length_s = 2,
        //.start_sampling_cb = ei_microphone_sample_record
    }
};

/**
 * @brief 
 * 
 */
void EiDeviceAlif::init_device_id(void)
{
    device_id = "alif-e7";
}

/**
 * @brief      Set output baudrate to max
 *
 */
void EiDeviceAlif::set_max_data_output_baudrate(void)
{
    ei_uart_init(MAX_BAUD);
}

/**
 * @brief      Set output baudrate to default
 *
 */
void EiDeviceAlif::set_default_data_output_baudrate(void)
{
    ei_uart_init(DEFAULT_BAUD);
}

/**
 * @brief      Create resolution list for snapshot setting
 *             The studio and daemon require this list
 * @param      snapshot_list       Place pointer to resolution list
 * @param      snapshot_list_size  Write number of resolutions here
 *
 * @return     False if all went ok
 */
bool EiDeviceAlif::get_snapshot_list(const ei_device_snapshot_resolutions_t **snapshot_list, size_t *snapshot_list_size,
                                         const char **color_depth)
{
    snapshot_resolutions[0].width = 64;
    snapshot_resolutions[0].height = 64;
    snapshot_resolutions[1].width = 96;
    snapshot_resolutions[1].height = 96;
    snapshot_resolutions[2].width = 160;
    snapshot_resolutions[2].height = 160;
    snapshot_resolutions[3].width = 320;
    snapshot_resolutions[3].height = 320;
    snapshot_resolutions[4].width = 480;
    snapshot_resolutions[4].height = 480;

    *snapshot_list      = snapshot_resolutions;
    *snapshot_list_size = EI_DEVICE_N_RESOLUTIONS;
    *color_depth = "RGB";

    return false;
}

bool EiDeviceAlif::get_sensor_list(const ei_device_sensor_t **p_sensor_list, size_t *sensor_list_size)
{
    *p_sensor_list = sensor_list;
    *sensor_list_size = ARRAY_LENGTH(sensor_list);
    return true;
}

/**
 * @brief get_device is a static method of EiDeviceInfo class
 * It is used to implement singleton paradigm, so we are returning
 * here pointer always to the same object (dev)
 * 
 * @return EiDeviceInfo* 
 */
EiDeviceInfo* EiDeviceInfo::get_device(void)
{
    static EiDeviceAlif dev;

    return &dev;
}
