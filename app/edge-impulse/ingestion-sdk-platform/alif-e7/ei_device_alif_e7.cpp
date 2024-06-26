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
#include "ingestion-sdk-platform/sensor/ei_microphone.h"

/** Data Output Baudrate */
const ei_device_data_output_baudrate_t ei_dev_max_data_output_baudrate = {
    ei_xstr(MAX_BAUD),
    MAX_BAUD,
};

const ei_device_data_output_baudrate_t ei_dev_default_data_output_baudrate = {
    ei_xstr(DEFAULT_BAUD),
    DEFAULT_BAUD,
};

EiDeviceAlif::EiDeviceAlif(EiDeviceMemory* mem)
{
    EiDeviceInfo::memory = mem;

    init_device_id();
    load_config();

    device_type = "ALIF_E7_APPKIT_GEN2";

#if defined (MIC_ENABLED)
    /* Init standalone sensor */
    sensors[0].name = "Microphone";
    sensors[0].frequencies[0] = 16000;
    sensors[0].max_sample_length_s = 2;
    sensors[0].start_sampling_cb = ei_microphone_sample_record;
#endif

    /* Init camera instance */
    cam = static_cast<EiAlifCamera*>(EiCamera::get_camera());
}

EiDeviceAlif::~EiDeviceAlif()
{
}

/**
 * @brief 
 * 
 */
void EiDeviceAlif::init_device_id(void)
{
    // TODO read id
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
EiSnapshotProperties EiDeviceAlif::get_snapshot_list(void)
{
    ei_device_snapshot_resolutions_t **res;
    uint8_t res_num;

    cam->get_resolutions(res, &res_num);

    EiSnapshotProperties props = {
        .has_snapshot = true,
        .support_stream = true,
        .color_depth = "RGB",
        .resolutions_num = res_num,
        .resolutions = *res
    };

    return props;
}

bool EiDeviceAlif::get_sensor_list(const ei_device_sensor_t **p_sensor_list, size_t *sensor_list_size)
{
    *p_sensor_list = sensors;
    *sensor_list_size = ARRAY_LENGTH(sensors);
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
    static EiDeviceRAM<1024, 32> memory(sizeof(EiConfig));
    static EiDeviceAlif dev(&memory);

    return &dev;
}
