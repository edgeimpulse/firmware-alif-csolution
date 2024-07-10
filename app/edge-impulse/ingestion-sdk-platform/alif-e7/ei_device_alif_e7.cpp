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
#include "peripheral/timer.h"
#include "se_services_port.h"

/* Private variables ------------------------------------------------------- */

extern uint32_t        se_services_s_handle;

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
    char temp[20];
    
    uint64_t id = 0;
    uint32_t error_code = SERVICES_REQ_SUCCESS;
    uint32_t service_error_code;

    uint8_t eui[5] = {0x04, 0x03, 0x02, 0x01, 0x00};

    SERVICES_system_get_eui_extension(se_services_s_handle, false, eui, &service_error_code);

    snprintf(temp, sizeof(temp), "%02x:%02x:%02x:%02x:%02x",
        eui[4],
        eui[3],
        eui[2],
        eui[1],
        eui[0]);
    device_id = std::string(temp);
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
    static EiDeviceRAM<4096, 32> memory(sizeof(EiConfig));
    static EiDeviceAlif dev(&memory);

    return &dev;
}

bool EiDeviceAlif::start_sample_thread(void (*sample_read_cb)(void), float sample_interval_ms)
{
    this->is_sampling = true;
    this->sample_read_callback = sample_read_cb;
    this->sample_interval_ms = sample_interval_ms;

#if MULTI_FREQ_ENABLED == 1
    this->actual_timer = 0;
    this->fusioning = 1;        //
#endif
    
    timer_sensor_start(this->sample_interval_ms);

    return true;
}

bool EiDeviceAlif::stop_sample_thread(void)
{
    timer_sensor_stop();

    this->set_state(eiStateIdle);
    this->is_sampling = false;

    return true;
}

void EiDeviceAlif::sample_thread(void)
{       
#if MULTI_FREQ_ENABLED == 1
    if (this->fusioning == 1) {
        if (timer_sensor_get() == true)
        {
            if (this->sample_read_callback != nullptr) {
                this->sample_read_callback();
            }
        }
    }
    else {
        uint8_t flag = 0;
        uint8_t i = 0;

        this->actual_timer += (uint32_t)this->sample_interval;

        if (timer_sensor_get() == true) {

            for (i = 0; i < this->fusioning; i++) {
                if (((uint32_t)(this->actual_timer % (uint32_t)this->multi_sample_interval[i])) == 0) {
                    flag |= (1<<i);
                }
            }

            if (this->sample_multi_read_callback != nullptr) {
                this->sample_multi_read_callback(flag);
            }
        }
    }
#else
    if (timer_sensor_get() == true)
    {
        if (this->sample_read_callback != nullptr) {
            this->sample_read_callback();
        }
    }

#endif
}

#if MULTI_FREQ_ENABLED == 1
/**
 *
 * @param sample_read_cb
 * @param multi_sample_interval_ms
 * @param num_fusioned
 * @return
 */
bool EiDeviceAlif::start_multi_sample_thread(void (*sample_multi_read_cb)(uint8_t), float* multi_sample_interval_ms, uint8_t num_fusioned)
{
    uint8_t i;
    uint8_t flag = 0;

    this->is_sampling = true;
    this->sample_multi_read_callback = sample_multi_read_cb;
    this->fusioning = num_fusioned;

    this->multi_sample_interval.clear();

    for (i = 0; i < num_fusioned; i++) {
        this->multi_sample_interval.push_back(1.f/multi_sample_interval_ms[i]*1000.f);
    }

    this->sample_interval = ei_fusion_calc_multi_gcd(this->multi_sample_interval.data(), this->fusioning);

    /* force first reading */
    for (i = 0; i < this->fusioning; i++) {
            flag |= (1<<i);
    }
    this->sample_multi_read_callback(flag);

    this->actual_timer = 0;
    timer_sensor_start((uint32_t)this->sample_interval);

    return true;
}
#endif