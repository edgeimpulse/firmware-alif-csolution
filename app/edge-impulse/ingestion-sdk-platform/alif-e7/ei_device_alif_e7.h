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

#ifndef EI_DEVICE_ALIF_E7
#define EI_DEVICE_ALIF_E7

/* Include ----------------------------------------------------------------- */
#include "firmware-sdk/ei_device_info_lib.h"
#include "firmware-sdk/ei_device_memory.h"

/* Const defines ----------------------------------------------------------- */
#define EI_DEVICE_N_RESOLUTIONS 5

/* Supported baud rates --------------------------------------------------- */
#define DEFAULT_BAUD 115200
#define MAX_BAUD 921600

class EiDeviceAlif : public EiDeviceInfo
{
private:
    ei_device_snapshot_resolutions_t snapshot_resolutions[EI_DEVICE_N_RESOLUTIONS];
    
public:
    void init_device_id(void);
    void set_default_data_output_baudrate(void);
    void set_max_data_output_baudrate(void);
    bool get_snapshot_list(const ei_device_snapshot_resolutions_t **resolution_list, size_t *resolution_list_size,
                const char **color_depth);
    bool get_sensor_list(const ei_device_sensor_t **p_sensor_list, size_t *sensor_list_size);
};

#endif