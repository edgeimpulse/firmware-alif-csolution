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
#include "ingestion-sdk-platform/sensor/ei_camera.h"

/* Const defines ----------------------------------------------------------- */
#define EI_DEVICE_N_RESOLUTIONS     5
#define EI_STANDALONE_SENSORS_COUNT 1

/* Supported baud rates --------------------------------------------------- */
#define DEFAULT_BAUD    115200
#define MAX_BAUD        921600

class EiDeviceAlif : public EiDeviceInfo
{
private:
    EiAlifCamera *cam;
    static const int sensors_count = EI_STANDALONE_SENSORS_COUNT;
    ei_device_sensor_t sensors[sensors_count];
    EiState state;
    EiDeviceMemory *data_flash;

public:
    EiAlifCamera* get_camera(void) { return cam; };
    EiDeviceAlif(void);
    void init_device_id(void) override;
    void set_default_data_output_baudrate(void) override;
    void set_max_data_output_baudrate(void) override;
    EiSnapshotProperties get_snapshot_list(void) override;
    bool get_sensor_list(const ei_device_sensor_t **p_sensor_list, size_t *sensor_list_size) override;
};

#endif
