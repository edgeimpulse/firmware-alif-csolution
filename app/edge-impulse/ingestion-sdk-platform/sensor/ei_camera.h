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

#ifndef _EI_CAMERA_H_
#define _EI_CAMERA_H_

#include "firmware-sdk/ei_camera_interface.h"
#include "firmware-sdk/ei_device_info_lib.h"

class EiAlifCamera : public EiCamera
{
private:
    static ei_device_snapshot_resolutions_t resolutions[];

    bool camera_found;
    uint16_t width;
    uint16_t height;

public:
    bool is_camera_present(void) {return camera_found;};
    bool init(uint16_t width, uint16_t height) override;
    bool deinit(void) override; 
    void get_resolutions(ei_device_snapshot_resolutions_t **res, uint8_t *res_num) override;
    bool set_resolution(const ei_device_snapshot_resolutions_t res) override;
    ei_device_snapshot_resolutions_t get_min_resolution(void) override;

    bool ei_camera_capture_rgb888_packed_big_endian(
        uint8_t *image,
        uint32_t image_size) override;

    bool get_fb_ptr(uint8_t** fb_ptr) override;
};

#endif
