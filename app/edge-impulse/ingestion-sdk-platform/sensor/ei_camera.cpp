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

#include "ei_camera.h"
#include "peripheral/camera/camera.h"

ei_device_snapshot_resolutions_t EiAlifCamera::resolutions[] = {
        {96, 64},
        {96, 96},
        {160, 120},
        {160, 160},
        {240, 240},
        {320, 240},
    };

/**
 * @brief 
 * 
 * @param width 
 * @param height 
 * @return true 
 * @return false 
 */
bool EiAlifCamera::init(uint16_t width, uint16_t height)
{
    if (camera_init() != 0) {
        return false;
    }
    camera_found = true;

    return true;
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool EiAlifCamera::deinit(void)
{
    return true;
}

/**
 * @brief 
 * 
 * @param res 
 * @param res_num 
 */
void EiAlifCamera::get_resolutions(ei_device_snapshot_resolutions_t **res, uint8_t *res_num)
{
    *res = &EiAlifCamera::resolutions[0];
    *res_num = sizeof(EiAlifCamera::resolutions) / sizeof(ei_device_snapshot_resolutions_t);
}

/**
 * @brief 
 * 
 * @param res 
 * @return true 
 * @return false 
 */
bool EiAlifCamera::set_resolution(const ei_device_snapshot_resolutions_t res)
{
    this->width = res.width;
    this->height = res.height;
    
    return true;
}

bool EiAlifCamera::ei_camera_capture_rgb888_packed_big_endian(
    uint8_t *image,
    uint32_t image_size)
{
    return camera_capture_frame(image) == 0;
}

bool EiAlifCamera::get_fb_ptr(uint8_t** fb_ptr)
{
    // Not implemented
    return false;
}

/**
 * @brief 
 * 
 * @return ei_device_snapshot_resolutions_t 
 */
ei_device_snapshot_resolutions_t EiAlifCamera::get_min_resolution(void)
{
    return EiAlifCamera::resolutions[0];
}

/**
 *
 * @return
 */
EiCamera* EiCamera::get_camera(void)
{
    static EiAlifCamera cam;

    return &cam;
}
