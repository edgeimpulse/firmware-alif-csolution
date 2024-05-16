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
    return true;
}

/**
 * @brief 
 * 
 * @return ei_device_snapshot_resolutions_t 
 */
ei_device_snapshot_resolutions_t EiAlifCamera::get_min_resolution(void)
{
    return { 96,96 };
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
