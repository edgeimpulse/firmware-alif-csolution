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

#ifndef _CAMERA_H_
#define _CAMERA_H_

#include <stdint.h>
#include <stdbool.h>
#include "RTE_Device.h"

#if defined(__cplusplus)
extern "C" {
#endif

#if RTE_Drivers_CAMERA_SENSOR_MT9M114
#if (RTE_MT9M114_CAMERA_SENSOR_MIPI_IMAGE_CONFIG == 2)
    #define CAM_FRAME_WIDTH        (1280)
    #define CAM_FRAME_HEIGHT       (720)
#elif (RTE_MT9M114_CAMERA_SENSOR_MIPI_IMAGE_CONFIG == 3)
    #define CAM_FRAME_WIDTH        (640)
    #define CAM_FRAME_HEIGHT       (480)
#elif (RTE_MT9M114_CAMERA_SENSOR_MIPI_IMAGE_CONFIG == 4)
    #define CAM_FRAME_WIDTH        (320)
    #define CAM_FRAME_HEIGHT       (240)
#elif (RTE_MT9M114_CAMERA_SENSOR_MIPI_IMAGE_CONFIG == 5)
    #define CAM_FRAME_WIDTH        (320)
    #define CAM_FRAME_HEIGHT       (320)
#else
    #error "Unsupported MT9M114 configuration"
#endif
#else
#define CAM_FRAME_WIDTH        (RTE_ARX3A0_CAMERA_SENSOR_FRAME_WIDTH)
#define CAM_FRAME_HEIGHT       (RTE_ARX3A0_CAMERA_SENSOR_FRAME_HEIGHT)
#endif

#define CAM_BYTES_PER_PIXEL 
#define CAM_FRAME_SIZE (CAM_FRAME_WIDTH * CAM_FRAME_HEIGHT)
#define CAM_MPIX (CAM_FRAME_SIZE / 1000000.0f)
#define CAM_FRAME_SIZE_BYTES (CAM_FRAME_SIZE + CAM_USE_RGB565 * CAM_FRAME_SIZE)

extern int camera_init(void);
extern int camera_capture_frame(uint8_t* bufferm,uint16_t width, uint16_t height, bool swap_BGR);
extern void camera_get_max_res(uint16_t* width, uint16_t* height);

#if defined(__cplusplus)
}
#endif

#endif