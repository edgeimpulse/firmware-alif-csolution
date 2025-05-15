/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _CAMERA_H_
#define _CAMERA_H_

#include <stdint.h>
#include <stdbool.h>
#include "RTE_Components.h"
#include "RTE_Device.h"
#include "model-parameters/model_metadata.h"

#if defined(__cplusplus)
extern "C" {
#endif

#if defined(RTE_Drivers_CAMERA_SENSOR_MT9M114) && (RTE_Drivers_CAMERA_SENSOR_MT9M114 == 1)
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
#elif defined (RTE_Drivers_CAMERA_SENSOR_ARX3A0) && (RTE_Drivers_CAMERA_SENSOR_ARX3A0 == 1)
#define CAM_FRAME_WIDTH        (RTE_ARX3A0_CAMERA_SENSOR_FRAME_WIDTH)
#define CAM_FRAME_HEIGHT       (RTE_ARX3A0_CAMERA_SENSOR_FRAME_HEIGHT)
#else
#error "Unsupported camera sensor"
#endif

#define CAM_BYTES_PER_PIXEL 
#define CAM_FRAME_SIZE (CAM_FRAME_WIDTH * CAM_FRAME_HEIGHT)
#define CAM_MPIX (CAM_FRAME_SIZE / 1000000.0f)
#define CAM_FRAME_SIZE_BYTES (CAM_FRAME_SIZE + CAM_USE_RGB565 * CAM_FRAME_SIZE)

extern int camera_init(void);
extern int camera_capture_frame(uint8_t* bufferm, uint16_t width, uint16_t height, bool to_rgb888);
extern void camera_get_max_res(uint16_t* width, uint16_t* height);
extern int camera_start_stream(void);

#if (defined(LCD_SUPPORTED) && (LCD_SUPPORTED == 1)) && (defined(EI_CLASSIFIER_SENSOR) && (EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_CAMERA))
extern void camera_stop_stream(void);
extern bool camera_capture_stream_frame(uint8_t* bufferm, uint16_t width, uint16_t height);
#endif

#if defined(__cplusplus)
}
#endif

#endif