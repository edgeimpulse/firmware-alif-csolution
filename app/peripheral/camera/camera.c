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

#include "RTE_Components.h"
#include CMSIS_device_header
#include "camera.h"
#include "Driver_CPI.h"    // Camera
#include "bayer.h"
#include "image_processing.h"

#define BAYER_FORMAT DC1394_COLOR_FILTER_GRBG

/* Camera Controller Resolution. */
#define CAM_RESOLUTION                   CAM_RESOLUTION_560x560
#if (CAM_RESOLUTION == CAM_RESOLUTION_560x560)
#define CAM_FRAME_WIDTH        (560)
#define CAM_FRAME_HEIGHT       (560)
#elif (CAM_RESOLUTION == CAM_RESOLUTION_480x480)
#define CAM_FRAME_WIDTH        (480)
#define CAM_FRAME_HEIGHT       (480)
#endif
#define BYTES_PER_PIXEL        (3)

#define CAM_FRAME_SIZE (CAM_FRAME_WIDTH * CAM_FRAME_HEIGHT)
#define CAM_MPIX (CAM_FRAME_SIZE / 1000000.0f)

static uint8_t camera_buffer[CAM_FRAME_SIZE] __attribute__((aligned(32), section(".bss.camera_frame_buf")));
static uint8_t image_buffer[CAM_FRAME_SIZE * BYTES_PER_PIXEL] __attribute__((aligned(32), section(".bss.camera_frame_bayer_to_rgb_buf")));

/* Camera  Driver instance 0 */
extern ARM_DRIVER_CPI Driver_CPI;
static ARM_DRIVER_CPI *CAMERAdrv = &Driver_CPI;

typedef enum {
    CAM_CB_EVENT_NONE            = 0,
    CAM_CB_EVENT_ERROR           = (1 << 0),
    DISP_CB_EVENT_ERROR          = (1 << 1),
    CAM_CB_EVENT_CAPTURE_STOPPED = (1 << 2)
} CB_EVENTS;

static volatile CB_EVENTS g_cb_events = CAM_CB_EVENT_NONE;

static void camera_callback(uint32_t event)
{
    switch (event)
    {
    case ARM_CPI_EVENT_CAMERA_CAPTURE_STOPPED:
        g_cb_events |= CAM_CB_EVENT_CAPTURE_STOPPED;
        break;
    case ARM_CPI_EVENT_CAMERA_FRAME_HSYNC_DETECTED:
        break;
    case ARM_CPI_EVENT_CAMERA_FRAME_VSYNC_DETECTED:
        break;

    case ARM_CPI_EVENT_ERR_CAMERA_INPUT_FIFO_OVERRUN:
    case ARM_CPI_EVENT_ERR_CAMERA_OUTPUT_FIFO_OVERRUN:
    case ARM_CPI_EVENT_ERR_HARDWARE:
    case ARM_CPI_EVENT_MIPI_CSI2_ERROR:
    default:
        g_cb_events |= CAM_CB_EVENT_ERROR;
        break;
    }
}

/**
 * @brief Intialise camera
 * 
 * @return int 
 */
int camera_init(void)
{
    int ret = CAMERAdrv->Initialize(camera_callback);
    if(ret != ARM_DRIVER_OK)
    {
        //printf("\r\n Error: CAMERA Initialize failed.\r\n");
        return ret;
    }

    /* Power up Camera peripheral */
    ret = CAMERAdrv->PowerControl(ARM_POWER_FULL);
    if(ret != ARM_DRIVER_OK)
    {
        //printf("\r\n Error: CAMERA Power Up failed.\r\n");
        return ret;
    }

    /* Control configuration for camera controller */
    ret = CAMERAdrv->Control(CPI_CONFIGURE, 0);
    if(ret != ARM_DRIVER_OK)
    {
        //printf("\r\n Error: CPI Configuration failed.\r\n");
        return ret;
    }

    /* Control configuration for camera sensor */
    ret = CAMERAdrv->Control(CPI_CAMERA_SENSOR_CONFIGURE, 0);
    if(ret != ARM_DRIVER_OK)
    {
        //printf("\r\n Error: CAMERA SENSOR Configuration failed.\r\n");
        return ret;
    }

    /*Control configuration for camera events */
    ret = CAMERAdrv->Control(CPI_EVENTS_CONFIGURE,
                             ARM_CPI_EVENT_CAMERA_CAPTURE_STOPPED |
                             ARM_CPI_EVENT_ERR_CAMERA_INPUT_FIFO_OVERRUN |
                             ARM_CPI_EVENT_ERR_CAMERA_OUTPUT_FIFO_OVERRUN |
                             ARM_CPI_EVENT_ERR_HARDWARE);
    if(ret != ARM_DRIVER_OK)
    {
        //printf("\r\n Error: CAMERA SENSOR Event Configuration failed.\r\n");
        return ret;
    }

    return ret;
}

/**
 * @brief 
 * 
 * @param buffer 
 * @return int 
 */
int camera_capture_frame(uint8_t* bufferm,uint16_t width, uint16_t height)
{
    int ret;

    g_cb_events = CAM_CB_EVENT_NONE;

    ret = CAMERAdrv->CaptureFrame(camera_buffer);

    if (ret != ARM_DRIVER_OK) {
        return ret;
    }
    // Wait for capture
    while (!(g_cb_events & CAM_CB_EVENT_CAPTURE_STOPPED)) {
        __WFI();
    }
    // Invalidate cache before reading the camera_buffer
    SCB_CleanInvalidateDCache();

    dc1394_bayer_Simple(camera_buffer, image_buffer, CAM_FRAME_WIDTH, CAM_FRAME_HEIGHT, BAYER_FORMAT);
    // White balance function does also RGB --> BGR conversion
    white_balance(CAM_FRAME_WIDTH, CAM_FRAME_HEIGHT, image_buffer, image_buffer);

    const int rescaleWidth = width;
    const int rescaleHeight = CAM_FRAME_HEIGHT * (float)rescaleWidth / CAM_FRAME_WIDTH;

    resize_image_A(image_buffer,
                           CAM_FRAME_WIDTH,
                           CAM_FRAME_HEIGHT,
                           (uint8_t*)bufferm,
                           width,
                           height,
                           BYTES_PER_PIXEL);
    return ret;
}

int camera_start_stream()
{
    int ret;

    g_cb_events = CAM_CB_EVENT_NONE;

    ret = CAMERAdrv->CaptureVideo(camera_buffer);

    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    return ret;
}