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
#if RTE_Drivers_CAMERA_SENSOR_MT9M114
#define CAM_FRAME_WIDTH        (RTE_MT9M114_CAMERA_SENSOR_MIPI_FRAME_WIDTH)
#define CAM_FRAME_HEIGHT       (RTE_MT9M114_CAMERA_SENSOR_MIPI_FRAME_HEIGHT)
#define CAM_COLOR_CORRECTION   (0)
#define CAM_USE_RGB565         (RTE_MT9M114_CAMERA_SENSOR_MIPI_CSI_DATA_TYPE == 0x22)
#define RGB_BUFFER_SECTION     ".bss.camera_frame_bayer_to_rgb_buf_at_sram0"
#elif RTE_Drivers_CAMERA_SENSOR_ARX3A0
#define CAM_FRAME_WIDTH        (RTE_ARX3A0_CAMERA_SENSOR_FRAME_WIDTH)
#define CAM_FRAME_HEIGHT       (RTE_ARX3A0_CAMERA_SENSOR_FRAME_HEIGHT)
#define CAM_COLOR_CORRECTION   (1)
#define CAM_USE_RGB565         (0)
#define RGB_BUFFER_SECTION     ".bss.camera_frame_bayer_to_rgb_buf"
#endif

#define CAM_BYTES_PER_PIXEL 
#define CAM_FRAME_SIZE (CAM_FRAME_WIDTH * CAM_FRAME_HEIGHT)
#define CAM_MPIX (CAM_FRAME_SIZE / 1000000.0f)
#define CAM_FRAME_SIZE_BYTES (CAM_FRAME_SIZE + CAM_USE_RGB565 * CAM_FRAME_SIZE)

static uint8_t camera_buffer[CAM_FRAME_SIZE_BYTES] __attribute__((aligned(32), section(".bss.camera_frame_buf")));

// Buffer for bayer conversion to RGB is not needed when camera outputs RGB565
#if CAM_USE_RGB565
static const uint8_t *get_resize_source_buffer() { return camera_buffer; }
#else
static uint8_t image_buffer[CAM_FRAME_SIZE * RGB_BYTES] __attribute__((aligned(32), section(RGB_BUFFER_SECTION)));
static const uint8_t *get_resize_source_buffer() { return image_buffer; }
#endif

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
static bool camera_is_init = false;

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
    if (camera_is_init == true) {
        return 0;
    }

    int ret = CAMERAdrv->Initialize(camera_callback);
    if (ret != ARM_DRIVER_OK) {
        //printf("\r\n Error: CAMERA Initialize failed.\r\n");
        return ret;
    }

    /* Power up Camera peripheral */
    ret = CAMERAdrv->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        //printf("\r\n Error: CAMERA Power Up failed.\r\n");
        return ret;
    }

    /* Control configuration for camera controller */
    ret = CAMERAdrv->Control(CPI_CONFIGURE, 0);
    if (ret != ARM_DRIVER_OK) {
        //printf("\r\n Error: CPI Configuration failed.\r\n");
        return ret;
    }

    /* Control configuration for camera sensor */
    ret = CAMERAdrv->Control(CPI_CAMERA_SENSOR_CONFIGURE, 0);
    if (ret != ARM_DRIVER_OK) {
        //printf("\r\n Error: CAMERA SENSOR Configuration failed.\r\n");
        return ret;
    }

    /*Control configuration for camera events */
    ret = CAMERAdrv->Control(CPI_EVENTS_CONFIGURE,
                             ARM_CPI_EVENT_CAMERA_CAPTURE_STOPPED |
                             ARM_CPI_EVENT_ERR_CAMERA_INPUT_FIFO_OVERRUN |
                             ARM_CPI_EVENT_ERR_CAMERA_OUTPUT_FIFO_OVERRUN |
                             ARM_CPI_EVENT_ERR_HARDWARE);
    if (ret != ARM_DRIVER_OK) {
        //printf("\r\n Error: CAMERA SENSOR Event Configuration failed.\r\n");
        return ret;
    }

    camera_is_init = true;
    return ret;
}

/**
 * @brief 
 * 
 * @param buffer 
 * @return int 
 */
int camera_capture_frame(uint8_t* bufferm,uint16_t width, uint16_t height, bool swap_BGR)
{
    int ret;

    g_cb_events = CAM_CB_EVENT_NONE;

    ret = CAMERAdrv->CaptureFrame(camera_buffer);

    if (ret != ARM_DRIVER_OK) {
        return -1;
    }
    // Wait for capture
    while (!(g_cb_events & CAM_CB_EVENT_CAPTURE_STOPPED)) {
        __WFI();
    }
    // Invalidate cache before reading the camera_buffer
    SCB_CleanInvalidateDCache();

    // ARX3A0 camera uses bayer output
    // MT9M114 can use bayer or RGB565 depending on RTE config
#if !CAM_USE_RGB565
    if (dc1394_bayer_Simple(camera_buffer, image_buffer, CAM_FRAME_WIDTH, CAM_FRAME_HEIGHT, BAYER_FORMAT) != DC1394_SUCCESS) {
        return -2;
    }
#endif

// Do color correction for the ARX3A0 camera
// White balance function does also RGB --> BGR conversion so when skipping color correction
// the RGB --> BGR is done in the resize phase
#if CAM_COLOR_CORRECTION
    white_balance(CAM_FRAME_WIDTH, CAM_FRAME_HEIGHT, image_buffer, image_buffer);
#endif

    const int rescaleWidth = width;
    const int rescaleHeight = (int)(CAM_FRAME_HEIGHT * (float)rescaleWidth / CAM_FRAME_WIDTH);

    // Swap to BGR in resize phase when using MT9M114 - 
    if (resize_image(get_resize_source_buffer(),
                CAM_FRAME_WIDTH,
                CAM_FRAME_HEIGHT,
                (uint8_t*)bufferm,
                width,   // width
                height,  // height
                CAM_USE_RGB565 ? RGB565_BYTES : RGB_BYTES,
                (swap_BGR && (CAM_COLOR_CORRECTION == 0))) != 0) {
        return -3;
    }

    return (width * height);
}

/**
 * @brief 
 * 
 * @return int 
 */
int camera_start_stream(void)
{
    int ret;

    g_cb_events = CAM_CB_EVENT_NONE;

    ret = CAMERAdrv->CaptureVideo(camera_buffer);

    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    return ret;
}

/**
 * @brief 
 * 
 * @param width 
 * @param height 
 */
void camera_get_max_res(uint16_t* width, uint16_t* height)
{
    *width = CAM_FRAME_WIDTH;
    *height = CAM_FRAME_HEIGHT;
}
