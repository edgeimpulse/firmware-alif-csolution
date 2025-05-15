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

 #include "camera.h"
 #include CMSIS_device_header
 #include "Driver_CPI.h"    // Camera
 #include "bayer.h"
 #include "image_processing.h"

#include "FreeRTOS.h"
#include "task.h"

#if (defined(LCD_SUPPORTED) && (LCD_SUPPORTED == 1)) && (defined(EI_CLASSIFIER_SENSOR) && (EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_CAMERA))
#include "aipl_image.h"
#include "aipl_crop.h"
#include "aipl_resize.h"
#include "aipl_color_conversion.h"
#endif

#define BAYER_FORMAT DC1394_COLOR_FILTER_GRBG

/* Camera Controller Resolution. */
#if defined(RTE_Drivers_CAMERA_SENSOR_MT9M114) && (RTE_Drivers_CAMERA_SENSOR_MT9M114 == 1)

#define CAM_COLOR_CORRECTION   (0)
#define CAM_USE_RGB565         (1)
#define RGB_BUFFER_SECTION     ".bss.camera_frame_bayer_to_rgb_buf_at_sram0"

#elif defined (RTE_Drivers_CAMERA_SENSOR_ARX3A0) && (RTE_Drivers_CAMERA_SENSOR_ARX3A0 == 1)

#define CAM_COLOR_CORRECTION   (1)
#define CAM_USE_RGB565         (0)
#define RGB_BUFFER_SECTION     ".bss.camera_frame_bayer_to_rgb_buf"
#else
#error "Unsupported camera sensor"
#endif

static uint8_t camera_buffer[CAM_FRAME_SIZE_BYTES] __attribute__((aligned(32), section(".bss.camera_frame_buf")));

// Buffer for bayer conversion to RGB is not needed when camera outputs RGB565
#if defined(CAM_USE_RGB565) && (CAM_USE_RGB565 == 1)
static const uint8_t *get_resize_source_buffer() { return camera_buffer; }
#else
static uint8_t image_buffer[CAM_FRAME_SIZE * RGB_BYTES] __attribute__((aligned(32), section(RGB_BUFFER_SECTION)));
static const uint8_t *get_resize_source_buffer() { return image_buffer; }
#endif

/* Camera  Driver instance 0 */
extern ARM_DRIVER_CPI Driver_CPI;
static ARM_DRIVER_CPI *CAMERAdrv = &Driver_CPI;

typedef enum {
    CAM_CB_EVENT_NONE               = 0,
    CAM_CB_EVENT_ERROR              = (1 << 0),
    CAM_CB_EVENT_CAPTURE_STOPPED    = (1 << 1),
} CB_EVENTS;

static volatile CB_EVENTS g_cb_events = CAM_CB_EVENT_NONE;
static bool camera_is_init = false;

static TaskHandle_t xTaskToNotify = NULL;

/**
 * 
 */
static void camera_callback(uint32_t event)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    switch (event)
    {
    case ARM_CPI_EVENT_CAMERA_CAPTURE_STOPPED:
        g_cb_events |= CAM_CB_EVENT_CAPTURE_STOPPED;
        if (xTaskToNotify != NULL){
            vTaskNotifyGiveFromISR( xTaskToNotify, &xHigherPriorityTaskWoken );
        }
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
        g_cb_events |= CAM_CB_EVENT_ERROR | CAM_CB_EVENT_CAPTURE_STOPPED;
        if (xTaskToNotify != NULL){
            vTaskNotifyGiveFromISR( xTaskToNotify, &xHigherPriorityTaskWoken );
        }
        break;
    }
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}


#define DEBUG_CAMERA_INIT 0

#if defined (DEBUG_CAMERA_INIT) && (DEBUG_CAMERA_INIT == 1)
volatile bool _aa_camera_init = true;
#endif
/**
 * @brief Intialise camera
 * 
 * @return int 
 */
int camera_init(void)
{

#if defined (DEBUG_CAMERA_INIT) && DEBUG_CAMERA_INIT == 1
    while (_aa_camera_init == true) {
        _aa_camera_init = _aa_camera_init;
    }
#endif

    if (camera_is_init == true) {
        return 0;
    }

    int ret = CAMERAdrv->Uninitialize();
    if (ret != ARM_DRIVER_OK) {
        return 1;
    }

    ret = CAMERAdrv->Initialize(camera_callback);
    if (ret != ARM_DRIVER_OK) {
        //printf("\r\n Error: CAMERA Initialize failed.\r\n");
        return 2;
    }

    /* Power up Camera peripheral */
    ret = CAMERAdrv->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        //printf("\r\n Error: CAMERA Power Up failed.\r\n");
        return 3;
    }

    /* Control configuration for camera controller */
    ret = CAMERAdrv->Control(CPI_CONFIGURE, 0);
    if (ret != ARM_DRIVER_OK) {
        //printf("\r\n Error: CPI Configuration failed.\r\n");
        return 4;
    }

    /* Control configuration for camera sensor */
    ret = CAMERAdrv->Control(CPI_CAMERA_SENSOR_CONFIGURE, 0);
    if (ret != ARM_DRIVER_OK) {
        //printf("\r\n Error: CAMERA SENSOR Configuration failed.\r\n");
        return 5;
    }

    /*Control configuration for camera events */
    ret = CAMERAdrv->Control(CPI_EVENTS_CONFIGURE,
                             ARM_CPI_EVENT_CAMERA_CAPTURE_STOPPED |
                             ARM_CPI_EVENT_ERR_CAMERA_INPUT_FIFO_OVERRUN |
                             ARM_CPI_EVENT_ERR_CAMERA_OUTPUT_FIFO_OVERRUN |
                             ARM_CPI_EVENT_ERR_HARDWARE);
    if (ret != ARM_DRIVER_OK) {
        //printf("\r\n Error: CAMERA SENSOR Event Configuration failed.\r\n");
        return 6;
    }

#if RTE_Drivers_CAMERA_SENSOR_ARX3A0
    CAMERAdrv->Control(CPI_CAMERA_SENSOR_GAIN, 0x10000 * 5.0f);
#endif

    camera_is_init = true;
    return ret;
}

/**
 * @brief 
 * 
 * @param buffer 
 * @return int 
 */
int camera_capture_frame(uint8_t* bufferm, uint16_t width, uint16_t height, bool to_rgb888)
{
    int ret;

    camera_start_stream();

    ret = CAMERAdrv->CaptureFrame(camera_buffer);

    if (ret != ARM_DRIVER_OK) {
        return -1;
    }
    // Wait for capture
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

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

    uint32_t cropWidth = 0;
    uint32_t cropHeight = 0;

    calculate_crop_dims(CAM_FRAME_WIDTH, CAM_FRAME_HEIGHT, 
                        width, height, &cropWidth, &cropHeight);

    int res = frame_crop(
        get_resize_source_buffer(),
        CAM_FRAME_WIDTH,
        CAM_FRAME_HEIGHT,
        (CAM_FRAME_WIDTH - cropWidth) / 2,
        (CAM_FRAME_HEIGHT - cropHeight) / 2,
        get_resize_source_buffer(),
        cropWidth,
        cropHeight,
		CAM_USE_RGB565 ? (RGB565_BYTES * 8) : (RGB_BYTES * 8));
    
    if (res < 0) { 
        return res;
    }

    // Swap to BGR in resize phase when using MT9M114 - 
    if (resize_image(get_resize_source_buffer(),
                cropWidth,
                cropHeight,
                (uint8_t*)bufferm,
                width,   // width
                height,  // height
                CAM_USE_RGB565 ? RGB565_BYTES : RGB_BYTES,
                (to_rgb888 && (CAM_COLOR_CORRECTION == 0))) != 0) {
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
    xTaskToNotify = xTaskGetCurrentTaskHandle();

    //ret = CAMERAdrv->CaptureVideo(camera_buffer);
    
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    return ret;
}


#if (defined(LCD_SUPPORTED) && (LCD_SUPPORTED == 1)) && (defined(EI_CLASSIFIER_SENSOR) && (EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_CAMERA))
/**
 * 
 */
bool camera_capture_stream_frame(uint8_t* bufferm, uint16_t width, uint16_t height)
{
    int ret;

    g_cb_events = CAM_CB_EVENT_NONE;

    ret = CAMERAdrv->CaptureFrame(camera_buffer);

    if (ret != ARM_DRIVER_OK) {
        return -1;
    }
    // Wait for capture
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
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

    aipl_image_t cam_image = {
        .data = camera_buffer,
        .pitch = CAM_FRAME_WIDTH,
        .width = CAM_FRAME_WIDTH,
        .height = CAM_FRAME_HEIGHT,
        .format = AIPL_COLOR_RGB565
    };

    aipl_image_t crop_image = {
        .data = bufferm,
        .pitch = width,
        .width = width,
        .height = height,
        .format = AIPL_COLOR_RGB565
    };

    // Crop image to a square using the smaller of the camera dimensions
    const uint32_t crop_dim = cam_image.width > cam_image.height ? cam_image.height : cam_image.width;
    const uint32_t crop_border = (cam_image.width - crop_dim) / 2;
    const uint32_t crop_header = (cam_image.height - crop_dim) / 2;

    aipl_error_t aipl_ret = AIPL_ERR_OK;

    aipl_ret = aipl_crop_img(&cam_image, &crop_image, crop_border, crop_header, cam_image.width - crop_border,
        cam_image.height - crop_header);
        if (aipl_ret != AIPL_ERR_OK) {
        //printf("Error: crop aipl_ret = %s\r\n", aipl_error_str(aipl_ret));
        return false;
    }

    return true;
}

/**
 * 
 */
void camera_stop_stream(void)
{
    g_cb_events = CAM_CB_EVENT_NONE;

    // ok ?
    CAMERAdrv->Stop();
    g_cb_events = CAM_CB_EVENT_NONE;
    xTaskToNotify = NULL;
}
#endif

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
