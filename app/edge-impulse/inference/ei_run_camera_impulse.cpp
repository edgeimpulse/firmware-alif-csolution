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

#include "model-parameters/model_metadata.h"
#if defined(EI_CLASSIFIER_SENSOR) && (EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_CAMERA)
#include "board.h"
#if defined (BOARD_IS_ALIF_DEVKIT_E1C_VARIANT)
#error "Camera not supported on Alif Devkit E1C devkit"
#else
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "firmware-sdk/ei_camera_interface.h"
#include "ingestion-sdk-platform/sensor/ei_camera.h"
#include "firmware-sdk/at_base64_lib.h"
#include "firmware-sdk/jpeg/encode_as_jpg.h"
#include "inference_task.h"

#include "FreeRTOS.h"
#include "task.h"


#if (defined(LCD_SUPPORTED) && (LCD_SUPPORTED == 1))

#include "lcd_task.h"

#if (defined(EI_CLASSIFIER_SENSOR) && (EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_CAMERA))

#include "peripheral/camera/image_processing.h"

#include "aipl_image.h"
#include "aipl_crop.h"
#include "aipl_resize.h"
#include "aipl_color_conversion.h"

#endif
#endif

typedef enum {
    INFERENCE_STOPPED,
    INFERENCE_WAITING,
    INFERENCE_SAMPLING,
    INFERENCE_DATA_READY
} inference_state_t;

static inference_state_t state = INFERENCE_STOPPED;
static uint64_t last_inference_ts = 0;

static bool debug_mode = false;
static bool continuous_mode = false;
static bool use_max_uart = false;

//static uint8_t *snapshot_buf = nullptr;
static uint32_t snapshot_buf_size;
#if defined (CORE_M55_HE)
static uint8_t snapshot_buf[EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT * 3] __attribute__((aligned(32), section(".bss.local_heap")));
#else
static uint8_t snapshot_buf[EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT * 3];
#endif
static ei_device_snapshot_resolutions_t snapshot_resolution;
static ei_device_snapshot_resolutions_t fb_resolution;

static bool resize_required = false;
static bool crop_required = false;

static uint32_t inference_delay;
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);

/**
 * @brief 
 * 
 * @param continuous 
 * @param debug 
 * @param use_max_uart_speed 
 */
void ei_start_impulse(bool continuous, bool debug, bool use_max_uart_speed)
{
    auto* dev = EiDeviceInfo::get_device();
    EiAlifCamera *cam = static_cast<EiAlifCamera*>(EiCamera::get_camera());

    debug_mode = debug;
    continuous_mode = debug? true : continuous;
    use_max_uart = use_max_uart_speed;

    if (cam->is_camera_present() == false) {
        ei_printf("ERR: Failed to start inference, camera is missing!\n");
        return;
    }

    snapshot_resolution = cam->search_resolution(EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT);
    if (cam->set_resolution(snapshot_resolution) == false) {
        ei_printf("ERR: Failed to set snapshot resolution (%ux%u)!\n", EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT);
        return;
    }

    if (snapshot_resolution.width > EI_CLASSIFIER_INPUT_WIDTH || snapshot_resolution.height > EI_CLASSIFIER_INPUT_HEIGHT) {
        crop_required = true;
        resize_required = false;
    }
    else if (snapshot_resolution.width < EI_CLASSIFIER_INPUT_WIDTH || snapshot_resolution.height < EI_CLASSIFIER_INPUT_HEIGHT) {
        crop_required = false;
        resize_required = true;
    }
    else {
        crop_required = false;
        resize_required = false;
    }

    snapshot_buf_size = snapshot_resolution.width * snapshot_resolution.height * 3;

    // summary of inferencing settings (from model_metadata.h)
    ei_printf("Inferencing settings:\n");
    ei_printf("\tImage resolution: %dx%d\n", EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT);
    ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) / sizeof(ei_classifier_inferencing_categories[0]));

    if (continuous_mode == true) {
        inference_delay = 0;
        state = INFERENCE_DATA_READY;
    }
    else {
        inference_delay = 2000;
        last_inference_ts = ei_read_timer_ms();
        state = INFERENCE_WAITING;
        ei_printf("Starting inferencing in %d seconds...\n", inference_delay / 1000);
    }

    if ((use_max_uart) || (debug_mode)) {
        ei_printf("OK\r\n");
        ei_sleep(100);
        dev->set_max_data_output_baudrate();
        ei_sleep(100);
    }

    inference_task_start();
}

/**
 * @brief 
 * 
 */
void ei_stop_impulse(void)
{
    auto* dev = EiDeviceInfo::get_device();

    if ((use_max_uart) || (debug_mode)) {
        ei_printf("\r\nOK\r\n");
        ei_sleep(100);
        dev->set_default_data_output_baudrate();
        ei_sleep(100);
    }

    state = INFERENCE_STOPPED;
}

/**
 * 
 */
void ei_run_impulse(void)
{
    switch(state) {
        case INFERENCE_STOPPED:
            // nothing to do
            return;
        case INFERENCE_WAITING:
            if (ei_read_timer_ms() < (last_inference_ts + inference_delay)) {
                return;
            }
            state = INFERENCE_DATA_READY;
            break;
        case INFERENCE_SAMPLING:
        case INFERENCE_DATA_READY:
            if (continuous_mode == true) {
                state = INFERENCE_WAITING;
            }
            break;
        default:
            break;
    }

    EiAlifCamera *camera = static_cast<EiAlifCamera*>(EiAlifCamera::get_camera());

    ei_printf("Taking photo...\n");

    bool isOK = camera->ei_camera_capture_rgb888_packed_big_endian(snapshot_buf, snapshot_buf_size);
    if (!isOK) {
        return;
    }

    if (resize_required) {
        ei::image::processing::crop_and_interpolate_rgb888(
            snapshot_buf,
            fb_resolution.width,
            fb_resolution.height,
            snapshot_buf,
            snapshot_resolution.width,
            snapshot_resolution.height);
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    // Print framebuffer as JPG during debugging
    if (debug_mode) {
        ei_printf("Begin output\n");

        size_t jpeg_buffer_size = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT >= 128 * 128 ?
            8192 * 3 :
            4096 * 4;
        uint8_t *jpeg_buffer = NULL;
        jpeg_buffer = (uint8_t*)ei_malloc(jpeg_buffer_size);
        if (!jpeg_buffer) {
            ei_printf("ERR: Failed to allocate JPG buffer\r\n");
            return;
        }

        size_t out_size;
        int x = encode_rgb888_signal_as_jpg(&signal, EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, jpeg_buffer, jpeg_buffer_size, &out_size);
        if (x != 0) {
            ei_printf("Failed to encode frame as JPEG (%d)\n", x);
            return;
        }

        ei_printf("Framebuffer: ");
        base64_encode((char*)jpeg_buffer, out_size, ei_putchar);
        ei_printf("\r\n");

        if (jpeg_buffer) {
            ei_free(jpeg_buffer);
        }
    }

    // run the impulse: DSP, neural network and the Anomaly algorithm
    ei_impulse_result_t result = { 0 };

    vTaskSuspendAll();
    EI_IMPULSE_ERROR ei_error = run_classifier(&signal, &result, false);
    xTaskResumeAll();
    if (ei_error != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run impulse (%d)\n", ei_error);
        //ei_free(snapshot_buf);
        return;
    }
    //ei_free(snapshot_buf);

    display_results(&ei_default_impulse, &result);
#if (defined(LCD_SUPPORTED) && (LCD_SUPPORTED == 1))
    lcd_set_result(&result);
#endif

    if (debug_mode) {
        ei_printf("\r\n----------------------------------\r\n");
        ei_printf("End output\r\n");
    }

    if (continuous_mode == false) {
        ei_printf("Starting inferencing in %d seconds...\n", inference_delay / 1000);
        last_inference_ts = ei_read_timer_ms();
        state = INFERENCE_WAITING;
    }
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool is_inference_running(void)
{
    return (state != INFERENCE_STOPPED);
}

/**
 *
 * @param offset
 * @param length
 * @param out_ptr
 * @return
 */
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
    // we already have a RGB888 buffer, so recalculate offset into pixel index
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix + 2];

        // go to the next pixel
        out_ptr_ix++;
        pixel_ix+=3;
        pixels_left--;
    }

    // and done!
    return 0;
}

#if (defined(LCD_SUPPORTED) && (LCD_SUPPORTED == 1)) && (defined(EI_CLASSIFIER_SENSOR) && (EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_CAMERA))
/**
 *
 * @param pbuffer
 * @param width
 * @param height
 * @param presult
 */
void ei_run_stream_impulse(uint8_t* pbuffer, uint16_t width, uint16_t height, ei_impulse_result_t* presult)
{
#if 0
    aipl_error_t aipl_ret = AIPL_ERR_OK;

    aipl_image_t cam_image = {
        .data = pbuffer,
        .pitch = width,
        .width = width,
        .height = height,
        .format = AIPL_COLOR_RGB565
    };

    aipl_image_t res_image = {
        .data = snapshot_buf,
        .pitch = EI_CLASSIFIER_INPUT_WIDTH,
        .width = EI_CLASSIFIER_INPUT_WIDTH,
        .height = EI_CLASSIFIER_INPUT_HEIGHT,
        .format = AIPL_COLOR_RGB565
    };

    if (pbuffer == NULL) {
        ei_printf("ERR: Invalid input framebuffer\n");
        return;
    }

    aipl_ret = aipl_resize_img(&cam_image, &res_image, true);  // interpolate

    if (aipl_ret != AIPL_ERR_OK) {
        ei_printf("Error: resize aipl_ret = %s\r\n", aipl_error_str(aipl_ret));
    }

    aipl_ret = aipl_color_convert(snapshot_buf, snapshot_buf, EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, AIPL_COLOR_RGB565, AIPL_COLOR_RGB888);
    if (aipl_ret != AIPL_ERR_OK) {
        ei_printf("Error: color conversion aipl_ret = %s\r\n", aipl_error_str(aipl_ret));
    }
#endif
        // Swap to BGR in resize phase when using MT9M114 - 
    if (resize_image(pbuffer,
        width,
        height,
        (uint8_t*)snapshot_buf,
        EI_CLASSIFIER_INPUT_WIDTH,   // width
        EI_CLASSIFIER_INPUT_HEIGHT,  // height
        RGB565_BYTES,
        (false && (0 == 0))) != 0) {
        ei_printf("Error: resize_image failed\r\n");
        return ;
    }

#if 0
    ei::image::processing::crop_and_interpolate_rgb888(
        snapshot_buf,
        EI_CLASSIFIER_INPUT_WIDTH,
        EI_CLASSIFIER_INPUT_HEIGHT,
        snapshot_buf,
        EI_CLASSIFIER_INPUT_WIDTH,
        EI_CLASSIFIER_INPUT_HEIGHT); // bytes per pixel
#endif

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    memset(presult, 0, sizeof(ei_impulse_result_t));

    //vTaskSuspendAll();
    EI_IMPULSE_ERROR ei_error = run_classifier(&signal, presult, false);
    //xTaskResumeAll();

    if (ei_error != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run impulse (%d)\n", ei_error);
    }
}
#endif  /* #if (defined(LCD_SUPPORTED) && (LCD_SUPPORTED == 1)) && (defined(EI_CLASSIFIER_SENSOR) && (EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_CAMERA)) */
#endif  /* defined (BOARD_IS_ALIF_DEVKIT_E1C_VARIANT) */
#endif /* defined(EI_CLASSIFIER_SENSOR) && EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_CAMERA */
