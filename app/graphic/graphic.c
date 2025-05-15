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

#include "FreeRTOS.h"
#include "dave_driver.h"
#include "task.h"
#include "event_groups.h"

#include <stdio.h>
#include "graphic.h"
#include "Driver_CDC200.h" // Display driver

// DAVE
#include "aipl_dave2d.h"
#include "dave_cfg.h"
#include "dave_d0lib.h"

// Alif Image Processing Library
#include "aipl_color_correction.h"
#include "aipl_crop.h"
#include "aipl_image.h"
#include "aipl_lut_transform.h"
#include "aipl_resize.h"
#include "aipl_rotate.h"

#include "image/ei_alif_logo.h"
#include "image/image.h"
#include "text/text.h"
#include <math.h>

#pragma pack(1)
#if RTE_CDC200_PIXEL_FORMAT == 0    // ARGB8888
typedef uint32_t Pixel;
#elif RTE_CDC200_PIXEL_FORMAT == 1  // RGB888
typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} Pixel;
#elif RTE_CDC200_PIXEL_FORMAT == 2  // RGB565
typedef uint16_t Pixel;
#else
#error "CDC200 Unsupported color format"
#endif
#pragma pack()

enum {
    BUFFER_1 = 0,
    BUFFER_2 = 1,
    NUM_BUFFERS
};



#define ANTI_ALIASING_VAL       (1U)

#define RED_COLOR_VAL           (0xFF0000)
#define GREEN_COLOR_VAL         (0x00FF00)
#define BLUE_COLOR_VAL          (0x0000FF)
#define YELLOW_COLOR_VAL        (0xFFFF00)
#define WHITE_COLOR_VAL         (0xFFFFFF)
#define BLACK_COLOR_VAL         (0x0)

#define LOGO_OFFSET_Y           (72u)
#define CAM_OFFSET_Y            (480u)
#define TEXT_OFFSET_Y           (LOGO_OFFSET_Y + CAM_OFFSET_Y + 10)
#define TEXT_OFFSET_X           (2u)

// spacing between strings
#define STRING_SPACING          (20u)

#if (D1_MEM_ALLOC == D1_MALLOC_D0LIB)
// D/AVE D0 heap address and size
#define D1_HEAP_SIZE	0x80000
#endif

#if (D1_MEM_ALLOC == D1_MALLOC_D0LIB)
static uint8_t __attribute__((section(".bss.at_sram1"))) d0_heap[D1_HEAP_SIZE];
#endif

static Pixel lcd_buffer_1[MY_DISP_VER_RES][MY_DISP_HOR_RES]
            __attribute__((section(".bss.lcd_frame_buf"))) = {0};
static Pixel lcd_buffer_2[MY_DISP_VER_RES][MY_DISP_HOR_RES]
            __attribute__((section(".bss.lcd_frame_buf"))) = {0};

static Pixel* buffers[NUM_BUFFERS] = { (Pixel*)&lcd_buffer_1, (Pixel*)&lcd_buffer_2 };
static uint8_t  current_buffer = BUFFER_1;

extern ARM_DRIVER_CDC200 Driver_CDC200;
static ARM_DRIVER_CDC200 *CDCdrv = &Driver_CDC200;

static TaskHandle_t notify_vsync_handle = NULL;

static uint16_t string_printed = 0;
static uint16_t g_offset_y = 0;

EventGroupHandle_t dispEventGroupHandle = NULL;

static void disp_init(void);
static void disp_callback(uint32_t event);
static void* disp_active_buffer(void);
static void* disp_inactive_buffer(void);
static void graphic_prepare_draw_over_image(uint16_t x0, uint16_t y0, uint16_t width, uint16_t height, uint32_t color);
static void graphic_get_float_string(char* s, float f);

/**
 * @brief
 * 
 */
void graphic_init(void)
{
    dispEventGroupHandle = xEventGroupCreate();

    disp_init();
    
    notify_vsync_handle = xTaskGetCurrentTaskHandle();
#if (D1_MEM_ALLOC == D1_MALLOC_D0LIB)
    /*-------------------------
     * Initialize D/AVE D0 heap
     * -----------------------*/
    if (!d0_initheapmanager(d0_heap, sizeof(d0_heap), d0_mm_fixed_range, NULL, 0, 0, (d0_memmanager_type)0, d0_ma_unified)) {
        ////printf("\nError: Heap manager initialization failed\n");
    }
#endif
    // Initialize D/AVE2D
    if (aipl_dave2d_init() != D2_OK) {
        //printf("\nError: D/AVE2D initialization falied\n");
        __BKPT(0);
    }

    d2_device* d2_handle = aipl_dave2d_handle();

    /* Clear both buffers */
    d2_framebuffer(d2_handle, disp_active_buffer(),
    RTE_PANEL_HACTIVE_TIME,
    RTE_PANEL_HACTIVE_TIME,
    RTE_PANEL_VACTIVE_LINE * 2, d2_mode_rgb565);
    d2_clear(d2_handle, WHITE_COLOR_VAL);

    /* Process active displaylist to clear framebuffers */
    d2_startframe(d2_handle);
    
    d2_endframe(d2_handle);

    d2_setblendmode(d2_handle, d2_bm_alpha, d2_bm_one_minus_alpha);
    d2_setalphamode(d2_handle, d2_am_constant);
    d2_setalpha(d2_handle, 0xff);
    d2_setantialiasing(d2_handle, ANTI_ALIASING_VAL);
    d2_setlinecap(d2_handle, d2_lc_butt);
    d2_setlinejoin(d2_handle, d2_lj_none);
}


/*Initialize your display and the required peripherals.*/
static void disp_init(void)
{
    /* Initialize CDC driver */
    int ret = CDCdrv->Initialize(disp_callback);
    if(ret != ARM_DRIVER_OK){
        //printf("\n Error: CDC init failed\n");
        __BKPT(0);
        return;
    }

    /* Power control CDC */
    ret = CDCdrv->PowerControl(ARM_POWER_FULL);
    if(ret != ARM_DRIVER_OK){
        //printf("\n Error: CDC Power up failed\n");
        __BKPT(0);
        return;
    }

    /* configure CDC controller */
    ret = CDCdrv->Control(CDC200_CONFIGURE_DISPLAY, (uint32_t)lcd_buffer_1);
    if(ret != ARM_DRIVER_OK){
        //printf("\n Error: CDC controller configuration failed\n");
        __BKPT(0);
        return;
    }

    /* Enable CDC SCANLINE0 event */
    ret = CDCdrv->Control(CDC200_SCANLINE0_EVENT, ENABLE);
    if(ret != ARM_DRIVER_OK){
        //printf("\n Error: CDC200_SCANLINE0_EVENT enable failed\n");
        __BKPT(0);
        return;
    }

    /* Start CDC */
    ret = CDCdrv->Start();
    if(ret != ARM_DRIVER_OK){
        //printf("\n Error: CDC Start failed\n");
        __BKPT(0);
        return;
    }
}

/**
 * @brief Draw Edge Impulse Alif logo
 */
void draw_logo(void)
{
    aipl_image_draw(0, 0, get_ei_alif_logo());
}

/**
 * @brief
 * 
 */
void graphic_set_centroid(const char* label, uint16_t x0, uint16_t y0, uint16_t width, uint16_t height, float value, float ratio)
{
    char centroid[256] = {0};
    char float_num[32] = {0};

    d2_device* d2_handle = aipl_dave2d_handle();
    uint16_t new_x0 = (x0 * ratio);
    uint16_t new_y0 = (y0 * ratio) + (LOGO_OFFSET_Y);
    uint16_t new_width = (width * ratio);
    uint16_t new_height = (height * ratio);
    const d2_width circle_radius = 10;
    const d2_width circle_width = 4;

    d2_point x_center = (((new_x0 << 1) + new_width) >> 1);
    d2_point y_center = (((new_y0 << 1) + new_height) >> 1);
    
    graphic_prepare_draw_over_image(new_x0, new_y0, new_width  + new_x0, new_height + new_y0, YELLOW_COLOR_VAL);    
    d2_rendercircle(d2_handle, (x_center << 4), (y_center << 4), (d2_width)(circle_radius << 4), (d2_width)(circle_width << 4));
}

/**
 * 
 */
void graphic_set_detection_text(const char* label, uint16_t x0, uint16_t y0, uint16_t width, uint16_t height, float value)
{
    d2_device* d2_handle = aipl_dave2d_handle();
    char detection_text[256] = {0};
    char float_num[32] = {0};

    // text    
    graphic_get_float_string(float_num, value);
    snprintf(detection_text, 256, "  %s (%s) [x: %u, y: %u, w: %u, h: %u]", label, float_num, x0, y0, width, height);

    text_print(d2_handle, detection_text, (uint8_t)strlen(detection_text), (TEXT_OFFSET_X), g_offset_y + (STRING_SPACING * string_printed), BLACK_COLOR_VAL);
    string_printed++;
}

/**
 * @brief
 * 
 */
void graphic_set_box(const char* label, uint16_t x0, uint16_t y0, uint16_t width, uint16_t height, float value, float ratio)
{
    char centroid[256] = {0};
    char float_num[32] = {0};

    d2_device* d2_handle = aipl_dave2d_handle();
    uint16_t new_x0 = (x0 * ratio);
    uint16_t new_y0 = (y0 * ratio) + LOGO_OFFSET_Y;
    uint16_t new_width = (width * ratio);
    uint16_t new_height = (height * ratio);

    //d2_setcolor(d2_handle, 0, GREEN_COLOR_VAL);

    graphic_prepare_draw_over_image(new_x0, new_y0, new_width  + new_x0, new_height + new_y0, YELLOW_COLOR_VAL);
    d2_renderline(d2_handle, (d2_point) ((new_x0) << 4), (d2_point) (new_y0 << 4), (d2_point) ((new_x0 + new_width) << 4), (d2_point) ((new_y0 ) << 4), (d2_point) (2 << 4), 0);
    d2_renderline(d2_handle, (d2_point) ((new_x0 + new_width) << 4), (d2_point) (new_y0 << 4), (d2_point) ((new_x0 + new_width) << 4), (d2_point) ((new_y0 + new_height) << 4), (d2_point) (2 << 4), 0);
    d2_renderline(d2_handle, (d2_point) ((new_x0 + new_width) << 4), (d2_point) ((new_y0 + new_height) << 4), (d2_point) (new_x0 << 4), (d2_point) ((new_y0 + new_height) << 4), (d2_point) (2 << 4), 0);
    d2_renderline(d2_handle, (d2_point) ((new_x0) << 4), (d2_point) ((new_y0 + new_height) << 4), (d2_point) (new_x0 << 4), (d2_point) (new_y0 << 4), (d2_point) (2 << 4), 0);
}

/**
 * @brief
 * 
 */
void graphic_start_label(const char* initial_string, bool is_live)
{
    d2_device* d2_handle = aipl_dave2d_handle();

    if (is_live) {
        g_offset_y = TEXT_OFFSET_Y;
    }
    else {
        g_offset_y = LOGO_OFFSET_Y + STRING_SPACING;
    }
    
    text_print(d2_handle, initial_string, (uint8_t)strlen(initial_string), TEXT_OFFSET_X,  g_offset_y + (STRING_SPACING * string_printed), BLACK_COLOR_VAL);
    string_printed++;
}

/**
 * @brief
 * 
 */
void graphic_no_detection(void)
{
    d2_device* d2_handle = aipl_dave2d_handle();
    const char temp_str[] = "    No objects found";
    
    text_print(d2_handle, temp_str, (uint8_t)strlen(temp_str), TEXT_OFFSET_X, g_offset_y + (STRING_SPACING * string_printed), BLACK_COLOR_VAL);
    string_printed++;
}

/**
 * @brief
 * 
 */
void graphic_update_classification(const char* label, float value)
{
    d2_device* d2_handle = aipl_dave2d_handle();
    char temp_str[256] = {0};
    char float_dsp[32] = {0};

    graphic_get_float_string(float_dsp, value);

    snprintf(temp_str, 256, "    %s: %s", label, float_dsp);
    text_print(d2_handle, temp_str, (uint8_t)strlen(temp_str), TEXT_OFFSET_X, g_offset_y + (STRING_SPACING * string_printed), BLACK_COLOR_VAL);
    string_printed++;
}

/**
 * @brief
 * 
 */
void graphic_set_timing(int16_t fps, uint32_t dsp_us, uint32_t classification_us)
{
    d2_device* d2_handle = aipl_dave2d_handle();
    char timing_str[256] = {0};
    char float_dsp[32] = {0};
    char float_classification[32] = {0};

    string_printed = 0;
    
    if (fps < 0) {
        snprintf(timing_str, 256, "Edge Impulse live inference");
    }
    else {
        snprintf(timing_str, 256, "Edge Impulse live inference - FPS: %d", fps);
    }
    
    text_print(d2_handle, timing_str, (uint8_t)strlen(timing_str), TEXT_OFFSET_X, g_offset_y + (STRING_SPACING * string_printed), BLACK_COLOR_VAL);
    string_printed++;

    graphic_get_float_string(float_dsp, (float)dsp_us / 1000.0f);
    graphic_get_float_string(float_classification, (float)classification_us / 1000.0f);

    snprintf(timing_str, 256, "Predictions (DSP: %s ms., Class: %s ms.)", float_dsp, float_classification);
    text_print(d2_handle, timing_str, (uint8_t)strlen(timing_str), TEXT_OFFSET_X, g_offset_y + (STRING_SPACING * string_printed), BLACK_COLOR_VAL);
    string_printed++;
}

/**
 * @brief
 * 
 */
void graphic_start_buffer(void)
{
    static d2_color color = 0xFFFFFFFF;

    d2_device* handle = aipl_dave2d_handle();
    
    /* Start a new display list */

    /* Prepare frame buffer */
    d2_framebuffer(handle, disp_inactive_buffer(),
                    RTE_PANEL_HACTIVE_TIME,
                    RTE_PANEL_HACTIVE_TIME,
                    RTE_PANEL_VACTIVE_LINE, d2_mode_rgb565);

    /* Clear the frame buffer */
    d2_clear(handle, WHITE_COLOR_VAL);

    d2_startframe(handle);
}

/**
 * @brief
 * 
 */
void graphic_end_frame(void)
{
    d2_device* handle = aipl_dave2d_handle();

    /* Close render buffer which has just captured all render commands */
    d2_endframe(handle);
    /* Start HW rendering of the closed frame */
    d2_startframe(handle);
    
    /* Wait until the render finishes */
    d2_endframe(handle);
}

/**
 * 
 */
void display_camera_screen(uint8_t* cam_buff, uint16_t cam_witdh, uint16_t cam_heigth)
{
    d2_device* handle = aipl_dave2d_handle();
    aipl_image_t cam_image = {
        .data = cam_buff,
        .pitch = cam_witdh,
        .width = cam_witdh,
        .height = cam_heigth,
        .format = AIPL_COLOR_RGB565
    };
    
    aipl_image_draw(0, LOGO_OFFSET_Y, &cam_image);
}

/**
 * 
 */
void disp_next_frame(void)
{
    current_buffer = (current_buffer + 1) % NUM_BUFFERS;

    xEventGroupClearBits(dispEventGroupHandle, EVENT_DISP_BUFFER_READY);
    // update at vsync
    CDCdrv->Control(CDC200_FRAMEBUF_UPDATE_VSYNC, (uint32_t)buffers[current_buffer]);
    xEventGroupSetBits(dispEventGroupHandle, EVENT_DISP_BUFFER_CHANGED);

    xEventGroupWaitBits(dispEventGroupHandle, 
        EVENT_DISP_BUFFER_READY,    //  uxBitsToWaitFor 
        pdFALSE,                 //  xClearOnExit
        pdFALSE,                //  xWaitForAllBits
        portMAX_DELAY);
}

/**
 * 
 */
static void* disp_active_buffer(void)
{
    return buffers[current_buffer];
}

/**
 * 
 */
static void* disp_inactive_buffer(void)
{
    return buffers[(current_buffer + 1) % NUM_BUFFERS];
}

/**
 * 
 */
static void disp_callback(uint32_t event)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
#if 0
    if (event & ARM_CDC_SCANLINE0_EVENT) {
        // Scanline 0 event: Vsync
        if(notify_vsync_handle != NULL) {
            vTaskNotifyGiveFromISR(notify_vsync_handle, &xHigherPriorityTaskWoken);

        }
    }
#else
    if (event & ARM_CDC_SCANLINE0_EVENT) {
        if (xEventGroupGetBitsFromISR(dispEventGroupHandle) & EVENT_DISP_BUFFER_CHANGED) {
            xEventGroupClearBitsFromISR(
                dispEventGroupHandle,
                EVENT_DISP_BUFFER_CHANGED);
            
            xEventGroupSetBitsFromISR(
                dispEventGroupHandle,
                EVENT_DISP_BUFFER_READY,
                &xHigherPriorityTaskWoken);
        }
    }   
#endif
    if(event & ARM_CDC_DSI_ERROR_EVENT)
    {
        // Transfer Error: Received Hardware error.
        __BKPT(0);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief
 */
static void graphic_prepare_draw_over_image(uint16_t x0, uint16_t y0, uint16_t width, uint16_t height, uint32_t color)
{
    d2_device* d2_handle = aipl_dave2d_handle();

    d2_settextureoperation(d2_handle, d2_to_copy, d2_to_replace, d2_to_replace, d2_to_replace);
    d2_settexopparam(d2_handle, d2_cc_red, color>>16, 0);
    d2_settexopparam(d2_handle, d2_cc_green, (color&0x0000FF00)>>8, 0);
    d2_settexopparam(d2_handle, d2_cc_blue, color&0x000000FF, 0);
    d2_setcolorkey(d2_handle, 1, 0x000000);

    d2_setfillmode(d2_handle, d2_fm_color);

    d2_setalpha(d2_handle, (d2_alpha)255);
    d2_setcolor(d2_handle, 0, (d2_color)YELLOW_COLOR_VAL);
    d2_cliprect(d2_handle, x0, y0, x0  + width, y0 + height);
}

/**
 * @brief get float string
 * 
 */
static void graphic_get_float_string(char* s, float f)
{
    float n = f;

    static double PRECISION = 0.001;

    if (n == 0.0) {
        strcpy(s, "0");
    }
    else {
        int digit, m;
        char *c = s;
        int neg = (n < 0);
        if (neg) {
            n = -n;
        }
        // calculate magnitude
        m = log10(n);
        if (neg) {
            *(c++) = '-';
        }
        if (m < 1.0) {
            m = 0;
        }
        // convert the number
        while (n > PRECISION || m >= 0) {
            double weight = pow(10.0, m);
            if (weight > 0 && !isinf(weight)) {
                digit = floor(n / weight);
                n -= (digit * weight);
                *(c++) = '0' + digit;
            }
            if (m == 0 && n > 0) {
                *(c++) = '.';
            }
            m--;
        }
        *(c) = '\0';
    }

}
