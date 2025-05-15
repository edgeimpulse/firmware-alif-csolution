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

#include "FreeRTOS.h"
#include "portmacrocommon.h"
#include "task.h"
#include "event_groups.h"
#include "semphr.h"

#include "lcd_task.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "graphic/graphic.h"
#include "peripheral/camera/camera.h"

#include "edge-impulse/inference/ei_run_impulse.h"
#include "common_events.h"

// LCD task parameters
#define LCD_TASK_STACK_SIZE_BYTE        (8192u)
#define LCD_TASK_PRIORITY               (configMAX_PRIORITIES - 2)
static TaskHandle_t lcd_task_handle = NULL;
static void lcd_task(void *pvParameters);

static EventGroupHandle_t lcd_state_group;
static t_snapshot_state snapshot_state = e_snapshot_idle;

#if (defined(EI_CLASSIFIER_SENSOR) && (EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_CAMERA))

#define CAMERA_TASK_STACK_SIZE_BYTE        (2048u)
#define CAMERA_TASK_PRIORITY               (configMAX_PRIORITIES - 4)
static TaskHandle_t camera_task_handle = NULL;
static void camera_task(void *pvParameters);

static SemaphoreHandle_t xCamera_count_sem = NULL;

#define CAMERA_CANVAS_WIDTH     (480u)
#define CAMERA_CANVAS_HEIGHT    (480u)

/*Create a buffer for the canvas*/
static uint8_t lcd_camera_buffer[2][CAMERA_CANVAS_WIDTH * CAMERA_CANVAS_HEIGHT * 2] __attribute__((aligned(32), section(".bss.lcd_frame_buf")));
static uint8_t camera_buffer_index = 0;
static uint8_t *camera_buffer_ptr;

static bool camera_is_running = false;

static void set_lcd_detection(ei_impulse_result_bounding_box_t* pbox);
#endif

static void lcd_inference(void);

static void lcd_print_result(bool is_live = false);
static ei_impulse_result_t lcd_result = {0};
static int16_t gFps;

/**
 * @brief 
 * 
 */
void lcd_task_start(void)
{
    //graphic_init();
    
    /* Only init once */
    if (lcd_task_handle != NULL) {
        return;
    }

    snapshot_state = e_snapshot_idle;

#if (defined(EI_CLASSIFIER_SENSOR) && (EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_CAMERA))
    camera_buffer_ptr = lcd_camera_buffer[0];
#endif

    if (xTaskCreate(lcd_task,
        (const char*) "LCD task",
        LCD_TASK_STACK_SIZE_BYTE / 4, // in words
        NULL, //pvParameters
        LCD_TASK_PRIORITY, //uxPriority
        &lcd_task_handle) != pdPASS) {
        ei_printf("Failed to create LCD task\r\n");
        
        return;
    }
}

t_snapshot_state lcd_get_state(void)
{
    return snapshot_state;
}

/**
 * @brief Set LCD state
 * 
 * @param state 
 */
void lcd_set_state(t_snapshot_state state)
{
    snapshot_state = state;
    xEventGroupWaitBits(lcd_state_group, 
        1,    //  uxBitsToWaitFor 
        pdTRUE,                 //  xClearOnExit
        pdFALSE,                //  xWaitForAllBits
        portMAX_DELAY);
}

// enable debug
#define DEBUG_LCD 0

#if defined (DEBUG_LCD) && (DEBUG_LCD == 1)
volatile bool _aa_snapshot_ready = true;
#endif

/**
 * @brief 
 * 
 * @param pvParameters 
 */
static void lcd_task(void *pvParameters)
{
    (void)pvParameters;
#if defined (DEBUG_LCD) && DEBUG_LCD == 1
    while (_aa_snapshot_ready == true) {
        _aa_snapshot_ready = _aa_snapshot_ready;
    }
#endif    
    lcd_state_group = xEventGroupCreate();

    graphic_init();

#if (defined(EI_CLASSIFIER_SENSOR) && (EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_CAMERA))
    xCamera_count_sem = xSemaphoreCreateCounting(2, 0);
    if (xTaskCreate(camera_task,
        (const char*) "Camera task",
        CAMERA_TASK_STACK_SIZE_BYTE / 4, // in words
        NULL, //pvParameters
        CAMERA_TASK_PRIORITY, //uxPriority
        &camera_task_handle) != pdPASS) {
        ei_printf("Failed to create camera task\r\n");
        return;
    }

    snapshot_state = e_snapshot_lcd_inference;

#else
    snapshot_state = e_snapshot_idle;
#endif

    while(1) {
        // get notify from display callback
        //ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        xEventGroupSetBits(lcd_state_group, 1);
        graphic_start_buffer();

        // always draw logo
        draw_logo();

        switch (snapshot_state) {
            case e_snapshot_idle:                
                // do nothing
                break;
            case e_snapshot_stream:
                // pause stream
                // do nothing
                break;
            case e_snapshot_ingestion:  // send to ingestion
                // pause stream
                // do nothing
                break;
#if (defined(EI_CLASSIFIER_SENSOR) && (EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_CAMERA))
            case e_snapshot_lcd_inference:  // run inference and show on LCD

                lcd_inference();
                break;
#endif
            case e_snapshot_inference:
                // run inference via serial
                // and print on the display
                gFps = -1;
                lcd_print_result();
                break;
            default:
                break;
        }

        graphic_end_frame();

        /* swap the active buffer */
        disp_next_frame();
    }

    (void)pvParameters;

    if (lcd_task_handle != NULL) {
        lcd_task_handle = NULL;
        vTaskDelete(NULL);
    }  
    
    while (1) {
        vTaskDelay(portMAX_DELAY);
    }

}

#if (defined(EI_CLASSIFIER_SENSOR) && (EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_CAMERA))
bool camera_task_is_running(void)
{
    return camera_is_running;
}

/**
 * 
 */
static void camera_task(void *pvParameters)
{
    (void)pvParameters;
    EventBits_t event_bit;
    camera_is_running = true;

#if 0
    while (_aa_snapshot_ready == true) {
        _aa_snapshot_ready = _aa_snapshot_ready;
    }
#endif

    camera_start_stream();
    
    while(1) {
        if (camera_is_running) {
            event_bit = xEventGroupWaitBits(common_event_group, 
                EVENT_STOP_LCD,    //  uxBitsToWaitFor 
                pdTRUE,                 //  xClearOnExit
                pdFALSE,                //  xWaitForAllBits
                0);

                if (event_bit & EVENT_STOP_LCD) { 
                    camera_is_running = false;
                    xEventGroupSetBits(common_event_group, EVENT_STOP_CAMERA);
                }
        }

        if (!camera_is_running) {
            event_bit = xEventGroupWaitBits(common_event_group, 
                EVENT_START_CAMERA,    //  uxBitsToWaitFor 
                pdTRUE,                 //  xClearOnExit
                pdFALSE,                //  xWaitForAllBits
                portMAX_DELAY);

                if (event_bit & EVENT_START_CAMERA) { 
                    camera_start_stream();  // start stream
                    camera_is_running = true;
                }
        }
        
        if (camera_capture_stream_frame(lcd_camera_buffer[camera_buffer_index], CAMERA_CANVAS_WIDTH, CAMERA_CANVAS_HEIGHT) == false)  {
            // what if false ?
            ei_printf("Camera capture failed\r\n");
        }
        else {
            // swap buffer
            camera_buffer_ptr = lcd_camera_buffer[camera_buffer_index];
            camera_buffer_index ^= 1;    
        }
        
        xSemaphoreGive(xCamera_count_sem);
    }

    if (camera_task_handle != NULL) {
        camera_task_handle = NULL;
        vTaskDelete(NULL);
    }  
    
    while (1) {
        vTaskDelay(portMAX_DELAY);
    }
}

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
/**
 *
 * @param pbox
 */
static void set_lcd_detection(ei_impulse_result_bounding_box_t* pbox)
{
    float ratio = (float)CAMERA_CANVAS_HEIGHT / (float)EI_CLASSIFIER_INPUT_HEIGHT;

#if (EI_CLASSIFIER_OBJECT_DETECTION_LAST_LAYER == EI_CLASSIFIER_LAST_LAYER_FOMO)
    graphic_set_centroid(pbox->label, pbox->x, pbox->y, pbox->width, pbox->height, pbox->value, ratio);

#else
    graphic_set_box(pbox->label, pbox->x, pbox->y, pbox->width, pbox->height, pbox->value, ratio);
#endif

}
#endif // EI_CLASSIFIER_OBJECT_DETECTION == 1

/**
 * 
 */
static void lcd_inference(void)
{
    ei_impulse_result_t result = {0};
    static uint32_t old_time = 0;
    static uint32_t new_time = 0;

    if (camera_is_running == false) {
        vTaskDelay(100); // wait a bit
        return;
    }

    // get sync with camera task
    xSemaphoreTake(xCamera_count_sem, portMAX_DELAY);    

    // draw logo and camera screen    
    display_camera_screen(camera_buffer_ptr, CAMERA_CANVAS_WIDTH, CAMERA_CANVAS_HEIGHT);

    // run impulse
    ei_run_stream_impulse(camera_buffer_ptr, CAMERA_CANVAS_WIDTH, CAMERA_CANVAS_HEIGHT, &result);        
    
    // get time
    new_time = (uint32_t)ei_read_timer_ms();
    gFps = 1000/(new_time - old_time);
    old_time = new_time;

    lcd_set_result(&result);
    lcd_print_result(true);    
}

#endif  // CAMERA


/**
 * @brief Print classification results
 * 
 * @param result
 */
void lcd_set_result(ei_impulse_result_t *result)
{
    memcpy(&lcd_result, result, sizeof(ei_impulse_result_t));
}

/**
 * 
 */
static void lcd_print_result(bool is_live)
{
    graphic_set_timing(gFps, (uint32_t)lcd_result.timing.dsp_us, (uint32_t)lcd_result.timing.classification_us);

    // draw obj detection or classification based on model
#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    graphic_start_label("#Object detection results:", is_live);
    
    bool bb_found = lcd_result.bounding_boxes[0].value > 0;
            
    for (size_t ix = 0; ix < lcd_result.bounding_boxes_count; ix++) {
        auto bb = lcd_result.bounding_boxes[ix];
        if (bb.value == 0) {
            continue;
        }

        // set detection
        if (is_live) {  // draw
            set_lcd_detection((ei_impulse_result_bounding_box_t*)&bb);
        }
        graphic_set_detection_text(bb.label, bb.x, bb.y, bb.width, bb.height, bb.value);
       
    }

    if (bb_found == false) {
        graphic_no_detection();
    }

#else  // EI_CLASSIFIER_OBJECT_DETECTION == 0
    graphic_start_label("#Classification results:", is_live);
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        graphic_update_classification(lcd_result.classification[ix].label, lcd_result.classification[ix].value);
    }
#endif // EI_CLASSIFIER_OBJECT_DETECTION == 1

}
