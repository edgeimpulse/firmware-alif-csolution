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
#include "task.h"
#include "event_groups.h"
#include "common_events.h"
#include "inference_task.h"
#include "inference/ei_run_impulse.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "lcd_task.h"
#include "model-parameters/model_metadata.h"

// Inference task parameters
#define INFERENCE_TASK_STACK_SIZE_BYTE        (4096u)
#define INFERENCE_TASK_PRIORITY               (configMAX_PRIORITIES - 3)
static TaskHandle_t inference_task_handle = NULL;
static void inference_task(void *pvParameters);

void inference_task_start(void)
{
    /* Only init once */
    if (inference_task_handle != NULL) {
        return;
    }

    if (xTaskCreate(inference_task,
        (const char*) "Inference task",
        INFERENCE_TASK_STACK_SIZE_BYTE / 4, // in words
        NULL, //pvParameters
        INFERENCE_TASK_PRIORITY, //uxPriority
        &inference_task_handle) != pdPASS) {
        ei_printf("Failed to create inference task\r\n");
    }
}

/**
 * @brief 
 * 
 * @param pvParameters 
 */
void inference_task(void *pvParameters)
{
    (void)pvParameters;
    // need vision model and LCD
#if (defined(LCD_SUPPORTED) && (LCD_SUPPORTED == 1)) && (defined(EI_CLASSIFIER_SENSOR) && (EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_CAMERA))
    EventBits_t event_bit;
    bool camera_task_was_running = false;

    if (camera_task_is_running()) {
        camera_task_was_running = true;
        // stop LCD if was running
        xEventGroupSetBits(common_event_group, EVENT_STOP_LCD);

        event_bit = xEventGroupWaitBits(common_event_group, 
            EVENT_STOP_CAMERA,    //  uxBitsToWaitFor 
            pdTRUE,                 //  xClearOnExit
            pdFALSE,                //  xWaitForAllBits
            portMAX_DELAY);
                
    }
    
#endif

#if (defined(LCD_SUPPORTED) && (LCD_SUPPORTED == 1))    // LCD supported
    t_snapshot_state old_state = lcd_get_state();
    lcd_set_state(e_snapshot_inference);
#endif

    while(is_inference_running() == true) {
        ei_run_impulse();
#if (defined(LCD_SUPPORTED) && (LCD_SUPPORTED == 1)) && (defined(EI_CLASSIFIER_SENSOR) && (EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_CAMERA))
        event_bit = xEventGroupWaitBits(common_event_group, 
            EVENT_STOP_INFERENCE,    //  uxBitsToWaitFor 
            pdTRUE,                 //  xClearOnExit
            pdFALSE,                //  xWaitForAllBits
            0);
            if (event_bit & EVENT_STOP_INFERENCE) {
                ei_stop_impulse();
                break;
            }
#endif
    }

    #if (defined(LCD_SUPPORTED) && (LCD_SUPPORTED == 1)) && (defined(EI_CLASSIFIER_SENSOR) && (EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_CAMERA))
    if (camera_task_was_running) {
        xEventGroupSetBits(common_event_group, EVENT_START_CAMERA);
    }

    
#endif

#if (defined(LCD_SUPPORTED) && (LCD_SUPPORTED == 1))    // LCD supported
    lcd_set_state(old_state);
#endif

    if (inference_task_handle != NULL) {
        inference_task_handle = NULL;
        vTaskDelete(NULL);
    }
    
    while (1) {
        vTaskDelay(portMAX_DELAY);
    }
}
