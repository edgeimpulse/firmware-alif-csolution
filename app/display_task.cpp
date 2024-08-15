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

#include "FreeRTOS.h"
#include "event_groups.h"
#include "task.h"
#include "common_events.h"
#include "display_task.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

#include "peripheral/display/display.h"
#include "peripheral/display/graphic.h"

// display task parameters
#define DISPLAY_TASK_STACK_SIZE_BYTE        (4096u)
#define DISPLAY_TASK_PRIORITY               (configMAX_PRIORITIES - 2)
static TaskHandle_t display_task_handle;
static void display_task(void *pvParameters);

/**
 * @brief 
 * 
 */
void display_task_start(void)
{
    if (xTaskCreate(display_task,
        (const char*) "EI Main Thread",
        DISPLAY_TASK_STACK_SIZE_BYTE / 4, // in words
        NULL, //pvParameters
        DISPLAY_TASK_PRIORITY, //uxPriority
        &display_task_handle) != pdPASS) {
        ei_printf("Failed to create Display task\r\n");
    }
}

/**
 * @brief 
 * 
 */
static void display_task(void *pvParameters)
{
    (void)pvParameters;
    EventBits_t event_bit;

    display_init();

    while(1) {
        event_bit = xEventGroupWaitBits(display_event_group, 
                                EVENT_VSYNC,            //  uxBitsToWaitFor 
                                pdTRUE,                 //  xClearOnExit
                                pdFALSE,                //  xWaitForAllBits
                                portMAX_DELAY);

        if (event_bit & EVENT_VSYNC) {
            /* prepare frame buffer */
            graphic_start_buffer();

            // update
            graphic_display_draw();

            /* end of graphic operation */
            graphic_end_frame();


            /* Swap the active framebuffer */
            display_swap_buffer();

            /* Now that the framebuffer is ready, update the GLCDC buffer pointer on the next Vsync */
            display_change_buffer();
        }
#if 0
        // wait vsync
        xSemaphoreTake(g_vsync_display, portMAX_DELAY);

        if (xSemaphoreTake(g_new_fb_semaphore, 0) == pdTRUE) {  // sync on new image to be displayed
            /* prepare frame buffer */
            graphic_start_buffer();

            // update
            graphic_display_draw();

            /* end of graphic operation */
            graphic_end_frame();


            /* Swap the active framebuffer */
            display_swap_buffer();

            /* Now that the framebuffer is ready, update the GLCDC buffer pointer on the next Vsync */
            display_change_buffer();
        }
#endif
    }
}
