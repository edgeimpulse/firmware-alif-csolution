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

#include "ei_main.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "edge-impulse/ingestion-sdk-platform/alif-e7/ei_at_handlers.h"
#include "edge-impulse/ingestion-sdk-platform/alif-e7/ei_device_alif_e7.h"
#include "edge-impulse/ingestion-sdk-platform/sensor/ei_microphone.h"
#include "board.h"
#include "peripheral/ei_uart.h"
#include "inference/ei_run_impulse.h"
#include <stdio.h>
#include "ei_inertial.h"
#include "common_events.h"
#include "model-parameters/model_metadata.h"

#if (defined(LCD_SUPPORTED) && (LCD_SUPPORTED == 1))
#include "lcd_task.h"
#endif

// main task parameters
#define EI_MAIN_TASK_STACK_SIZE_BYTE        (4096u)
#define EI_MAIN_TASK_PRIORITY               (configMAX_PRIORITIES - 1)
static TaskHandle_t ei_main_task_handle;
static void ei_main_task(void *pvParameters);

/**
 * 
 */
void start_ei_main(void)
{
    if (xTaskCreate(ei_main_task,
        (const char*) "EI Main Thread",
        EI_MAIN_TASK_STACK_SIZE_BYTE / 4, // in words
        NULL, //pvParameters
        EI_MAIN_TASK_PRIORITY, //uxPriority
        &ei_main_task_handle) != pdPASS) {        
        ei_printf("Failed to create EI Main Thread\r\n");    
        while(1);
    }

}

/**
 * @brief 
 * 
 */
static void ei_main_task(void *pvParameters)
{
    (void)pvParameters;
    EiDeviceAlif *dev = static_cast<EiDeviceAlif*>(EiDeviceInfo::get_device());
    ATServer *at;
    bool in_rx_loop = false;
    EventBits_t event_bit;

#if defined (CORE_M55_HE)
    ei_printf("Edge Impulse SDK - Alif Ensamble E7 HE core\r\n");
#else
    ei_printf("Edge Impulse SDK - Alif Ensamble E7 HP core\r\n");
#endif
    ei_printf("Type AT+HELP to see a list of commands.\r\n");
    ei_printf("Starting main loop\r\n");

    at = ei_at_init(dev);
    at->print_prompt();

    dev->get_camera()->init(320, 240);
    // TODO: ei_microphone_init returns an error code, should we check it?
    ei_microphone_init();
    // TODO: ei_inertial_init returns an error code, should we check it?
    ei_inertial_init();

#if (defined(LCD_SUPPORTED) && (LCD_SUPPORTED == 1))
    if (dev->get_camera()->is_camera_present()) {
        lcd_task_start();
    }
#endif

    // Event loop
    while (1) {

        event_bit = xEventGroupWaitBits(common_event_group, 
                                        EVENT_RX_READY,    //  uxBitsToWaitFor 
                                        pdTRUE,                 //  xClearOnExit
                                        pdFALSE,                //  xWaitForAllBits
                                        portMAX_DELAY);

        if (event_bit & EVENT_RX_READY) {
            char data = ei_get_serial_byte();

            in_rx_loop = false;

            while ((uint8_t)data != 0xFF) {
                if ((is_inference_running() == true) && (data == 'b') && (in_rx_loop == false)) {
#if (defined(LCD_SUPPORTED) && (LCD_SUPPORTED == 1)) && (defined(EI_CLASSIFIER_SENSOR) && (EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_CAMERA))
                    xEventGroupSetBits(common_event_group, EVENT_STOP_INFERENCE);
                    while(is_inference_running() == true) {
                        vTaskDelay(1);  // wait for inference to stop
                    }
#else
                    ei_stop_impulse();
#endif
                    at->print_prompt();
                    continue;
                }

                in_rx_loop = true;
                at->handle(data);
                data = ei_get_serial_byte();
            }
        }


    } // while(1)

    if (ei_main_task_handle != NULL) {
        ei_main_task_handle = NULL;
        vTaskDelete(NULL);
    }

    while (1) {
        vTaskDelay(portMAX_DELAY);
    }
}

void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
    ei_printf("%s\n",pcTaskName);

    for (;;) {
        //__WFE();
    }
}
