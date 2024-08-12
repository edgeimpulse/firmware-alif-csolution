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
#include "ei_main.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "edge-impulse/ingestion-sdk-platform/alif-e7/ei_at_handlers.h"
#include "edge-impulse/ingestion-sdk-platform/alif-e7/ei_device_alif_e7.h"
#include "edge-impulse/ingestion-sdk-platform/sensor/ei_microphone.h"
#include "peripheral/ei_uart.h"
#include "inference/ei_run_impulse.h"
#include "peripheral/inertial/bmi323_icm42670.h"
#include "ei_inertial.h"

// ei main task parameters
#define EI_MAIN_TASK_STACK_SIZE_BYTE        (4096u)
#define EI_MAIN_TASK_PRIORITY               (configMAX_PRIORITIES - 2)
static TaskHandle_t ei_main_task_handle;
static void ei_main_task(void *pvParameters);

void ei_main_start(void)
{
    if (xTaskCreate(ei_main_task,
        (const char*) "EI Main Thread",
        EI_MAIN_TASK_STACK_SIZE_BYTE / 4, // in words
        NULL, //pvParameters
        EI_MAIN_TASK_PRIORITY, //uxPriority
        &ei_main_task_handle) != pdPASS) {
        ei_printf("Failed to create EI Main Thread\r\n");
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
    EventBits_t event_bit;
    bool in_rx_loop = false;

    ei_printf("Type AT+HELP to see a list of commands.\r\n");
    ei_printf("Starting main loop\r\n");

    at = ei_at_init(dev);
    at->print_prompt();

    dev->get_camera()->init(320, 240);
    // TODO: ei_microphone_init returns an error code, should we check it?
    ei_microphone_init();
    // TODO: ei_inertial_init returns an error code, should we check it?
    ei_inertial_init();

#if 0
    while(1) {
        /* handle command comming from uart */
        char data = ei_get_serial_byte();

        while ((uint8_t)data != 0xFF) {
            
            if(is_inference_running() && data == 'b') {
                ei_stop_impulse();
                at->print_prompt();
                continue;
            }

            at->handle(data);
            data = ei_get_serial_byte();
        }

        if (is_inference_running() == true) {
            ei_run_impulse();
        }
        
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
            char data = ei_get_serial_byte(is_inference_running());

            in_rx_loop = false;

            while ((uint8_t)data != 0xFF) {
                if ((is_inference_running() == true) && (data == 'b') && (in_rx_loop == false)) {
                    ei_stop_impulse();
                    at->print_prompt();
                    continue;
                }

                in_rx_loop = true;
                at->handle(data);
                data = ei_get_serial_byte(is_inference_running());
            }
        }
    } // while(1)
}
