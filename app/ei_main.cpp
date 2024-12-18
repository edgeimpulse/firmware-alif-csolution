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

#include "ei_main.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "edge-impulse/ingestion-sdk-platform/alif-e7/ei_at_handlers.h"
#include "edge-impulse/ingestion-sdk-platform/alif-e7/ei_device_alif_e7.h"
#include "edge-impulse/ingestion-sdk-platform/sensor/ei_microphone.h"
#include "npu/npu_handler.h"
#include "board.h"
#include "peripheral/ei_uart.h"
#include "inference/ei_run_impulse.h"
#include <stdio.h>
#include "ei_inertial.h"

/**
 * @brief 
 * 
 */
void ei_main(void)
{
    EiDeviceAlif *dev = static_cast<EiDeviceAlif*>(EiDeviceInfo::get_device());
    ATServer *at;

    if (npu_init()) {
        BOARD_LED1_Control(BOARD_LED_STATE_TOGGLE);
        BOARD_LED2_Control(BOARD_LED_STATE_TOGGLE);
        ei_sleep(1000);
    }

    cpu_cache_enable();

    /* This is needed so that output of printf
    is output immediately without buffering
    */

    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);

    ei_printf("Type AT+HELP to see a list of commands.\r\n");
    ei_printf("Starting main loop\r\n");

    at = ei_at_init(dev);
    at->print_prompt();

    dev->get_camera()->init(320, 240);
    // TODO: ei_microphone_init returns an error code, should we check it?
    ei_microphone_init();
    // TODO: ei_inertial_init returns an error code, should we check it?
    ei_inertial_init();

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
}
