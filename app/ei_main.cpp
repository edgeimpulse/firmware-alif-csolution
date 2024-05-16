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
#include "npu/npu_handler.h"
#include "board.h"
#include "peripheral/ei_uart.h"

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

    ei_printf("Type AT+HELP to see a list of commands.\r\n");
    ei_printf("Starting main loop\r\n");

    at = ei_at_init(dev);

    while(1) {
        char data = ei_get_serial_byte();

        do {
            
            at->handle(data);

            data = ei_get_serial_byte();
        } while (data != 0xFF);
        
    }
}
