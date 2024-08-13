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

#include "RTE_Components.h"
#include CMSIS_device_header
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "event_groups.h"
#include "task.h"
#include "ei_main.h"
#include "display_task.h"
#include "peripheral/peripheral.h"
#include "npu/npu_handler.h"
#include <cstdio>
#include "board.h"

EventGroupHandle_t common_event_group;
EventGroupHandle_t display_event_group;

uint8_t ucHeap[ configTOTAL_HEAP_SIZE ] __attribute__((aligned(32), section(".heap")));

int main (void)
{
    peripheral_init();    
    cpu_cache_enable();

    if (npu_init()) {
        BOARD_LED1_Control(BOARD_LED_STATE_TOGGLE);
        BOARD_LED2_Control(BOARD_LED_STATE_TOGGLE);
        //ei_sleep(1000);
    }
    /* This is needed so that output of printf
    is output immediately without buffering
    */

    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);

    ei_main_start();
    display_task_start();

    common_event_group = xEventGroupCreate();
    display_event_group = xEventGroupCreate();

    // Start thread execution
    vTaskStartScheduler();

    //while (1) __WFI();

    return 0;
}
