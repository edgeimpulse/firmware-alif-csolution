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
#include "task.h"
#include "ei_main.h"
#include "peripheral/peripheral.h"

int main (void)
{
    peripheral_init();

    ei_main_start();

    // Start thread execution
    vTaskStartScheduler();

    //while (1) __WFI();

    return 0;
}
