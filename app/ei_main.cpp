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

#include "board.h"

/**
 * @brief 
 * 
 */
void ei_main(void)
{
    ei_printf("Type AT+HELP to see a list of commands.\r\n");
    ei_printf("Starting main loop\r\n");

    while(1) {
        ei_sleep(500);

        BOARD_LED1_Control(BOARD_LED_STATE_TOGGLE);
        BOARD_LED2_Control(BOARD_LED_STATE_TOGGLE);
    }
}
