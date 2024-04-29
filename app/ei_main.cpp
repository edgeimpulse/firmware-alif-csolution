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
#include "board.h"
//#include "hal.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "se_services_port.h"

/**
 * @brief 
 * 
 */
void ei_init(void)
{
    //init_trigger_rx();
    //hal_platform_init();

    BOARD_Pinmux_Init();

    /* Initialize the SE services */
    se_services_port_init();
#if 0
    /* Initialise the camera */
    int err = hal_image_init();
    if (0 != err) {
        ei_printf("hal_image_init failed with error: %d\n", err);
    }
#endif
}

/**
 * @brief 
 * 
 */
void ei_main(void)
{

}
