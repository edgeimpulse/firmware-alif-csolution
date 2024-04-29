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

#include "peripheral.h"
#include "board.h"
#include "se_services_port.h"
#include <stdio.h>
#include "uart_tracelib.h"
#include "camera.h"

static void clock_init(void);
static void uart_callback(uint32_t event);

extern void clk_init(void); // retarget.c

/**
 * @brief 
 * 
 */
void peripheral_init(void)
{
    BOARD_Pinmux_Init();

    /* Initialize the SE services */
    se_services_port_init();

    tracelib_init(NULL, uart_callback);

    clock_init();

    camera_init();
}

/**
 * @brief 
 * 
 */
static void clock_init(void)
{
    uint32_t service_error_code = 0;
    /* Enable Clocks */
    uint32_t error_code = SERVICES_clocks_enable_clock(se_services_s_handle, CLKEN_CLK_100M, true, &service_error_code);
    if(error_code || service_error_code){
        //printf("SE: 100MHz clock enable error_code=%u se_error_code=%u\n", error_code, service_error_code);
        return;
    }

    error_code = SERVICES_clocks_enable_clock(se_services_s_handle, CLKEN_HFOSC, true, &service_error_code);
    if(error_code || service_error_code){
        //printf("SE: HFOSC enable error_code=%u se_error_code=%u\n", error_code, service_error_code);
        return;
    }
}

static void uart_callback(uint32_t event)
{
    //
}
