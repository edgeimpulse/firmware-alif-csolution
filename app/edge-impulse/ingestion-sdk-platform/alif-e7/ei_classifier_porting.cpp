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
#include "task.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include "peripheral/ei_uart.h"
#include "peripheral/timer.h"

/* Extern function prototypes ---------------------------------------------- */

EI_IMPULSE_ERROR ei_run_impulse_check_canceled()
{
    return EI_IMPULSE_OK;
}

/**
 * Cancelable sleep, can be triggered with signal from other thread
 */
EI_IMPULSE_ERROR ei_sleep(int32_t time_ms)
{
    if (time_ms < 0) { 
        return EI_IMPULSE_OK; 
    }

    uint64_t start_time = ei_read_timer_ms();
    // cast so that we get correct wrap around behavior
    while ((ei_read_timer_ms() - start_time) < (uint64_t) time_ms)
        ;
    return EI_IMPULSE_OK;
}

uint64_t ei_read_timer_us()
{
    return timer_get_us();
}

uint64_t ei_read_timer_ms()
{
    return ei_read_timer_us() / 1000;
}

void ei_printf(const char *format, ...)
{
    char buffer[1024] = {0};
    int length;

    va_list myargs;
    va_start(myargs, format);
    length = vsnprintf(buffer, sizeof(buffer), format, myargs);
    va_end(myargs);
    
    if (length > 0) {
        ei_uart_send(buffer, length);
    }
}

void ei_printf_float(float f)
{
    float n = f;

    static double PRECISION = 0.001;
    static int MAX_NUMBER_STRING_SIZE = 32;

    char s[MAX_NUMBER_STRING_SIZE];

    if (n == 0.0) {
        strcpy(s, "0");
    }
    else {
        int digit, m;
        char *c = s;
        int neg = (n < 0);
        if (neg) {
            n = -n;
        }
        // calculate magnitude
        m = log10(n);
        if (neg) {
            *(c++) = '-';
        }
        if (m < 1.0) {
            m = 0;
        }
        // convert the number
        while (n > PRECISION || m >= 0) {
            double weight = pow(10.0, m);
            if (weight > 0 && !isinf(weight)) {
                digit = floor(n / weight);
                n -= (digit * weight);
                *(c++) = '0' + digit;
            }
            if (m == 0 && n > 0) {
                *(c++) = '.';
            }
            m--;
        }
        *(c) = '\0';
    }

    ei_printf("%s", s);
}

void ei_putchar(char c) 
{ 
    ei_uart_send(&c, 1);
    fflush(stdout);
}

char ei_getchar(void)
{
    // dummy implementation
    char ch = 0;
    return ch;
}

__attribute__((weak)) void *ei_malloc(size_t size) {
    if (size > 0){
        return pvPortMalloc(size);
    }
    else {
        return NULL;
    }
}

__attribute__((weak)) void *ei_calloc(size_t nitems, size_t size) {
    if ((size*nitems) > 0) {
        return pvPortCalloc(nitems, size);
    }
    else {
        return NULL;
    }  
}

__attribute__((weak)) void ei_free(void *ptr) {
    vPortFree(ptr);
}

#if defined(__cplusplus) && EI_C_LINKAGE == 1
extern "C"
#endif
    void
    DebugLog(const char *s)
{
    ei_printf("%s", s);
}
