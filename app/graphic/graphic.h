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

#ifndef _GRAPHIC_H_
#define _GRAPHIC_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif
#define MY_DISP_HOR_RES    (RTE_PANEL_HACTIVE_TIME)

#define MY_DISP_VER_RES    (RTE_PANEL_VACTIVE_LINE)

#define EVENT_DISP_BUFFER_READY     ( 1 << 0 )
#define EVENT_DISP_BUFFER_CHANGED   ( 1 << 1 )

extern void graphic_init(void);
extern void graphic_start_buffer(void);
extern void graphic_end_frame(void);
extern void disp_next_frame(void);

extern void draw_logo(void);
extern void graphic_set_centroid(const char* label, uint16_t x0, uint16_t y0, uint16_t width, uint16_t height, float value, float ratio);
extern void graphic_set_box(const char* label, uint16_t x0, uint16_t y0, uint16_t width, uint16_t height, float value, float ratio);
extern void graphic_set_detection_text(const char* label, uint16_t x0, uint16_t y0, uint16_t width, uint16_t height, float value);
extern void display_camera_screen(uint8_t* pbuff, uint16_t width, uint16_t height);

extern void graphic_start_text_obj_detection(void);
extern void graphic_no_detection(void);
extern void graphic_set_timing(int16_t fps, uint32_t dsp_us, uint32_t classification_us);
extern void graphic_start_label(const char* initial_string, bool is_live);
extern void graphic_update_classification(const char* label, float value);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif // _GRAPHIC_H_
