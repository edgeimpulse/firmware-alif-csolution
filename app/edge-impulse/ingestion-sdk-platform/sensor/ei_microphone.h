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
#ifndef _EI_MICROPHONE_H_
#define _EI_MICROPHONE_H_

#include <cstdint>
#include <cstdlib>

typedef int16_t microphone_sample_t;

extern int ei_microphone_init(void);
extern bool ei_microphone_sample_record(void);
extern int ei_microphone_inference_get_data(size_t offset, size_t length, float *out_ptr);
extern bool ei_microphone_inference_start(uint32_t n_samples, float interval_ms);
extern bool ei_microphone_inference_record_continuous(void);
extern bool ei_microphone_inference_record(void);
extern bool ei_microphone_inference_is_recording(void);
extern void ei_microphone_inference_reset_buffers(void);
extern bool ei_microphone_inference_end(void);
extern int ei_microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr);

#endif
