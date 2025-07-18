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
/* Include ----------------------------------------------------------------- */
#include "model-parameters/model_metadata.h"
#if defined(EI_CLASSIFIER_SENSOR) && (EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_MICROPHONE)
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "edge-impulse-sdk/dsp/numpy.hpp"
#include "inference/ei_run_impulse.h"
#include "model-parameters/model_variables.h"
#include "ingestion-sdk-platform/sensor/ei_microphone.h"
#include "edge-impulse/ingestion-sdk-platform/alif-e7/ei_device_alif_e7.h"
#include "inference_task.h"

#if (defined(LCD_SUPPORTED) && (LCD_SUPPORTED == 1))
#include "lcd_task.h"
#endif

typedef enum {
    INFERENCE_STOPPED,
    INFERENCE_WAITING,
    INFERENCE_SAMPLING,
    INFERENCE_DATA_READY
} inference_state_t;

/* Private variables ------------------------------------------------------- */
static int print_results;
static inference_state_t state = INFERENCE_STOPPED;
static uint64_t last_inference_ts = 0;
static bool continuous_mode = false;
static bool debug_mode = false;
static uint8_t inference_channels = 0;

/**
 * @brief 
 * 
 */
void ei_run_impulse(void)
{
    switch(state) {
        case INFERENCE_STOPPED:
        {
            // nothing to do
            return;
        }
        break;
        case INFERENCE_WAITING:
        {
            if (ei_read_timer_ms() < (last_inference_ts + 2000)) {
                return;
            }
            ei_printf("Recording\n");            
            ei_microphone_inference_reset_buffers();
            state = INFERENCE_SAMPLING;
            // fall down
        }
        case INFERENCE_SAMPLING:
        {
            do {
                if (ei_microphone_start_inference_recording() == false) {
                    //ei_printf("ERR: Failed to start audio sampling\r\n");
                    return;
                }
                
                ei_mic_thread(ei_mic_inference_samples_callback);
            }while(ei_microphone_inference_is_recording() == true);
            
            state = INFERENCE_DATA_READY;
            if (continuous_mode == false) {
                ei_printf("Recording done\n");
            }
        }
        break;
        case INFERENCE_DATA_READY:
        {
            // nothing to do, just continue to inference provcessing below
        }
        break;
        default:
        {

        }
        break;
    }

    signal_t signal;

    signal.total_length = continuous_mode ? (EI_CLASSIFIER_SLICE_SIZE * ei_default_impulse.impulse->raw_samples_per_frame): ei_default_impulse.impulse->dsp_input_frame_size;
    signal.get_data = &ei_microphone_audio_signal_get_data;

    // run the impulse: DSP, neural network and the Anomaly algorithm
    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR ei_error;
    if (continuous_mode == true) {
        ei_error = run_classifier_continuous(&signal, &result, debug_mode);
    }
    else {
        ei_error = run_classifier(&signal, &result, debug_mode);
    }
    if (ei_error != EI_IMPULSE_OK) {
        ei_printf("Failed to run impulse (%d)", ei_error);
        return;
    }

    if (continuous_mode == true) {
        if (++print_results >= (EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW >> 1)) {
            display_results(&ei_default_impulse, &result);
            print_results = 0;
#if (defined(LCD_SUPPORTED) && (LCD_SUPPORTED == 1))
            lcd_set_result(&result);
#endif
        }
    }
    else {
        display_results(&ei_default_impulse, &result);
#if (defined(LCD_SUPPORTED) && (LCD_SUPPORTED == 1))
        lcd_set_result(&result);
#endif
    }

    if (continuous_mode == true) {
        state = INFERENCE_SAMPLING;
    }
    else {
        ei_printf("Starting inferencing in 2 seconds...\n");
        last_inference_ts = ei_read_timer_ms();
        state = INFERENCE_WAITING;
    }
}

/**
 *
 * @param continuous
 * @param debug
 * @param use_max_uart_speed
 */
void ei_start_impulse(bool continuous, bool debug, bool use_max_uart_speed)
{
    const float sample_length = 1000.0f * static_cast<float>(EI_CLASSIFIER_RAW_SAMPLE_COUNT) /
                        (1000.0f / static_cast<float>(EI_CLASSIFIER_INTERVAL_MS));

    inference_channels = ei_default_impulse.impulse->raw_samples_per_frame;

    continuous_mode = continuous;
    debug_mode = debug;

    // summary of inferencing settings (from model_metadata.h)
    ei_printf("Inferencing settings:\n");
    ei_printf("\tInterval: ");
    ei_printf_float(EI_CLASSIFIER_INTERVAL_MS);
    ei_printf("ms.");
    ei_printf("\tFrame size: %d\n", ei_default_impulse.impulse->dsp_input_frame_size);
    ei_printf("\tSample length: ");
    ei_printf_float(sample_length);
    ei_printf(" ms.");
    ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) /
                                            sizeof(ei_classifier_inferencing_categories[0]));
    ei_printf("Starting inferencing, press 'b' to break\n");

    if (ei_microphone_inference_start(continuous_mode ? EI_CLASSIFIER_SLICE_SIZE : ei_default_impulse.impulse->raw_sample_count, inference_channels, ei_default_impulse.impulse->frequency) == false) {
        ei_printf("ERR: Could not allocate audio buffer (size %d), this could be due to the window length of your model\r\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT);        
        return;
    }

    if (continuous == true) {
        // In order to have meaningful classification results, continuous inference has to run over
        // the complete model window. So the first iterations will print out garbage.
        // We now use a fixed length moving average filter of half the slices per model window and
        // only print when we run the complete maf buffer to prevent printing the same classification multiple times.
        print_results = -(EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW);
        run_classifier_init();
        ei_microphone_start_inference_recording();
        state = INFERENCE_SAMPLING;
    }
    else {
        // it's time to prepare for sampling
        ei_printf("Starting inferencing in 2 seconds...\n");
        last_inference_ts = ei_read_timer_ms();
        state = INFERENCE_WAITING;
    }

    inference_task_start();
}


/**
 * @brief 
 * 
 */
void ei_stop_impulse(void)
{
    if (state != INFERENCE_STOPPED) {
        ei_microphone_inference_end();

        ei_printf("Inferencing stopped by user\r\n");

        /* reset samples buffer */
        run_classifier_deinit();
    }
    inference_channels = 0;
    state = INFERENCE_STOPPED;
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool is_inference_running(void)
{
    return (state != INFERENCE_STOPPED);
}

#endif