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
/* Include ----------------------------------------------------------------- */
#include "model-parameters/model_metadata.h"
#if defined(EI_CLASSIFIER_SENSOR) && (EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_MICROPHONE)
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "edge-impulse-sdk/dsp/numpy.hpp"
#include "inference/ei_run_impulse.h"
#include "model-parameters/model_variables.h"
#include "ingestion-sdk-platform/sensor/ei_microphone.h"
#include "edge-impulse/ingestion-sdk-platform/alif-e7/ei_device_alif_e7.h"

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
static int samples_wr_index = 0;
static uint8_t inference_channels = 0;

static void local_display_results(ei_impulse_result_t* result);

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
                    ei_printf("ERR: Failed to start audio sampling\r\n");
                    return;
                }
                __WFI();
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
            local_display_results(&result);
            print_results = 0;
        }
    }
    else {
        local_display_results(&result);
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
}


/**
 * @brief 
 * 
 */
void ei_stop_impulse(void)
{
    if (state != INFERENCE_STOPPED) {

        if (continuous_mode == false) {
            if (state == INFERENCE_SAMPLING) {
                while (ei_microphone_inference_record() == false) {
                    ei_sleep(10);
                };
                ei_microphone_inference_end();
            }
        }
        else {
            ei_microphone_inference_end();
        }

        ei_printf("Inferencing stopped by user\r\n");

        /* reset samples buffer */
        samples_wr_index = 0;
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

static void local_display_results(ei_impulse_result_t* result)
{
    // print the predictions    
    ei_printf("Predictions (DSP: ");
    ei_printf_float((float)result->timing.dsp_us/1000.0);
    ei_printf(" ms., Classification: ");
    ei_printf_float((float)result->timing.classification_us/1000.0);
    ei_printf(" ms., Anomaly: ");
    ei_printf_float(result->timing.anomaly_us);
    ei_printf(" ms.): \n");

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    ei_printf("#Object detection results:\r\n");
    bool bb_found = result->bounding_boxes[0].value > 0;
    for (size_t ix = 0; ix < result->bounding_boxes_count; ix++) {
        auto bb = result->bounding_boxes[ix];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("    %s (", bb.label);
        ei_printf_float(bb.value);
        ei_printf(") [ x: %u, y: %u, width: %u, height: %u ]\n", bb.x, bb.y, bb.width, bb.height);
    }

    if (!bb_found) {
        ei_printf("    No objects found\n");
    }

#elif (EI_CLASSIFIER_LABEL_COUNT == 1) && (!EI_CLASSIFIER_HAS_ANOMALY)// regression
    ei_printf("#Regression results:\r\n");
    ei_printf("    %s: ", result->classification[0].label);
    ei_printf_float(result->classification[0].value);
    ei_printf("\n");

#elif EI_CLASSIFIER_LABEL_COUNT > 1 // if there is only one label, this is an anomaly only
    ei_printf("#Classification results:\r\n");
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("    %s: ", result->classification[ix].label);
        ei_printf_float(result->classification[ix].value);
        ei_printf("\n");
    }
#endif
#if EI_CLASSIFIER_HAS_ANOMALY == 3 // visual AD
    ei_printf("#Visual anomaly grid results:\r\n");
    for (uint32_t i = 0; i < result->visual_ad_count; i++) {
        ei_impulse_result_bounding_box_t bb = result->visual_ad_grid_cells[i];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("    %s (", bb.label);
        ei_printf_float(bb.value);
        ei_printf(") [ x: %u, y: %u, width: %u, height: %u ]\n", bb.x, bb.y, bb.width, bb.height);
    }
    ei_printf("Visual anomaly values: Mean %.3f Max %.3f\r\n", result->visual_ad_result.mean_value, result->visual_ad_result.max_value);
#elif (EI_CLASSIFIER_HAS_ANOMALY > 0) // except for visual AD
    ei_printf("Anomaly prediction: %.3f\r\n", result->anomaly);
#endif
}

#endif