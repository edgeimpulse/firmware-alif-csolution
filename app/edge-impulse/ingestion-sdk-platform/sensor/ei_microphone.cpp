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

#include "ei_microphone.h"

#include "RTE_Components.h"
#include "RTE_Device.h"
#include "ei_classifier_porting.h"
#include <Driver_SAI.h>
#include CMSIS_device_header
#include "board.h"

#include "ingestion-sdk-platform/alif-e7/ei_device_alif_e7.h"
#include "ingestion-sdk-c/sensor_aq_mbedtls_hs256.h"
#include "edge-impulse-sdk/dsp/numpy.hpp"
#include "firmware-sdk/sensor-aq/sensor_aq.h"

#define I2S_ADC       BOARD_I2S_INSTANCE

#define _I2S_IRQ(n)      I2S##n##_IRQ_IRQn
#define  I2S_IRQ(n)     _I2S_IRQ(n)

extern ARM_DRIVER_SAI ARM_Driver_SAI_(I2S_ADC);
ARM_DRIVER_SAI*       s_i2s_drv;

static bool create_header(sensor_aq_payload_info *payload);
static size_t ei_write(const void*, size_t size, size_t count, EI_SENSOR_AQ_STREAM*);
static int ei_seek(EI_SENSOR_AQ_STREAM*, long int offset, int origin);
static void audio_buffer_callback(void *buffer, uint32_t n_bytes);

static int insert_ref(char *buffer, int hdrLength);
static void write_value_to_cbor_buffer(uint8_t *buf, int16_t value);
static void I2SCallback(uint32_t event);

static bool ei_microphone_init_channels(void);
static bool ei_microphone_sample_start(void);
static bool ei_microphone_start(void);
static bool ei_microphone_stop(void);

//TODO: use multiply of memory block size
#define MIC_SAMPLE_SIZE_MONO        1600                            /* 50 ms - mono */
#define MIC_SAMPLE_SIZE_STEREO      (MIC_SAMPLE_SIZE_MONO * 2)      /* 50 ms - stereo */

static int16_t mic_buffer[MIC_SAMPLE_SIZE_STEREO] __attribute__((aligned(32), section(".bss.mic_buffer")));;

/* Private variables ------------------------------------------------------- */
static bool record_ready = false;
static bool is_recording = false;
static uint8_t n_audio_channels = 2;
const uint32_t sampling_rate = 16000;
static uint32_t cbor_current_sample;

static volatile bool rx_event;
static volatile bool rx_overflow_event;

/** Status and control struct for inferencing struct */
typedef struct {
    int16_t *buffers[2];
    uint8_t buf_select;
    uint8_t buf_ready;
    uint32_t buf_count;
    uint32_t n_samples;
} inference_t;

static inference_t inference;
static uint32_t headerOffset;
static uint32_t samples_required;
static uint32_t current_sample;

static unsigned char ei_mic_ctx_buffer[1024];
static sensor_aq_signing_ctx_t ei_mic_signing_ctx;
static sensor_aq_mbedtls_hs256_ctx_t ei_mic_hs_ctx;
static sensor_aq_ctx ei_mic_ctx = {
    { ei_mic_ctx_buffer, 1024 },
    &ei_mic_signing_ctx,
    &ei_write,
    &ei_seek,
    NULL,
};

/**
 * @brief 
 * 
 * @return int 
 */
int ei_microphone_init(void)
{
    int32_t status = 0;
    /* Use the I2S as Receiver */
    s_i2s_drv = &ARM_Driver_SAI_(I2S_ADC);

    /* Verify if I2S protocol is supported */
    ARM_SAI_CAPABILITIES cap = s_i2s_drv->GetCapabilities();
    if (!cap.protocol_i2s) {
        ei_printf("I2S is not supported\n");
        return false;
    }

    /* Initializes I2S interface */
    status = s_i2s_drv->Initialize(I2SCallback);
    if (status) {
        ei_printf("I2S Initialize failed status = %d\n", status);
        return false;
    }

    /* Enable the power for I2S */
    status = s_i2s_drv->PowerControl(ARM_POWER_FULL);
    if (status) {
        ei_printf("I2S Power failed status = %d\n", status);
        s_i2s_drv->Uninitialize();
        return false;
    }

    return true;
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool ei_microphone_sample_start_mono(void)
{
    n_audio_channels = 1;
    ei_microphone_init_channels();

    return ei_microphone_sample_start();
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool ei_microphone_sample_start_stereo(void)
{
    n_audio_channels = 2;
    ei_microphone_init_channels();

    return ei_microphone_sample_start();
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
static bool ei_microphone_sample_start(void)
{
    EiDeviceInfo *dev = EiDeviceInfo::get_device();
    EiDeviceMemory *mem = dev->get_memory();

    sensor_aq_payload_info payload = {
        dev->get_device_id().c_str(),
        dev->get_device_type().c_str(),
        dev->get_sample_interval_ms(),
        { { "audio", "wav" } }
    };

    if(n_audio_channels > 1) {
        payload.sensors[1] = { "audio2", "wav"};
    }

    ei_printf("Sampling settings:\n");
    ei_printf("\tInterval: %.5f ms.\n", dev->get_sample_interval_ms());
    ei_printf("\tLength: %lu ms.\n", dev->get_sample_length_ms());
    ei_printf("\tName: %s\n", dev->get_sample_label().c_str());
    ei_printf("\tHMAC Key: %s\n", dev->get_sample_hmac_key().c_str());
    ei_printf("\tFile name: /fs/%s\n", dev->get_sample_label().c_str());

    samples_required = (uint32_t)((dev->get_sample_length_ms()) / dev->get_sample_interval_ms());

    /* Round to even number of samples for word align flash write */
    if(samples_required & 1) {
        samples_required++;
    }

    current_sample = 0;
    cbor_current_sample = 0;

    ei_printf("Starting in 2000 ms... (or until all flash was erased)\n");
    ei_sleep(2000); // no need to erase

    if (create_header(&payload) == false) {
        return false;
    }

    // start
    rx_event = false;
    rx_overflow_event = false;

    ei_printf("Sampling...\r\n");
    is_recording = true;    
    
    record_ready = false;
    dev->set_state(eiStateSampling);

    while (record_ready == false) {
        ei_microphone_start();
        __WFI();
        ei_mic_thread(audio_buffer_callback);        
    };

    // stop
    ei_microphone_stop();

    dev->set_state(eiStateIdle);

    /* Write end of cbor + dummy */
    const uint8_t end_of_cbor[] = {0xff, 0xff, 0xff, 0xff};
    mem->write_sample_data((uint8_t*)end_of_cbor, headerOffset + cbor_current_sample, 4);

    int ctx_err =
        ei_mic_ctx.signature_ctx->finish(ei_mic_ctx.signature_ctx, ei_mic_ctx.hash_buffer.buffer);
    if (ctx_err != 0) {
        ei_printf("Failed to finish signature (%d)\n", ctx_err);
        return false;
    }

    ei_printf("Done sampling, total bytes collected: %lu\n", (current_sample * 2));
    ei_printf("[1/1] Uploading file to Edge Impulse...\n");
    ei_printf("Not uploading file, not connected to WiFi. Used buffer, from=0, to=%lu.\n", (cbor_current_sample + 1 + headerOffset));
    ei_printf("OK\n");

    return true;
}

/**
 * @brief 
 * 
 * @param n_samples 
 * @param interval_ms 
 * @return true 
 * @return false 
 */
bool ei_microphone_inference_start(uint32_t n_samples, uint8_t n_channels_inference, uint32_t freq)
{
    int32_t status = 0;

    if (n_channels_inference > 2) {
        ei_printf("Wrong number of channels\r\n");
        return false;
    }

    n_audio_channels = n_channels_inference;

    inference.buffers[0] = (int16_t *)ei_malloc(n_samples * sizeof(microphone_sample_t)  * n_audio_channels);
    if (inference.buffers[0] == NULL) {
        ei_printf("Can't allocate first audio buffer\r\n");
        return false;
    }

    inference.buffers[1] = (int16_t *)ei_malloc(n_samples * sizeof(microphone_sample_t)  * n_audio_channels);
    if (inference.buffers[1] == NULL) {
        ei_free(inference.buffers[0]);
        ei_printf("Can't allocate second audio buffer\r\n");
        return false;
    }

    inference.buf_select = 0;
    inference.buf_count = 0;
    inference.n_samples = n_samples;    // this represents the number of samples per channel
    inference.buf_ready = 0;

    ei_microphone_init_channels();
    return true;
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool ei_microphone_inference_record(void)
{
    bool recorded = false;

    //ei_mic_thread(&ei_mic_inference_samples_callback);

    // if not full, start again
    //if (inference.buf_ready == 0) {
        //ei_microphone_start();
    //}

    return recorded;
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool ei_microphone_inference_is_recording(void)
{
    return (inference.buf_ready == 0);
}

/**
 * @brief 
 * 
 */
void ei_microphone_inference_reset_buffers(void)
{
    inference.buf_ready = 0;
    inference.buf_count = 0;
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool ei_microphone_inference_end(void)
{
    ei_microphone_stop();

    ei_free(inference.buffers[0]);
    ei_free(inference.buffers[1]);

    return true; 
}

/**
 * @brief 
 * 
 * @param offset 
 * @param length 
 * @param out_ptr 
 * @return int 
 */
int ei_microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr)
{
    ei::numpy::int16_to_float(&inference.buffers[inference.buf_select ^ 1][offset], out_ptr, length);
    inference.buf_ready = 0;

    return 0;
}

/**
 * @brief Callback routine from the i2s driver.
 *
 * @param[in]  event  Event for which the callback has been called.
 */
static void I2SCallback(uint32_t event)
{
    if (event & ARM_SAI_EVENT_RECEIVE_COMPLETE) {
        rx_event = true;
    }
    else if (event & ARM_SAI_EVENT_RX_OVERFLOW) {
        rx_overflow_event = true;
    }
}

/**
 * @brief 
 * 
 * @param buffer 
 * @param sample_count 
 */
void ei_mic_inference_samples_callback(void *buffer, uint32_t sample_count)
{
    int16_t *samples = (int16_t *)buffer;

    for (uint32_t i = 0; i < (sample_count >> 1); i++) {
        inference.buffers[inference.buf_select][inference.buf_count++] = samples[i];

        if (inference.buf_count >= (inference.n_samples * n_audio_channels)) {  // n_samples is per channel

            inference.buf_select ^= 1;
            inference.buf_count = 0;
            inference.buf_ready = 1;
        }
    }   
}

/**
 * @brief 
 * 
 * @param buffer 
 * @param n_samples The number of bytes collected
 */
static void audio_buffer_callback(void *buffer, uint32_t n_bytes)
{
    EiDeviceMemory *mem = EiDeviceInfo::get_device()->get_memory();
    int16_t *sbuffer = (int16_t *)buffer;
    uint32_t sbuf_ptr = 0;
    uint32_t n_samples = n_bytes / 2;

    /* Calculate the cbor buffer length: header + 3 bytes per audio channel */
    uint32_t cbor_length = n_samples + ((n_samples + n_bytes) * n_audio_channels);    
    uint8_t *cbor_buf = (uint8_t *)ei_malloc(cbor_length);


    if (cbor_buf == NULL) {
        record_ready = true;
        is_recording = false;
        ei_printf("ERR: memory allocation error\r\n");
        return;
    }

    for (int i = 0; i < n_samples; i++) {
        uint32_t cval_ptr = i * ((n_audio_channels * 3) + 1);

        cbor_buf[cval_ptr] = 0x80 + n_audio_channels;
        for (int y = 0; y < n_audio_channels; y++) {
            write_value_to_cbor_buffer(&cbor_buf[cval_ptr + 1 + (3 * y)], sbuffer[sbuf_ptr + y]);
        }
        sbuf_ptr += n_audio_channels;
    }

    mem->write_sample_data((uint8_t*)cbor_buf, headerOffset + cbor_current_sample, cbor_length);

    ei_free(cbor_buf);

    cbor_current_sample += cbor_length;
    current_sample += n_samples;

    if (current_sample >= samples_required) {
        record_ready = true;
        is_recording = false;
    }

}

/**
 * @brief 
 * 
 * @param buffer 
 * @param size 
 * @param count 
 * @param stream 
 * @return size_t 
 */
static size_t ei_write(const void* buffer, size_t size, size_t count, EI_SENSOR_AQ_STREAM* stream)
{
    (void)buffer;
    (void)size;
    (void)stream;

    return count;
}

/**
 * @brief 
 * 
 * @param stream 
 * @param offset 
 * @param origin 
 * @return int 
 */
static int ei_seek(EI_SENSOR_AQ_STREAM* stream, long int offset, int origin)
{
    (void)stream;
    (void)offset;
    (void)origin;

    return 0;
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
static bool ei_microphone_start(void)
{
    int32_t status = 0;

    memset(mic_buffer, 0, sizeof(mic_buffer));

    rx_event = false;
    rx_overflow_event = false;
    /* Receive data */
    status = s_i2s_drv->Receive(mic_buffer, (MIC_SAMPLE_SIZE_MONO * n_audio_channels)); // Number of samples times number of channels
    if (status) {
        //ei_printf("ERR: I2S Receive status = %d\n", status);
        return false;
    }

    return true;
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool ei_microphone_start_inference_recording(void)
{
    return ei_microphone_start();
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
static bool ei_microphone_stop(void)
{
    int32_t status = 0;
    
    /* Receive data */
    status = s_i2s_drv->Control(ARM_SAI_CONTROL_RX, 0, 0);  // Uncomment to start/stop the mic.
    if (status) {
        ei_printf("ERR: I2S Control status = %d\n", status);
        return false;
    }

    return true;
}

/**
 * @brief Create a header object
 * 
 * @param payload 
 * @return true 
 * @return false 
 */
static bool create_header(sensor_aq_payload_info *payload)
{
    int ret;
    EiDeviceInfo *dev = EiDeviceInfo::get_device();
    EiDeviceMemory *mem = dev->get_memory();

    sensor_aq_init_mbedtls_hs256_context(&ei_mic_signing_ctx, &ei_mic_hs_ctx, dev->get_sample_hmac_key().c_str());

    ret = sensor_aq_init(&ei_mic_ctx, payload, NULL, true);

    if (ret != AQ_OK) {
        ei_printf("sensor_aq_init failed (%d)\n", ret);
        return false;
    }

    // then we're gonna find the last byte that is not 0x00 in the CBOR buffer.
    // That should give us the whole header
    size_t end_of_header_ix = 0;
    for (size_t ix = ei_mic_ctx.cbor_buffer.len - 1; ix != 0; ix--) {
        if (((uint8_t *)ei_mic_ctx.cbor_buffer.ptr)[ix] != 0x0) {
            end_of_header_ix = ix;
            break;
        }
    }

    if (end_of_header_ix == 0) {
        ei_printf("Failed to find end of header\n");
        return false;
    }

    // Write to blockdevice
    ret = mem->write_sample_data((uint8_t*)ei_mic_ctx.cbor_buffer.ptr, 0, end_of_header_ix);
    if ((size_t)ret != end_of_header_ix) {
        ei_printf("Failed to write to header blockdevice (%d)\n", ret);
        return false;
    }

    headerOffset = end_of_header_ix;

    return true;
}

/**
 * @brief Get the audio data object
 * 
 * @param callback 
 */
void ei_mic_thread(void (*callback)(void *buffer, uint32_t n_bytes))
{
    if (rx_event) {
        rx_event = false;
        callback(mic_buffer, (MIC_SAMPLE_SIZE_MONO * 2));   // in bytes !
    }
    else if (rx_overflow_event == true) {
        rx_overflow_event = false;
        ei_printf("I2S RX Overflow\n");
    }
}

/**
 *
 * @param buffer
 * @param hdrLength
 * @return
 */
static int insert_ref(char *buffer, int hdrLength)
{
    #define EXTRA_BYTES(a)  ((a & 0x3) ? 4 - (a & 0x3) : (a & 0x03))

    const char *ref = "Ref-BINARY-i16";
    int addLength = 0;
    int padding = EXTRA_BYTES(hdrLength);

    buffer[addLength++] = 0x60 + 14 + padding;
    for(unsigned int i = 0; i < strlen(ref); i++) {
        buffer[addLength++] = *(ref + i);
    }
    for(int i = 0; i < padding; i++) {
        buffer[addLength++] = ' ';
    }

    buffer[addLength++] = 0xFF;

    return addLength;
}

/**
 * @brief Convert to CBOR int16 format and write to buf
 */
static void write_value_to_cbor_buffer(uint8_t *buf, int16_t value)
{
    uint8_t datatype;
    uint16_t sample;

    if(value < 0) {
        datatype = 0x39;
        /* Convert from 2's complement */
        sample = (uint16_t)~value + 1;
    }
    else {
        datatype = 0x19;
        sample = value;
    }
    buf[0] = datatype;
    buf[1] = sample >> 8;
    buf[2] = sample & 0xFF;    
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
static bool ei_microphone_init_channels(void)
{
    int32_t status = 0;
    constexpr uint32_t wlen = 16;

    /* Configure I2S Receiver to Asynchronous Master */
    status = s_i2s_drv->Control(ARM_SAI_CONFIGURE_RX | ARM_SAI_MODE_MASTER | ARM_SAI_ASYNCHRONOUS |
                                ARM_SAI_PROTOCOL_I2S | ARM_SAI_DATA_SIZE(wlen) | ((n_audio_channels == 1) << 19) ,
                                (wlen * 2),
                                sampling_rate);

    if (status) {
        ei_printf("I2S Control status = %d\n", status);
        s_i2s_drv->PowerControl(ARM_POWER_OFF);
        s_i2s_drv->Uninitialize();
        return false;
    }

    status = s_i2s_drv->Control(ARM_SAI_CONTROL_RX, 1, 0); // Added here to keep recording going

    return (status == 0);
}
