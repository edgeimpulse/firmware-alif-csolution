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

#include "ei_at_handlers.h"
#include "firmware-sdk/at-server/ei_at_command_set.h"
#include "firmware-sdk/ei_device_lib.h"
//#include "model-parameters/model_metadata.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

EiDeviceAlif *pei_device;

/* Private function declaration */
static bool at_list_config(void);
static bool at_clear_config(void);

static bool at_device_info(void);
static bool at_get_sample_settings(void);
static bool at_set_sample_settings(const char **argv, const int argc);
static bool at_get_upload_settings(void);
static bool at_set_upload_settings(const char **argv, const int argc);
static bool at_set_upload_host(const char **argv, const int argc);

static bool at_read_buffer(const char **argv, const int argc);
static bool at_run_nn_normal(void);
static bool at_run_nn_normal_cont(void);
static bool at_run_impulse_debug(const char **argv, const int argc);
static bool at_run_nn_normal(void);
static bool at_run_nn_normal_cont(void);
static bool at_run_impulse_debug(const char **argv, const int argc);

static bool at_get_mgmt_settings(void);
static bool at_set_mgmt_settings(const char **argv, const int argc);

static bool at_get_snapshot(void);
static bool at_take_snapshot(const char **argv, const int argc);
static bool at_snapshot_stream(const char **argv, const int argc);

static inline bool check_args_num(const int &required, const int &received);

/* Public function definition */
/**
 *
 * @return
 */
ATServer *ei_at_init(EiDeviceAlif *ei_device)
{
    ATServer *at;

    at = ATServer::get_instance();
    pei_device = ei_device;

    //at->register_command(AT_CONFIG, AT_CONFIG_HELP_TEXT, nullptr, at_list_config, nullptr, nullptr);
    //at->register_command(AT_CLEARCONFIG, AT_CLEARCONFIG_HELP_TEXT, at_clear_config, nullptr, nullptr, nullptr);
    //at->register_command(AT_SAMPLESETTINGS, AT_SAMPLESETTINGS_HELP_TEXT, nullptr, at_get_sample_settings, at_set_sample_settings, AT_SAMPLESETTINGS_ARGS);
    //at->register_command(AT_RUNIMPULSE, AT_RUNIMPULSE_HELP_TEXT, at_run_nn_normal, nullptr, nullptr, nullptr);
    //at->register_command(AT_RUNIMPULSEDEBUG, AT_RUNIMPULSEDEBUG_HELP_TEXT, nullptr, nullptr, at_run_impulse_debug, AT_RUNIMPULSEDEBUG_ARGS);
    //at->register_command(AT_RUNIMPULSECONT, AT_RUNIMPULSECONT_HELP_TEXT, at_run_nn_normal_cont, nullptr, nullptr, nullptr);
    //at->register_command(AT_READBUFFER, AT_READBUFFER_HELP_TEXT, nullptr, nullptr, at_read_buffer, AT_READBUFFER_ARGS);
    //at->register_command(AT_MGMTSETTINGS, AT_MGMTSETTINGS_HELP_TEXT, nullptr, at_get_mgmt_settings, at_set_mgmt_settings, AT_MGMTSETTINGS_ARGS);
    //at->register_command(AT_UPLOADSETTINGS, AT_UPLOADSETTINGS_HELP_TEXT, nullptr, at_get_upload_settings, at_set_upload_settings, AT_UPLOADSETTINGS_ARGS);
    //at->register_command(AT_UPLOADHOST, AT_UPLOADHOST_HELP_TEXT, nullptr, nullptr, at_set_upload_host, AT_UPLOADHOST_ARGS);
    //at->register_command(AT_SNAPSHOT, AT_SNAPSHOT_HELP_TEXT, nullptr, at_get_snapshot, at_take_snapshot, AT_SNAPSHOT_ARGS);
    //at->register_command(AT_SNAPSHOTSTREAM, AT_SNAPSHOTSTREAM_HELP_TEXT, nullptr, nullptr, at_snapshot_stream, AT_SNAPSHOTSTREAM_ARGS);

    return at;
}

/**
 *
 * @return
 */
static bool at_list_config(void)
{
    ei_printf("===== Device info =====\r\n");
    at_device_info();
    ei_printf("\r\n");
    ei_printf("===== Sensors ======\r\n");
    ei_printf("\r\n");
    ei_printf("===== Snapshot ======\r\n");
    at_get_snapshot();
    ei_printf("\r\n");
    ei_printf("===== Inference ======\r\n");
    //ei_printf("Sensor:           %d\r\n", EI_CLASSIFIER_SENSOR);
#if EI_CLASSIFIER_OBJECT_DETECTION
    #if EI_CLASSIFIER_OBJECT_DETECTION_LAST_LAYER == EI_CLASSIFIER_LAST_LAYER_FOMO
        const char *model_type = "constrained_object_detection";
    #else
        const char *model_type = "object_detection";
    #endif
#else
    const char *model_type = "classification";
#endif
    ei_printf("Model type:       %s\r\n", model_type);
    ei_printf("\r\n");
    ei_printf("===== WIFI =====\r\n");
    ei_printf("SSID:      \r\n");
    ei_printf("Password:  \r\n");
    ei_printf("Security:  0\r\n");
    ei_printf("MAC:       00:00:00:00:00:00\r\n");
    ei_printf("Connected: 0\r\n");
    ei_printf("Present:   0\r\n");
    ei_printf("\r\n");
    ei_printf("===== Sampling parameters =====\r\n");
    at_get_sample_settings();
    ei_printf("\r\n");
    ei_printf("===== Upload settings =====\r\n");
    at_get_upload_settings();
    ei_printf("\r\n");
    ei_printf("===== Remote management =====\r\n");
    at_get_mgmt_settings();
    ei_printf("\r\n");

    return true;
}

/**
 *
 * @return
 */
static bool at_device_info(void)
{
    bool ret_val = false;

    if (pei_device != nullptr) {
        ei_printf("ID:         %s\r\n", pei_device->get_device_id().c_str());
        ei_printf("Type:       %s\r\n", pei_device->get_device_type().c_str());
        ei_printf("AT Version: %s\r\n", AT_COMMAND_VERSION);
        ei_printf("Data Transfer Baudrate: %lu\r\n", pei_device->get_data_output_baudrate());
        ret_val = true;
    }
    else {

    }

    return ret_val;
}

/**
 *
 * @return
 */
static bool at_clear_config(void)
{
    bool ret_val = false;

    if (pei_device != nullptr) {
        ei_printf("Clearing config and restarting system...\r\n");
        pei_device->clear_config();
        //pei_device->init_device_id(); // done in clear config
        ret_val = true;
    }
    else {

    }

    return ret_val;
}

/**
 *
 * @return
 */
static bool at_get_sample_settings(void)
{
    bool ret_val = false;

    if (pei_device != nullptr) {
        ei_printf("Label:     %s\r\n", pei_device->get_sample_label().c_str());
        ei_printf("Interval:  ");
        ei_printf_float(pei_device->get_sample_interval_ms());
        ei_printf(" ms.\r\n");
        ei_printf("Length:    %lu ms.\r\n", pei_device->get_sample_length_ms());
        ei_printf("HMAC key:  %s\r\n", pei_device->get_sample_hmac_key().c_str());
        ret_val = true;
    }
    else {

    }

    return ret_val;
}
