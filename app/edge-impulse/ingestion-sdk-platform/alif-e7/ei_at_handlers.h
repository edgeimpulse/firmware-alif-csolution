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

#ifndef EI_AT_HANDLERS_H
#define EI_AT_HANDLERS_H

#include "firmware-sdk/at-server/ei_at_server.h"
#include "ingestion-sdk-platform/alif-e7/ei_device_alif_e7.h"

ATServer *ei_at_init(EiDeviceAlif *device);

#endif /* AT_HANDLERS_H_ */
