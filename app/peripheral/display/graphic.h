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

#ifndef _GRAPHIC_H_
#define _GRAPHIC_H_

#if defined(__cplusplus)
extern "C" {
#endif

typedef enum {
    e_snapshot_idle,
    e_snapshot_stream,
    e_snapshot_ingestion,
    e_snapshot_lcd,
    e_snapshot_lcd_inference,
    e_snapshot_inference,
} t_snapshot_state;



extern t_snapshot_state snapshot_get(void);
extern void snapshot_set(t_snapshot_state state);
extern void snapshot_handler(void);

#if defined(__cplusplus)
}
#endif

#endif
