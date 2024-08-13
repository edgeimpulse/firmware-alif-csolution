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

#include "graphic.h"

static t_snapshot_state snapshot_state = e_snapshot_lcd;

/**
 * @brief 
 * 
 * @return t_snapshot_state 
 */
t_snapshot_state snapshot_get(void)
{
    return snapshot_state;
}

/**
 * @brief 
 * 
 * @param state 
 */
void snapshot_set(t_snapshot_state state)
{
    snapshot_state = state;
}

/**
 * @brief 
 * 
 */
void snapshot_handler(void)
{
    switch (snapshot_state)
    {
        case e_snapshot_idle:
        case e_snapshot_stream:
        {
            // nothing to do in these states
        }        
        break;        
        case e_snapshot_ingestion:
        {
            // we can display the snapshot on the LCD
        }        
        break;
        case e_snapshot_lcd:
        {
            // camera stream on LCD
            //display_get_frame();
        }        
        break;
        case e_snapshot_lcd_inference:
        {
            // stream of camera inference on LCD
        }        
        break;
        case e_snapshot_inference:
        {
            // inference info on LCD
        }        
        break;
        default:
        {

        }
        break;
    }
}