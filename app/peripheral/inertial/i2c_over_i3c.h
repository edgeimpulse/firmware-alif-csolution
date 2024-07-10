/* Copyright (C) 2022 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */
#ifndef I2C_OVER_I3C_H
#define I2C_OVER_I3C_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ---------------------------------------------------------------- */
#include <stdint.h>

/* Function prototypes ----------------------------------------------------- */
int i2c_over_i3c_init(uint8_t *i2c_devices, uint8_t num_i2c_devices);
int i2c_over_i3cTransfer(uint8_t dev, uint8_t *tx, uint16_t tx_len, uint8_t *rx, uint16_t rx_len);

#ifdef __cplusplus
}   // extern "C"
#endif

#endif // I2C_OVER_I3C_H