/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef IMAGE_PROCESSING_H_
#define IMAGE_PROCESSING_H_

#include <stdint.h>
#include <stdbool.h>

#define RGB_BYTES 		3
#define RGBA_BYTES 		4
#define RGB565_BYTES	2
#define PIXEL_BYTES 	1

// Camera dimensions
#define CIMAGE_X		560
#define CIMAGE_Y		560

/*error status*/
#define FRAME_FORMAT_NOT_SUPPORTED   -1
#define FRAME_OUT_OF_RANGE           -2

#if defined(__cplusplus)
extern "C" {
#endif


extern uint32_t exposure_under_count, exposure_low_count, exposure_high_count, exposure_over_count;

int frame_crop(const void *input_fb, uint32_t ip_row_size, uint32_t ip_col_size, uint32_t row_start, uint32_t col_start, void *output_fb, uint32_t op_row_size, uint32_t op_col_size, uint32_t bpp);
int crop_and_interpolate(uint8_t *image, uint32_t srcWidth, uint32_t srcHeight, uint32_t dstWidth, uint32_t dstHeight, uint32_t bpp);
void white_balance(int width, int height, const uint8_t *sp, uint8_t *dp);
int bayer_to_RGB(uint8_t *src, uint8_t *dest);
void calculate_crop_dims(uint32_t srcWidth,
						 uint32_t srcHeight,
						 uint32_t dstWidth,
						 uint32_t dstHeight,
						 uint32_t *cropWidth,
						 uint32_t *cropHeight);
int resize_image(
    const uint8_t *srcImage,
    int srcWidth,
    int srcHeight,
    uint8_t *dstImage,
    int dstWidth,
    int dstHeight,
    int pixel_size_B,
    bool swapRB);

#if defined(__cplusplus)
}
#endif

#endif /* IMAGE_PROCESSING_H_ */
