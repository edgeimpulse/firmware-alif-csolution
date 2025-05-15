#include "dave_driver.h"
#include <stdint.h>

#ifndef TEXT_TEXT_H_
#define TEXT_TEXT_H_

#ifdef __cplusplus
extern "C" {
#endif

void text_print(d2_device *d2_handle, const char *string, uint8_t length, uint16_t x, uint16_t y, uint32_t color);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /* TEXT_TEXT_H_ */
