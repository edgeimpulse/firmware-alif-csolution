#ifndef _DISPLAY_H_
#define _DISPLAY_H_

#include <stdint.h>

#if defined(__cplusplus)
extern "C" {
#endif

extern void display_init(void);
extern void display_swap_buffer(void);
extern void display_change_buffer(void);
extern void graphic_display_draw(void);
extern void graphic_start_buffer(void);
extern void graphic_end_frame(void);
extern uint8_t* graphic_get_draw_buffer(void);

#if defined(__cplusplus)
}
#endif

#endif