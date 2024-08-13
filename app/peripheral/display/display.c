#include "display.h"
#include "RTE_Components.h"
#include CMSIS_device_header
#include <RTE_Device.h>
#include "Driver_CDC200.h" // Display driver

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "event_groups.h"

#include "dave_driver.h"
#include "dave_d0lib.h"

#include "common_events.h"

#define USE_EVT_GROUP
#define USE_VSYNC

#ifndef MY_DISP_HOR_RES
    // Replace the macro MY_DISP_HOR_RES with the actual screen width.
    #define MY_DISP_HOR_RES    (RTE_PANEL_HACTIVE_TIME)
#endif

#ifndef MY_DISP_VER_RES
    // Replace the macro MY_DISP_HOR_RES with the actual screen height.
    #define MY_DISP_VER_RES    (RTE_PANEL_VACTIVE_LINE)
#endif

#if ((LV_COLOR_DEPTH == 16) && (RTE_CDC200_PIXEL_FORMAT != 2)) || \
    ((LV_COLOR_DEPTH == 24) && (RTE_CDC200_PIXEL_FORMAT != 1)) || \
    ((LV_COLOR_DEPTH == 32) && (RTE_CDC200_PIXEL_FORMAT != 0))
#error "The LV_COLOR_DEPTH and RTE_CDC200_PIXEL_FORMAT must match."
#endif

#if defined(USE_EVT_GROUP)
#define EVENT_DISP_BUFFER_READY     ( 1 << 0 )
#define EVENT_DISP_BUFFER_CHANGED   ( 1 << 1 )
#endif

#if (D1_MEM_ALLOC == D1_MALLOC_D0LIB)
// D/AVE D0 heap address and size
#define D1_HEAP_SIZE	0x80000
#endif

#if defined(USE_EVT_GROUP)
static EventGroupHandle_t dispEventGroupHandle = NULL;
#else
static volatile bool disp_buf_ready = false;
static volatile bool disp_buf_changed = false;
#endif

/**********************
 *      TYPEDEFS
 **********************/

#pragma pack(1)
#if RTE_CDC200_PIXEL_FORMAT == 0    // ARGB8888
typedef uint32_t Pixel;
#elif RTE_CDC200_PIXEL_FORMAT == 1  // RGB888
typedef uint32_t Pixel;
#elif RTE_CDC200_PIXEL_FORMAT == 2  // RGB565
typedef uint16_t Pixel;
#else
#error "CDC200 Unsupported color format"
#endif
#pragma pack()

#define BUFFER_CLEAR_VAL        (0x000000)
#define RED_COLOR_VAL           (0xFF0000)
#define GREEN_COLOR_VAL         (0x00FF00)
#define BLUE_COLOR_VAL          (0x0000FF)
#define YELLOW_COLOR_VAL        (0xFFFF00)
#define BLACK_COLO_VAL          (0xFFFFFF)

#define LCD_BUF_NUM    (2)

#if (D1_MEM_ALLOC == D1_MALLOC_D0LIB)
static uint8_t __attribute__((section(".bss.at_sram1"))) d0_heap[D1_HEAP_SIZE];
#endif

static Pixel lcd_buffer_1[MY_DISP_VER_RES][MY_DISP_HOR_RES]
            __attribute__((section(".bss.lcd_frame_buf"))) = {0};
static Pixel lcd_buffer_2[MY_DISP_VER_RES][MY_DISP_HOR_RES]
            __attribute__((section(".bss.lcd_frame_buf"))) = {0};

extern ARM_DRIVER_CDC200 Driver_CDC200;
static ARM_DRIVER_CDC200 *CDCdrv = &Driver_CDC200;

static volatile bool disp_flush_enabled = true;

static d2_device * d2_handle;
static uint8_t * p_framebuffer = NULL;

static uint8_t * g_p_single_buffer;
static uint8_t * g_p_double_buffer;

static void dave_init(void);
static void disp_callback(uint32_t event);
static void d2_start_rendering(void);
static void d2_finish_rendering(void);
static void d2_buf_switch(void);
static void d2_buf_clear(void);

/**
 * @brief 
 * 
 */
void display_init(void)
{
    /* Initialize CDC driver */
    int ret = CDCdrv->Initialize(disp_callback);
    if(ret != ARM_DRIVER_OK){
        //printf("\r\n Error: CDC init failed\n");
        __BKPT(0);
        return;
    }

    /* Power control CDC */
    ret = CDCdrv->PowerControl(ARM_POWER_FULL);
    if(ret != ARM_DRIVER_OK){
        //printf("\r\n Error: CDC Power up failed\n");
        __BKPT(0);
        return;
    }

    /* configure CDC controller */
    ret = CDCdrv->Control(CDC200_CONFIGURE_DISPLAY, (uint32_t)lcd_buffer_1);
    if(ret != ARM_DRIVER_OK){
        //printf("\r\n Error: CDC controller configuration failed\n");
        __BKPT(0);
        return;
    }

    /* Enable CDC SCANLINE0 event */
    ret = CDCdrv->Control(CDC200_SCANLINE0_EVENT, ENABLE);
    if(ret != ARM_DRIVER_OK){
        //printf("\r\n Error: CDC200_SCANLINE0_EVENT enable failed\n");
        __BKPT(0);
        return;
    }

    /* Start CDC */
    ret = CDCdrv->Start();
    if(ret != ARM_DRIVER_OK){
        //printf("\r\n Error: CDC Start failed\n");
        __BKPT(0);
        return;
    }

    dave_init();
}

/*Flush the content of the internal buffer the specific area on the display.
 *`px_map` contains the rendered image as raw pixel map and it should be copied to `area` on the display.
 *You can use DMA or any hardware acceleration to do this operation in the background but
 *'lv_display_flush_ready()' has to be called when it's finished.*/
static void disp_flush(uint8_t * px_map)
{
    if (disp_flush_enabled ) {
        d2_finish_rendering();

        //uint32_t size = lv_area_get_width(area) * lv_area_get_height(area)
        //                * lv_color_format_get_size(lv_display_get_color_format(disp_drv));
        //SCB_CleanInvalidateDCache_by_Addr(px_map, size);

#ifdef USE_VSYNC
        #if defined(USE_EVT_GROUP)
        xEventGroupClearBits(dispEventGroupHandle, EVENT_DISP_BUFFER_READY);
        #else
        disp_buf_ready = false;
        #endif

        CDCdrv->Control(CDC200_FRAMEBUF_UPDATE_VSYNC, (uint32_t)px_map);

        #if defined(USE_EVT_GROUP)
        xEventGroupSetBits(dispEventGroupHandle, EVENT_DISP_BUFFER_CHANGED);
        #else
        disp_buf_changed = true;
        #endif
#else
        CDCdrv->Control(CDC200_FRAMEBUF_UPDATE, (uint32_t)px_map);
#endif
    }
}

/* Display buffer flush waiting callback
 */
static void disp_flush_wait(void)
{
#ifdef USE_VSYNC
    #if defined(USE_EVT_GROUP)
    xEventGroupWaitBits(dispEventGroupHandle, EVENT_DISP_BUFFER_READY,
                        pdFALSE, pdFALSE, (100/portTICK_PERIOD_MS));
    #else
    while (!disp_buf_ready)
        sys_busy_loop_us(33);
    #endif
#endif
}

/* Display event handler
 */
static void disp_callback(uint32_t event)
{
    if (event & ARM_CDC_SCANLINE0_EVENT) {
        BaseType_t context_switch = pdFALSE;

        xEventGroupSetBitsFromISR(
                display_event_group,
                EVENT_VSYNC,
                &context_switch);
        portYIELD_FROM_ISR(context_switch);
#if 0
        if (xEventGroupGetBitsFromISR(dispEventGroupHandle)
           & EVENT_DISP_BUFFER_CHANGED) {
            xEventGroupClearBitsFromISR(
                dispEventGroupHandle,
                EVENT_DISP_BUFFER_CHANGED);

            BaseType_t context_switch = pdFALSE;
            xEventGroupSetBitsFromISR(
                dispEventGroupHandle,
                EVENT_DISP_BUFFER_READY,
                &context_switch);
            portYIELD_FROM_ISR(context_switch);
        }
#endif
    }

    if (event & ARM_CDC_DSI_ERROR_EVENT) {
        // Transfer Error: Received Hardware error.
        //__BKPT(0);
    }
    
}

static void dave_init(void)
{
    #if defined(USE_EVT_GROUP)
    // Create event group to sync display states
    dispEventGroupHandle = xEventGroupCreate();
    #endif
    
    /*-------------------------
     * Set display interrupt priority to FreeRTOS kernel level
     * -----------------------*/
    NVIC_SetPriority(CDC_SCANLINE0_IRQ_IRQn, configKERNEL_INTERRUPT_PRIORITY);

#if (D1_MEM_ALLOC == D1_MALLOC_D0LIB)
    /*-------------------------
     * Initialize D/AVE D0 heap
     * -----------------------*/
    if (!d0_initheapmanager(d0_heap, sizeof(d0_heap), d0_mm_fixed_range,
                            NULL, 0, 0, 0, d0_ma_unified))
    {
        //printf("\r\nError: Heap manager initialization failed\n");
        return;
    }
#endif

    /* Initialize D/AVE 2D driver */
    d2_handle = d2_opendevice(0);
    d2_inithw(d2_handle, 0);

#if 1
    /* Clear both buffers */
    d2_framebuffer(d2_handle, lcd_buffer_1, MY_DISP_VER_RES, MY_DISP_HOR_RES, MY_DISP_VER_RES * LCD_BUF_NUM, d2_mode_rgb565);
    d2_clear(d2_handle, BLUE_COLOR_VAL);

    /* Process active displaylist to clear framebuffers */
    d2_startframe(d2_handle);
    //d2_flushframe(d2_handle);
    d2_endframe(d2_handle);
#endif

    /* Set various D2 parameters */
    d2_setcolor(d2_handle, 0, 0xffffff );   // set white
    d2_setblendmode(d2_handle, d2_bm_alpha, d2_bm_one_minus_alpha);
    d2_setalphamode(d2_handle, d2_am_constant);
    d2_setalpha(d2_handle, 0xff);
    d2_setantialiasing(d2_handle, 1);
    d2_setlinecap(d2_handle, d2_lc_butt);
    d2_setlinejoin(d2_handle, d2_lj_none);
}


static void d2_start_rendering(void)
{
    // Wait for previous rendering to finish
    d2_endframe(d2_handle);

    // Switch and clear buf
    d2_buf_switch();
    d2_buf_clear();

    // Execute the next render buffer
    d2_startframe(d2_handle);
}

static void d2_finish_rendering(void)
{
    d2_endframe(d2_handle);
}

static void d2_buf_switch(void)
{
    //_d2_buf_act = _d2_buf_act == &_d2_buf_1 ? &_d2_buf_2 : &_d2_buf_1;
}

static void d2_buf_clear(void)
{
    //_lv_ll_clear_custom(_d2_buf_act, d2_buf_clear_cb);
}