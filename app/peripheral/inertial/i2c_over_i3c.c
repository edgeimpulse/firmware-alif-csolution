/* Copyright (C) 2022 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/* Includes ---------------------------------------------------------------- */
#include <stdio.h>
#include "Driver_I3C.h"
#include "pinconf.h"

#include <RTE_Components.h>
#include CMSIS_device_header
#if defined(RTE_Compiler_IO_STDOUT)
#include "retarget_stdout.h"
#endif  /* RTE_Compiler_IO_STDOUT */

#include <stdarg.h>

/* i3c Driver */
extern ARM_DRIVER_I3C Driver_I3C;
static ARM_DRIVER_I3C *I3Cdrv = &Driver_I3C;


/* i3c callback events */
typedef enum _I3C_CB_EVENT{
    I3C_CB_EVENT_SUCCESS        = (1 << 0),
    I3C_CB_EVENT_ERROR          = (1 << 1)
}I3C_CB_EVENT;

volatile int32_t cb_event_flag = 0;

/**
  \fn          INT hardware_init(void)
  \brief       i3c hardware pin initialization:
                - PIN-MUX configuration
                - PIN-PAD configuration
  \param[in]   void
  \return      ARM_DRIVER_OK : success; ARM_DRIVER_ERROR : failure
*/
static int32_t hardware_init(void)
{

  /* I3C_SDA_D */
  pinconf_set(PORT_7, PIN_6, PINMUX_ALTERNATE_FUNCTION_6,
          PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP | \
          PADCTRL_OUTPUT_DRIVE_STRENGTH_4MA);

  /* I3C_SCL_D */
  pinconf_set( PORT_7, PIN_7, PINMUX_ALTERNATE_FUNCTION_6,
          PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP | \
          PADCTRL_OUTPUT_DRIVE_STRENGTH_4MA);

    return ARM_DRIVER_OK;
}

/**
  \fn          void I3C_callback(UINT event)
  \brief       i3c isr callback
  \param[in]   event: i3c Event
  \return      none
*/
static void I3C_callback(uint32_t event)
{
    if (event & ARM_I3C_EVENT_TRANSFER_DONE)
    {
        /* Transfer Success */
        cb_event_flag = I3C_CB_EVENT_SUCCESS;
    }

    if (event & ARM_I3C_EVENT_TRANSFER_ERROR)
    {
        /* Transfer Error */
        cb_event_flag = I3C_CB_EVENT_ERROR;
    }
}

/**
 * @brief  Initialize I3C driver and attach all the i2c devices
 * @param  i2c_devices: pointer to i2c slave devices
 * @param  num_i2c_devices: number of i2c slave devices
 */
int i2c_over_i3c_init(uint8_t *i2c_devices, uint8_t num_i2c_devices)
{
    int32_t i = 0;
    int32_t ret = 0;
    ARM_DRIVER_VERSION version;

    /* Get i3c driver version. */
    version = I3Cdrv->GetVersion();

    if(version.api < 0x100 || version.drv < 0x100) {
        return -1;
    }

    /* Initialize i3c hardware pins using PinMux Driver. */
    ret = hardware_init();
    if(ret != ARM_DRIVER_OK) {
        return -2;
    }

    /* Initialize I3C driver */
    ret = I3Cdrv->Initialize(I3C_callback);
    if(ret != ARM_DRIVER_OK) {
        return -3;
    }

    /* Power up I3C peripheral */
    ret = I3Cdrv->PowerControl(ARM_POWER_FULL);
    if(ret != ARM_DRIVER_OK) {
        goto error_uninitialize;
    }

    /* i2c Speed Mode Configuration:
     *  I3C_BUS_MODE_MIXED_FAST_I2C_FMP_SPEED_1_MBPS  : Fast Mode Plus   1 MBPS
     *  I3C_BUS_MODE_MIXED_FAST_I2C_FM_SPEED_400_KBPS : Fast Mode      400 KBPS
     *  I3C_BUS_MODE_MIXED_SLOW_I2C_SS_SPEED_100_KBPS : Standard Mode  100 KBPS
     */
    ret = I3Cdrv->Control(I3C_MASTER_SET_BUS_MODE,
                           I3C_BUS_MODE_MIXED_SLOW_I2C_SS_SPEED_100_KBPS);
    if(ret != ARM_DRIVER_OK) {
        goto error_poweroff;
    }

    /* Attach all the slave address */
    for(i=0; i<num_i2c_devices; i++)
    {
        // cei_printf("\r\n  >> i=%d attaching i2c slave addr:0x%X to i3c...\r\n",  \
        //                    i, i2c_devices[i]);

        ret = I3Cdrv->AttachI2Cdev(i2c_devices[i]);
        if(ret != ARM_DRIVER_OK) {
            goto error_poweroff;
        }
    }

    /* Init successful */
    return 0;

error_detach:

    /* Detach all attached slave address */
    for(i=0; i<num_i2c_devices; i++) {
        I3Cdrv->Detachdev(i2c_devices[i]);
    }

error_poweroff:

    /* Power off I3C peripheral */
    I3Cdrv->PowerControl(ARM_POWER_OFF);

error_uninitialize:

    /* Un-initialize I3C driver */
    I3Cdrv->Uninitialize();
    return -4;
}


/**
 * @brief  Transfer data to/from i2c slave devices using i3c IP
 * @param  dev: i2c slave device address
 * @param  tx: pointer to tx data
 * @param  tx_len: length of tx data
 * @param  rx: pointer to rx data
 * @param  rx_len: length of rx data
 */
void i2c_over_i3cTransfer(uint8_t dev, uint8_t *tx, uint16_t tx_len, uint8_t *rx, uint16_t rx_len)
{
    int retry_cnt = 1000;
    int ret = 0;

    /* For TX, User has to pass
        * Slave Address + TX data + length of the TX data.
        */
    cb_event_flag = 0;
    ret = I3Cdrv->MasterTransmit(dev, tx, tx_len);
    if(ret != ARM_DRIVER_OK) {
        goto error_detach;
    }

    /* wait till any event success/error comes in isr callback */
    retry_cnt = 1000;

    while (retry_cnt)
    {
        retry_cnt--;

        /* delay for 1 milli sec */
        sys_busy_loop_us (1000);

        if(cb_event_flag == I3C_CB_EVENT_SUCCESS) {
            break;
        }

        if(cb_event_flag == I3C_CB_EVENT_ERROR) {
            break;
        }
    }

    if ( (!(retry_cnt)) && (!(cb_event_flag)) ) {
        goto error_detach;
    }

    /* For RX, User has to pass
        * Slave Address + Pointer to RX data + length of the RX data.
        */
    if(rx_len > 0) {

        cb_event_flag = 0;
        ret = I3Cdrv->MasterReceive(dev, rx, rx_len);
        if(ret != ARM_DRIVER_OK) {
            goto error_detach;;
        }

        /* wait till any event success/error comes in isr callback */
        retry_cnt = 1000;

        while (retry_cnt)
        {
            retry_cnt--;

            /* delay for 1 milli sec */
            sys_busy_loop_us (1000);

            if(cb_event_flag == I3C_CB_EVENT_SUCCESS) {
                return;
            }

            if(cb_event_flag == I3C_CB_EVENT_ERROR) {
                return;
            }

        }

        if ( (!(retry_cnt)) && (!(cb_event_flag)) ) {
            goto error_detach;
        }
    }
    else {
        return;
    }


error_detach:

    /* Detach all attached slave address */
    ret = I3Cdrv->Detachdev(dev);

error_poweroff:

    /* Power off I3C peripheral */
    ret = I3Cdrv->PowerControl(ARM_POWER_OFF);

error_uninitialize:

    /* Un-initialize I3C driver */
    ret = I3Cdrv->Uninitialize();
}


/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
