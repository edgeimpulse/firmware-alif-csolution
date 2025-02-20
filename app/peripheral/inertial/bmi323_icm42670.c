/*
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
 *  Created on: Feb 2, 2024
 *      Author: KevinBraun
 */

/* Includes ---------------------------------------------------------------- */
#include "bmi323_icm42670.h"
#include <stdio.h>
#include <string.h>

#include <RTE_Components.h>
#include CMSIS_device_header

#include "Driver_I3C.h"
#include "i2c_over_i3c.h"

/* Private variables ------------------------------------------------------- */
bool i2c_bmi323_ok = false;

uint32_t icm42670pCnt = 0;
uint32_t bmi323Cnt = 0;

uint8_t i3c_tx_data[4];
uint8_t i3c_rx_data[16];

/* Private functions ------------------------------------------------------- */

static void delay_10us(uint32_t delay)
{
	sys_busy_loop_us(delay * 10);
}


/* Public functions -------------------------------------------------------- */

/**
 * Enable I2C over I3C and initialize the BMI323 and ICM42670P
 */
int bmi323_icm42670_init(void)
{
	int retval = 0;
	//TODO: RETRUN THE ERROR CODES
	if (i2c_over_i3c_init((uint8_t[]){BMI323_ADDR, ICM42670_ADDR}, 2) != ARM_DRIVER_OK) {
		return -1;
	}
		
	if (initBMI323() == ARM_DRIVER_OK) {
		retval |= 1;
	}
	
	if (initICM42670P() == ARM_DRIVER_OK) {
		retval |= 2;
	}

	return retval;
}

/**
 * get a single 16 bit register.  Always returns 2 bytes
 */
int32_t getRegBMI323(uint8_t reg, uint16_t * aData)
{
	int32_t ret = ARM_DRIVER_ERROR;

	i3c_tx_data[0] = reg;
	ret = i2c_over_i3cTransfer(BMI323_ADDR, i3c_tx_data, 1U, i3c_rx_data, 4U);

	*aData = (i3c_rx_data[3] << 8U) | i3c_rx_data[2];

	return ret;
}

/**
 * get the BMI323 gyro data
 */
int32_t getBMI323Gyro(xyz_accel_gyro_imu_s *imu)
{
	int32_t ret = ARM_DRIVER_ERROR;

	i3c_tx_data[0] = BMI323_GYR_X_REG;
	ret = i2c_over_i3cTransfer(BMI323_ADDR, i3c_tx_data, 1U, i3c_rx_data, 8U);

	imu->gyro_x = (int16_t)((i3c_rx_data[3] << 8) | i3c_rx_data[2]);
	imu->gyro_y = (int16_t)((i3c_rx_data[5] << 8) | i3c_rx_data[4]);
	imu->gyro_z = (int16_t)((i3c_rx_data[7] << 8) | i3c_rx_data[6]);

	return ret;
}

/**
 * get the BMI323 accel data
 */
int32_t getBMI323Accel(xyz_accel_gyro_imu_s *imu)
{
	int32_t ret = ARM_DRIVER_ERROR;

	i3c_tx_data[0] = BMI323_ACC_X_REG;
	ret = i2c_over_i3cTransfer(BMI323_ADDR, i3c_tx_data, 1U, i3c_rx_data, 8U);

	imu->acc_x = (int16_t)((i3c_rx_data[3] << 8) | i3c_rx_data[2]);
	imu->acc_y = (int16_t)((i3c_rx_data[5] << 8) | i3c_rx_data[4]);
	imu->acc_z = (int16_t)((i3c_rx_data[7] << 8) | i3c_rx_data[6]);

	return ret;
}

/**
 * get the BMI323 temperature
 */
int32_t getBMI323Temp(xyz_accel_gyro_imu_s *imu)
{
	int32_t ret = ARM_DRIVER_ERROR;

	i3c_tx_data[0] = BMI323_TEMP_DATA_REG;
	ret = i2c_over_i3cTransfer(BMI323_ADDR, i3c_tx_data, 1U, i3c_rx_data, 4U);

	imu->temp = (int16_t)((i3c_rx_data[3] << 8) | i3c_rx_data[2]);
	imu->temp_float = (float)imu->temp / 512.0f + 23.0f;

	return ret;
}

/**
 * get all of the BMI323 sensor data
 */
int32_t getBMI323AllSensors(xyz_accel_gyro_imu_s *get)
{
	int32_t ret = ARM_DRIVER_ERROR;

	i3c_tx_data[0] = BMI323_ACC_X_REG;
	ret = i2c_over_i3cTransfer(BMI323_ADDR, i3c_tx_data, 1U, i3c_rx_data, 16U);

	get->acc_x   = (uint16_t)(i3c_rx_data[ 2] | i3c_rx_data[ 3] << 8);
	get->acc_y   = (uint16_t)(i3c_rx_data[ 4] | i3c_rx_data[ 5] << 8);
	get->acc_z   = (uint16_t)(i3c_rx_data[ 6] | i3c_rx_data[ 7] << 8);
	get->gyro_x  = (uint16_t)(i3c_rx_data[ 8] | i3c_rx_data[ 9] << 8);
	get->gyro_y  = (uint16_t)(i3c_rx_data[10] | i3c_rx_data[11] << 8);
	get->gyro_z  = (uint16_t)(i3c_rx_data[12] | i3c_rx_data[13] << 8);
	get->temp    = (uint16_t)(i3c_rx_data[14] | i3c_rx_data[15] << 8);
	get->temp_float = (float)get->temp / 512.0f + 23.0f;

	//
	// in the ICM42670P data structure, swapping accel/gyro x <-> y data
	// AND making y = -y, values of the ICM42670P structure track the
	// BMI323 structure.
	//
	// also, in the BMI323 data structure, divide BMI323 accel data by
	// 2, the accelerometer values match (at least statically).
	//
	//get->acc_x   = get->acc_x / 2;
	//get->acc_y   = get->acc_y / 2;
	//get->acc_z   = get->acc_z / 2;

	return ret;
}

/**
 * get the BMI323 interrupt status
 */
int32_t getBMI323IntStat(bmi323_int_status_t *stat)
{
	int32_t ret = ARM_DRIVER_ERROR;

	i3c_tx_data[0] = BMI323_ERROR_REG;
	ret = i2c_over_i3cTransfer(BMI323_ADDR, i3c_tx_data, 1U, i3c_rx_data, 6U);

	stat->ERROR_REG  	  = (i3c_rx_data[3] << 8) | i3c_rx_data[2];
	stat->STATUS_REG 	  = (i3c_rx_data[5] << 8) | i3c_rx_data[4];

	i3c_tx_data[0] = BMI323_SAT_FLAGS;
	ret |= i2c_over_i3cTransfer(BMI323_ADDR, i3c_tx_data, 1U, i3c_rx_data, 10U);

	stat->SAT_FLAGS       = (i3c_rx_data[3] << 8) | i3c_rx_data[2];
	stat->INT_STATUS_INT1 = (i3c_rx_data[5] << 8) | i3c_rx_data[4];
	stat->INT_STATUS_INT2 = (i3c_rx_data[7] << 8) | i3c_rx_data[6];
	stat->INT_STATUS_IBI  = (i3c_rx_data[9] << 8) | i3c_rx_data[8];

	i3c_tx_data[0] = BMI323_ALT_STATUS;
	ret |= i2c_over_i3cTransfer(BMI323_ADDR, i3c_tx_data, 1U, i3c_rx_data, 10U);

	stat->ALT_STATUS      = (i3c_rx_data[3] << 8) | i3c_rx_data[2];

	return ret;
}

/**
 * initialize the BMI323
 */
int32_t initBMI323(void)
{
	int32_t ret = ARM_DRIVER_ERROR;
	uint16_t bid = 0;

	ret = getRegBMI323(BMI323_CHIP_ID_REG, &bid);
	if(ret != ARM_DRIVER_OK)return ret;

	bid = bid & 0x00FF;

	if ((0x0041 != bid) && (0x0043 != bid)) {
		return ARM_DRIVER_ERROR;
	}

	//
	// acc_conf register
	// Normal mode
	//
	i3c_tx_data[0] = BMI323_ACC_CONF;
	i3c_tx_data[1] = 0x09;
	i3c_tx_data[2] = 0x43;
	ret = i2c_over_i3cTransfer(BMI323_ADDR, i3c_tx_data, 3U, i3c_rx_data, 0U);
	if(ret != ARM_DRIVER_OK) return ret;

	//
	// gyr_conf register
	// Normal mode
	//
	i3c_tx_data[0] = BMI323_GYR_CONF;
	i3c_tx_data[1] = 0x4B;
	i3c_tx_data[2] = 0x40;
	ret = i2c_over_i3cTransfer(BMI323_ADDR, i3c_tx_data, 3U, i3c_rx_data, 0U);
	if(ret != ARM_DRIVER_OK) return ret;

	//
	// err_reg register
	//
	i3c_tx_data[0] = BMI323_ERROR_REG;
	ret = i2c_over_i3cTransfer(BMI323_ADDR, i3c_tx_data, 1U, i3c_rx_data, 4U);
	if(ret != ARM_DRIVER_OK) return ret;

	if((0 != i3c_rx_data[1]) || (0 != i3c_rx_data[0])) return -300;

	//
	// io_int_ctrl register
	// int 1 and int 2
	//
	i3c_tx_data[0] = BMI323_IO_INT_CTRL;
	i3c_tx_data[1] = 0x06;
	i3c_tx_data[2] = 0x06;
	ret = i2c_over_i3cTransfer(BMI323_ADDR, i3c_tx_data, 3U, i3c_rx_data, 0U);
	if(ret != ARM_DRIVER_OK) return ret;

	//
	// int_map_1 register
	//
	i3c_tx_data[0] = BMI323_INT_MAP1;
	i3c_tx_data[1] = 0x21;
	i3c_tx_data[2] = 0x00;
	ret = i2c_over_i3cTransfer(BMI323_ADDR, i3c_tx_data, 3U, i3c_rx_data, 0U);
	if(ret != ARM_DRIVER_OK) return ret;

	//
	// int_map_2 register
	// Normal mode
	//
	i3c_tx_data[0] = BMI323_INT_MAP2;
	i3c_tx_data[1] = 0x00;
	i3c_tx_data[2] = 0x09;
	ret = i2c_over_i3cTransfer(BMI323_ADDR, i3c_tx_data, 3U, i3c_rx_data, 0U);
	if(ret != ARM_DRIVER_OK) return ret;

	i2c_bmi323_ok = true;

	return ARM_DRIVER_OK;
}

/**
 * temporary, get 4 registers in sequence
 */
int32_t getICM42670PRegs(uint8_t reg, uint16_t qty)
{
	uint8_t rx_buff[16];
	int32_t ret = ARM_DRIVER_ERROR;

	for(int i = 0; i < sizeof(rx_buff); i++) rx_buff[i] = 0x69;

	i3c_tx_data[0] = reg;
	ret = i2c_over_i3cTransfer(ICM42670_ADDR, i3c_tx_data, 1U, rx_buff, qty);
	return ret;
}

/**
 * get a single 16 bit register.  Always returns 2 bytes
 */
int32_t getRegICM42670(uint8_t reg, uint16_t * aData)
{
	int32_t ret = ARM_DRIVER_ERROR;

	i3c_tx_data[0] = reg;
	ret = i2c_over_i3cTransfer(ICM42670_ADDR, i3c_tx_data, 1U, i3c_rx_data, 2U);

	*aData = (i3c_rx_data[0] << 8U) | i3c_rx_data[1];

	return ret;
}


/**
 * get a single 8 bit register from MREG1 register bank area.
 */
int32_t getM1RegICM42670(uint8_t reg, uint8_t * m1Data)
{
	int32_t ret = ARM_DRIVER_ERROR;
	uint16_t mclk = 0;

	//
	// first, check MCLKRDY status
	//
	ret = getRegICM42670(ICM42670P_MCLK_RDY, &mclk);

	// is mclk ready?
	if(!(0x0004 & mclk)) ret -= 3;
	if(ret != ARM_DRIVER_OK) return ret;

	//
	// seutp up pointer for access
	//
	i3c_tx_data[0] = ICM42670P_BLK_SEL_R;
	i3c_tx_data[1] = 0x00; // MREG1
	i3c_tx_data[2] = reg;
	ret = i2c_over_i3cTransfer(ICM42670_ADDR, i3c_tx_data, 3U, i3c_rx_data, 0U);

	// wait
	delay_10us(10);

	//
	// now get data
	//
	i3c_tx_data[0] = ICM42670P_M_R;
	ret |= i2c_over_i3cTransfer(ICM42670_ADDR, i3c_tx_data, 1U, i3c_rx_data, 1U);

	*m1Data = i3c_rx_data[0];

	// wait
	delay_10us(10);

	return ret;
}

/**
 * send a single 8 bit register to MREG1 register bank area.
 */
int32_t putM1RegICM42670(uint8_t reg, uint8_t * m1Data)
{
	int32_t ret = ARM_DRIVER_ERROR;
	uint16_t mclk = 0;

	//
	// first, check MCLKRDY status
	//
	ret = getRegICM42670(ICM42670P_MCLK_RDY, &mclk);

	// is mclk ready?
	if(!(0x0004 & mclk)) ret -= 3;
	if(ret != ARM_DRIVER_OK) return ret;

	//
	// setup up pointer for access
	//
	i3c_tx_data[0] = ICM42670P_BLK_SEL_W;
	i3c_tx_data[1] = 0x00; // MREG1
	i3c_tx_data[2] = reg;
	i3c_tx_data[3] = * m1Data;
	ret = i2c_over_i3cTransfer(ICM42670_ADDR, i3c_tx_data, 4U, i3c_rx_data, 0U);

	// wait
	delay_10us(10);

	return ret;
}

/**
 * get the ICM42670P gyro data
 */
int32_t getICM42670PGyro(xyz_accel_gyro_imu_s *imu)
{
	int32_t ret = ARM_DRIVER_ERROR;

	i3c_tx_data[0] = ICM42670P_GYRO_DATA_X1;
	ret = i2c_over_i3cTransfer(ICM42670_ADDR, i3c_tx_data, 1U, i3c_rx_data, 6U);

	imu->gyro_x = (int16_t)((i3c_rx_data[0] << 8) | i3c_rx_data[1]);
	imu->gyro_y = (int16_t)((i3c_rx_data[2] << 8) | i3c_rx_data[3]);
	imu->gyro_z = (int16_t)((i3c_rx_data[4] << 8) | i3c_rx_data[5]);

	return ret;
}

/**
 * get the ICM42670P accel data
 */
int32_t getICM42670PAccel(xyz_accel_gyro_imu_s *imu)
{
	int32_t ret = ARM_DRIVER_ERROR;

	i3c_tx_data[0] = ICM42670P_ACCEL_DATA_X1;
	ret = i2c_over_i3cTransfer(ICM42670_ADDR, i3c_tx_data, 1U, i3c_rx_data, 6U);

	imu->acc_x = (int16_t)((i3c_rx_data[0] << 8) | i3c_rx_data[1]);
	imu->acc_y = (int16_t)((i3c_rx_data[2] << 8) | i3c_rx_data[3]);
	imu->acc_z = (int16_t)((i3c_rx_data[4] << 8) | i3c_rx_data[5]);

	return ret;
}

/**
 * get the ICM42670P temperature
 */
int32_t getICM42670PTemp(xyz_accel_gyro_imu_s *imu)
{
	int32_t ret = ARM_DRIVER_ERROR;

	i3c_tx_data[0] = ICM42670P_TEMP_DATA1;
	ret = i2c_over_i3cTransfer(ICM42670_ADDR, i3c_tx_data, 1U, i3c_rx_data, 2U);

	imu->temp = (int16_t)((i3c_rx_data[0] << 8) | i3c_rx_data[1]);
	imu->temp_float = (float)imu->temp / 128.0f + 25.0f;

	return ret;
}

/**
 * get all of the ICM42670P sensor data
 * NOTE: Some of the data is manipulated to the accel/gryo data matches the BMI323
 * Also NOTE: BMI323 values 2x that of ICM42670P
 */
int32_t getICM42670PAllSensors(xyz_accel_gyro_imu_s *imu)
{
	int32_t ret = ARM_DRIVER_ERROR;

	i3c_tx_data[0] = ICM42670P_TEMP_DATA1;
	ret = i2c_over_i3cTransfer(ICM42670_ADDR, i3c_tx_data, 1U, i3c_rx_data, 14U);

	//
	// in the ICM42670P data structure, swapping accel/gyro x <-> y data
	// AND making y = -y, values of the ICM42670P structure track the
	// BMI323 structure.
	//
	// also, in the BMI323 data structure, divide BMI323 accel data by
	// 2, the accelerometer values match (at least statically).
	//
#ifdef ICM_VAL_SAME_AS_BMI
	imu->acc_x   = (int16_t)+((i3c_rx_data[ 4] << 8) | i3c_rx_data[ 5]); // swapped x-y so data same as BMI323
	imu->acc_y   = (int16_t)-((i3c_rx_data[ 2] << 8) | i3c_rx_data[ 3]);
	imu->gyro_x  = (int16_t)+((i3c_rx_data[10] << 8) | i3c_rx_data[11]); // swapped x-y so data same as BMI323
	imu->gyro_y  = (int16_t)-((i3c_rx_data[ 8] << 8) | i3c_rx_data[ 9]);
#else
	imu->acc_x   = (int16_t)((i3c_rx_data[ 4] << 8) | i3c_rx_data[ 3]);
	imu->acc_y   = (int16_t)((i3c_rx_data[ 2] << 8) | i3c_rx_data[ 5]);
	imu->gyro_x  = (int16_t)((i3c_rx_data[ 8] << 8) | i3c_rx_data[ 9]);
	imu->gyro_y  = (int16_t)((i3c_rx_data[10] << 8) | i3c_rx_data[11]);
#endif
	imu->acc_z   = (int16_t)((i3c_rx_data[ 6] << 8) | i3c_rx_data[ 7]);
	imu->gyro_z  = (int16_t)((i3c_rx_data[12] << 8) | i3c_rx_data[13]);
	imu->temp    = (int16_t)((i3c_rx_data[ 0] << 8) | i3c_rx_data[ 1]);
	imu->temp_float = (float)imu->temp / 128.0f + 25.0f;

	return ret;
}

/**
 * get the ICM42670P interrupt status
 */
int32_t getICM42670PIntStat(icm42670p_int_status_t *stat)
{
	int32_t ret = ARM_DRIVER_ERROR;

	i3c_tx_data[0] = ICM42670P_INT_STATUS_DRDY;
	ret = i2c_over_i3cTransfer(ICM42670_ADDR, i3c_tx_data, 1U, i3c_rx_data, 4U);

	stat->INT_STATUS_DRDY = i3c_rx_data[0];
	stat->INT_STATUS  = i3c_rx_data[1];
	stat->INT_STATUS2 = i3c_rx_data[2];
	stat->INT_STATUS3 = i3c_rx_data[3];

	return ret;
}

/**
 * initialize the ICM42670P
 */
int32_t initICM42670P(void)
{
	int32_t ret = ARM_DRIVER_ERROR;
	uint16_t iwho = 0;
	uint8_t mreg1 = 0;
	icm42670p_int_status_t icm_s;

	ret = getRegICM42670(ICM42670P_WHO_AM_I, &iwho);
	if (ret != ARM_DRIVER_OK) return ret;
	iwho = iwho >> 8;

	if(0x0067 != iwho) {
		return ARM_DRIVER_ERROR;
	}

	//
	// power management register
	// gyro0 and accel0 config registers
	//
	i3c_tx_data[0] = ICM42670P_PWR_MGMT0;
	i3c_tx_data[1] = GYRO_MODE_low_noise | ACCR_MODE_Low_noise;
	i3c_tx_data[2] = GYRO_LN_MODE | (0x3 << 5); // gyro0 config -> 250 dps
	i3c_tx_data[3] = ACCEL_LN_MODE | (0x3 << 5); // accel0 config -> 2G
	ret = i2c_over_i3cTransfer(ICM42670_ADDR, i3c_tx_data, 4U, i3c_rx_data, 0U);
	if(ret != ARM_DRIVER_OK) return ret;

	//
	// clear pending interrupts
	//
	ret = getICM42670PIntStat(&icm_s);
	if(ret != ARM_DRIVER_OK) return ret;

	//
	// interrupt 1 source register
	//
	i3c_tx_data[0] = ICM42670P_INT_SOURCE0;
	i3c_tx_data[1] = 0x08; // DRDY bit set
	ret = i2c_over_i3cTransfer(ICM42670_ADDR, i3c_tx_data, 2U, i3c_rx_data, 0U);
	if(ret != ARM_DRIVER_OK) return ret;

	//
	// clear all pending interrupts
	//
	ret = getICM42670PIntStat(&icm_s);
	if(ret != ARM_DRIVER_OK) return ret;

	return ret;
}

/**
 * test the IMUs
 * - first time through, every routine above gets tested
 * - first time through, delays are added so IMUs can start up before reading data
 * - after the first pass, only the device ID and getAllSensor tests are run
 * - data results are stored in bmi & icm data structures
 * - add JTAG breakpoint to "return ret;" to see the data
 */
int32_t IMUtest(void)
{
	static bool firsttime = true;
	bool icm42670pIrqFlag = true;
	bool bmi323IrqFlag = true;
	bool i2c_icm42670p_ok = true;
	int32_t ret = ARM_DRIVER_ERROR;
	uint16_t bid = 0;
	uint16_t iwho = 0;
	uint16_t err = 0;
	uint16_t isstat = 0;

	uint32_t i_irq = 0;
	uint32_t b_irq = 0;

	xyz_accel_gyro_imu_s bmi;
	xyz_accel_gyro_imu_s icm;

	icm42670p_int_status_t icm_s;
	bmi323_int_status_t bmi_s;

	//
	// test the bme323 routines
	//
	if(i2c_bmi323_ok)
	{
		ret = getRegBMI323(BMI323_CHIP_ID_REG, &bid);
		if(ret != ARM_DRIVER_OK) err |= 0x01;
		bid = bid & 0x00FF;
	}

	if((0x0041 == bid) || (0x0043 == bid))
	{
		//
		// do this only once: initialize, then get accel, gyro
		// and temperature data in separate transactions
		//
		if(firsttime)
		{
			ret = initBMI323();
			if(ret != ARM_DRIVER_OK) err |= 0x02;
			delay_10us(10000); // wait 100mS for IMU to be ready

			ret = getRegBMI323(BMI323_STATUS_REG, &isstat);
			if(ret != ARM_DRIVER_OK) err |= 0x04;

			ret = getBMI323Temp(&bmi);
			if(ret != ARM_DRIVER_OK) err |= 0x08;

			ret = getBMI323Accel(&bmi);
			if(ret != ARM_DRIVER_OK) err |= 0x10;

			ret = getBMI323Gyro(&bmi);
			if(ret != ARM_DRIVER_OK) err |= 0x20;
		}

		//
		// did an BMI323 interrupt occur?
		//
		if(bmi323IrqFlag)
		{
			bmi323IrqFlag = false;
			ret = getBMI323IntStat(&bmi_s);
			if(ret != ARM_DRIVER_OK) err |= 0x80;
			// reEnableBMI323irq();  // interrupts work on the BMI323
		}

		//
		// get accel, gyro and temperature data in 1 I2C transaction
		//
		ret = getBMI323AllSensors(&bmi);
		if(ret != ARM_DRIVER_OK) err |= 0x40;
	} else
	{
		// printf(" - BMI323 wrong ID 0x%02x\r\n", (uint8_t)bid);
	}

	//
	// -------------------------
	//
	// test the icm42670p routines
	//
	if(i2c_icm42670p_ok)
	{
		ret = getRegICM42670(ICM42670P_WHO_AM_I, &iwho);
		if(ret != ARM_DRIVER_OK) err |= 0x100;
		iwho = iwho >> 8;
	}

	if(0x0067 == iwho)
	{
		//
		// do this only once: initialize, then get accel, gyro
		// and temperature data in separate transactions
		//
		if(firsttime)
		{
			firsttime = false;
			ret = initICM42670P();
			if(ret != ARM_DRIVER_OK) err |= 0x200;

			delay_10us(10000); // wait 100mS for IMU to be ready

			ret = getICM42670PTemp(&icm);
			if(ret != ARM_DRIVER_OK) err |= 0x400;

			ret = getICM42670PGyro(&icm);
			if(ret != ARM_DRIVER_OK) err |= 0x800;

			ret = getICM42670PAccel(&icm);
			if(ret != ARM_DRIVER_OK) err |= 0x1000;
		}

		//
		// did an ICM42670P interrupt occur?
		//
		if(icm42670pIrqFlag)
		{
			icm42670pIrqFlag = false;
			ret = getICM42670PIntStat(&icm_s);
			if(ret != ARM_DRIVER_OK) err |= 0x4000;
			// reEnableICM42670Pirq(); // interrupts work on the ICM42670P
		}

		//
		// get accel, gyro and temperature data in 1 I2C transaction
		//
		ret = getICM42670PAllSensors(&icm);
		if(ret != ARM_DRIVER_OK) err |= 0x2000;

		//
		// interrupt counts
		//
		i_irq = icm42670pCnt;
		b_irq = bmi323Cnt;

		//
		// for testing only
		//
		// getICM42670PRegs(ICM42670P_INT_SOURCE0, 4U);
		// getICM42670PRegs(ICM42670P_PWR_MGMT0, 16U);


	} else
	{
		// printf(" - ICM42670P wrong ID 0x%02x\r\n", (uint8_t)iwho);
	}

	return ret;
}
