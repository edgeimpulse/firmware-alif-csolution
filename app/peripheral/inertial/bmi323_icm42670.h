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
 *  Created on: Feb 2, 2024
 *      Author: KevinBraun
 */

#ifndef DRIVERS_INCLUDE_IMUS_H_
#define DRIVERS_INCLUDE_IMUS_H_

/* Includes ---------------------------------------------------------------- */
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

/* Device addesses --------------------------------------------------------- */
#define		ICM42670_ADDR						0x68
#define		BMI323_ADDR							0x69

//
//-----------------------------------------------------------------------------
// IMU data structure for both the BMI313 and ICM42670P
//

typedef struct {
	int16_t acc_x;
	int16_t acc_y;
	int16_t acc_z;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	int16_t temp;
	float temp_float;
} xyz_accel_gyro_imu_s;

//
//-----------------------------------------------------------------------------
// status register data structure the BMI323
//

typedef struct {
	uint16_t ERROR_REG;
	uint16_t STATUS_REG;

	uint16_t SAT_FLAGS;
	uint16_t INT_STATUS_INT1;
	uint16_t INT_STATUS_INT2;
	uint16_t INT_STATUS_IBI;

	uint16_t ALT_STATUS;
} bmi323_int_status_t;

//
//-----------------------------------------------------------------------------
// BMI313 register set
//

typedef enum {
	BMI323_CHIP_ID_REG = 0x00,
	BMI323_ERROR_REG,
	BMI323_STATUS_REG,

	BMI323_ACC_X_REG,
	BMI323_ACC_Y_REG,
	BMI323_ACC_Z_REG,

	BMI323_GYR_X_REG,
	BMI323_GYR_Y_REG,
	BMI323_GYR_Z_REG,

	BMI323_TEMP_DATA_REG,
	BMI323_TIME_0_REG,
	BMI323_TIME_1_REG,
	BMI323_SAT_FLAGS,
	BMI323_INT_STATUS_INT1,
	BMI323_INT_STATUS_INT2,
	BMI323_INT_STATUS_IBI,
	BMI323_FEATURE_IO0,
	BMI323_FEATURE_IO1,
	BMI323_FEATURE_IO2,
	BMI323_FEATURE_IO3,
	BMI323_FEATURE_IO_STATUS,
	BMI323_FIFO_FILL_LEVEL,
	BMI323_FIFO_DATA,

	BMI323_ACC_CONF = 0x20,
	BMI323_GYR_CONF,

	BMI323_ACC_CONF_ALT = 0x28,
	BMI323_GYR_CONF_ALT,
	BMI323_ALT_CONF,
	BMI323_ALT_STATUS,
	BMI323_FIFO_WATERMARK = 0x35,
	BMI323_FIFO_CONF,
	BMI323_FIFO_CTRL,
	BMI323_IO_INT_CTRL,

	BMI323_INT_CONF,
	BMI323_INT_MAP1,
	BMI323_INT_MAP2,

	BMI323_FEATURE_CTRL = 0x40,
	BMI323_FEATURE_DATA_ADDR,
	BMI323_FEATURE_DATA_TX,

	BMI323_CMD_REG = 0x7E,
} BMI323_REG_MAP_t;

//
//-----------------------------------------------------------------------------
// ICM42670P register setup options
//

#define 	ICM_46_PWR_MGMT0_Pos					2
#define 	ICM_46_PWR_MGMT0_Mask					0x0C
#define 	ICM_46_PWR_MGMT0_GYRO_Value(Value)		(ICM_46_PWR_MGMT0_Mask & ((Value ) << ICM_46_PWR_MGMT0_Pos))
#define 	GYRO_MODE_Standby						ICM_46_PWR_MGMT0_GYRO_Value(1)
#define 	GYRO_MODE_low_noise						ICM_46_PWR_MGMT0_GYRO_Value(3)

/* Register Masking to Enable Accelerometer */
#define 	ICM_46_PWR_MGMT0_ACCRPos				0
#define 	ICM_46_PWR_MGMT0_ACCRMask				0x03
#define 	ICM_46_PWR_MGMT0_ACCR_Value(Value)		(ICM_46_PWR_MGMT0_ACCRMask & ((Value ) << ICM_46_PWR_MGMT0_ACCRPos))
#define 	ACCR_MODE_Standby						ICM_46_PWR_MGMT0_ACCR_Value(1)
#define 	ACCR_MODE_Low_noise						ICM_46_PWR_MGMT0_ACCR_Value(3)

/* Register Masking to fix Threshold value of Gyroscope */
#define 	ICM_GYRO_ODR_Pos					1
#define 	ICM_GYRO_ODR_Mask					0x06
#define 	ICM_46_GYRO_ODR_Value(Value)		(ICM_GYRO_ODR_Mask & ((Value ) << ICM_GYRO_ODR_Pos))
#define 	GYRO_LN_MODE						ICM_46_GYRO_ODR_Value(6)

/* Register Masking to fix Threshold value of Accelerometer */
#define 	ICM_ACCEL_ODR_Pos					1
#define 	ICM_ACCEL_ODR_Mask					0x06
#define 	ICM_46_ACCR_ODR_Value(Value)		(ICM_ACCEL_ODR_Mask & ((Value ) << ICM_ACCEL_ODR_Pos))
#define 	ACCEL_LN_MODE						ICM_46_ACCR_ODR_Value(6)

/* Interrupts for TILT Detection */
#define 	ICM_INT_SOURCE6						0x2F
#define 	ICM_INT_SOURCE7						0x30
#define 	ICM_INT_STATUS3						0x3C

//
//-----------------------------------------------------------------------------
// ICM42670P bank 0 register set
//

typedef enum {
	ICM42670P_MCLK_RDY = 0x00,
	ICM42670P_DEVICE_CONFIG,
	ICM42670P_SIGNAL_PATH_RESET,
	ICM42670P_DRIVE_CONFIG1,
	ICM42670P_DRIVE_CONFIG2,
	ICM42670P_DRIVE_CONFIG3,
	ICM42670P_INT_CONFIG,

	ICM42670P_TEMP_DATA1 = 0x09,
	ICM42670P_TEMP_DATA0,
	ICM42670P_ACCEL_DATA_X1,
	ICM42670P_ACCEL_DATA_X0,
	ICM42670P_ACCEL_DATA_Y1,
	ICM42670P_ACCEL_DATA_Y0,
	ICM42670P_ACCEL_DATA_Z1,
	ICM42670P_ACCEL_DATA_Z0,
	ICM42670P_GYRO_DATA_X1,
	ICM42670P_GYRO_DATA_X0,
	ICM42670P_GYRO_DATA_Y1,
	ICM42670P_GYRO_DATA_Y0,
	ICM42670P_GYRO_DATA_Z1,
	ICM42670P_GYRO_DATA_Z0,
	ICM42670P_TMST_FSYNCH,
	ICM42670P_TMST_FSYNCL,

	ICM42670P_APEX_DATA4 = 0x1D,
	ICM42670P_APEX_DATA5,
	ICM42670P_PWR_MGMT0,
	ICM42670P_GYRO_CONFIG0,
	ICM42670P_ACCEL_CONFIG0,
	ICM42670P_TEMP_CONFIG0,
	ICM42670P_GYRO_CONFIG1,
	ICM42670P_ACCEL_CONFIG1,
	ICM42670P_APEX_CONFIG0,
	ICM42670P_APEX_CONFIG1,
	ICM42670P_WOM_CONFIG,
	ICM42670P_FIFO_CONFIG1,
	ICM42670P_FIFO_CONFIG2,
	ICM42670P_FIFO_CONFIG3,
	ICM42670P_INT_SOURCE0,
	ICM42670P_INT_SOURCE1,
	ICM42670P_INT_SOURCE3,
	ICM42670P_INT_SOURCE4,
	ICM42670P_FIFO_LOST_PKT0,
	ICM42670P_FIFO_LOST_PKT1,
	ICM42670P_APEX_DATA0,
	ICM42670P_APEX_DATA1,
	ICM42670P_APEX_DATA2,
	ICM42670P_APEX_DATA3,
	ICM42670P_INTF_CONFIG0,
	ICM42670P_INTF_CONFIG1,

	ICM42670P_INT_STATUS_DRDY = 0x39,
	ICM42670P_INT_STATUS,
	ICM42670P_INT_STATUS2,
	ICM42670P_INT_STATUS3,
	ICM42670P_FIFO_COUNTH,
	ICM42670P_FIFO_COUNTL,
	ICM42670P_FIFO_DATA,

	ICM42670P_WHO_AM_I = 0x75,

	ICM42670P_BLK_SEL_W = 0x79,
	ICM42670P_MADDR_W,
	ICM42670P_M_W,
	ICM42670P_BLK_SEL_R,
	ICM42670P_MADDR_R,
	ICM42670P_M_R,
} ICM42670P_RB0_REG_MAP_t;

//
//-----------------------------------------------------------------------------
// ICM42670P bank 0 register set
//

typedef enum {
	ICM42670P_M1_TMST_CONFIG1 = 0x00,
	ICM42670P_M1_FIFO_CONFIG5,
	ICM42670P_M1_FIFO_CONFIG6,
	ICM42670P_M1_FSYNC_CONFIG,
	ICM42670P_M1_INT_CONFIG0,
	ICM42670P_M1_INT_CONFIG1,
	ICM42670P_M1_SENSOR_CONFIG3,

	ICM42670P_M1_ST_CONFIG = 0x13,
	ICM42670P_M1_SELFTEST,

	ICM42670P_M1_INTF_CONFIG6 = 0x23,

	ICM42670P_M1_INTF_CONFIG10 = 0x25,

	ICM42670P_M1_INTF_CONFIG7 = 0x28,

	ICM42670P_M1_OTP_CONFIG = 0x2B,

	ICM42670P_M1_INT_SOURCE6 = 0x2F,
	ICM42670P_M1_INT_SOURCE7,
	ICM42670P_M1_INT_SOURCE8,
	ICM42670P_M1_INT_SOURCE9,
	ICM42670P_M1_INT_SOURCE10,

	ICM42670P_M1_APEX_CONFIG2 = 0x44,
	ICM42670P_M1_APEX_CONFIG3,
	ICM42670P_M1_APEX_CONFIG4,
	ICM42670P_M1_APEX_CONFIG5,
	ICM42670P_M1_APEX_CONFIG9,
	ICM42670P_M1_APEX_CONFIG10,
	ICM42670P_M1_APEX_CONFIG11,
	ICM42670P_M1_ACCEL_WOM_X_THR,
	ICM42670P_M1_ACCEL_WOM_Y_THR,
	ICM42670P_M1_ACCEL_WOM_Z_THR,
	ICM42670P_M1_OFFSET_USER0,
	ICM42670P_M1_OFFSET_USER1,
	ICM42670P_M1_OFFSET_USER2,
	ICM42670P_M1_OFFSET_USER3,
	ICM42670P_M1_OFFSET_USER4,
	ICM42670P_M1_OFFSET_USER5,
	ICM42670P_M1_OFFSET_USER6,
	ICM42670P_M1_OFFSET_USER7,
	ICM42670P_M1_OFFSET_USER8,

	ICM42670P_M1_ST_STATUS1 = 0x63,
	ICM42670P_M1_ST_STATUS2,

	ICM42670P_M1_FDR_CONFIG = 0x66,
	ICM42670P_M1_APEX_CONFIG12

} ICM42670P_MREG1_REG_MAP_t;


//
//-----------------------------------------------------------------------------
// status register data structure the ICM42670P
//

typedef struct {
	uint8_t INT_STATUS_DRDY;
	uint8_t INT_STATUS;
	uint8_t INT_STATUS2;
	uint8_t INT_STATUS3;
} icm42670p_int_status_t;


/* Function prototypes ----------------------------------------------------- */
int bmi323_icm42670_init(void);


//
//-----------------------------------------------------------------------------
// BMI323 routines
//

int32_t getRegBMI323(uint8_t reg, uint16_t *aData);
int32_t getBMI323Gyro(xyz_accel_gyro_imu_s *);
int32_t getBMI323Accel(xyz_accel_gyro_imu_s *);
int32_t getBMI323Temp(xyz_accel_gyro_imu_s *);
int32_t getBMI323IntStat(bmi323_int_status_t *);
int32_t getBMI323AllSensors(xyz_accel_gyro_imu_s *);
int32_t initBMI323(void);

//
//-----------------------------------------------------------------------------
// ICM42670P register set
//

int32_t getICM42670PGyro(xyz_accel_gyro_imu_s *);
int32_t getICM42670PAccel(xyz_accel_gyro_imu_s *);
int32_t getICM42670PTemp(xyz_accel_gyro_imu_s *);
int32_t ICM_IMU_all_Get(xyz_accel_gyro_imu_s *);
int32_t getRegICM42670(uint8_t reg, uint16_t * aData);
int32_t getICM42670PAllSensors(xyz_accel_gyro_imu_s *);
int32_t getICM42670PIntStat(icm42670p_int_status_t *stat);
int32_t initICM42670P(void);

int32_t IMUtest(void);


#ifdef __cplusplus
}	// extern "C"
#endif

#endif /* DRIVERS_INCLUDE_IMUS_H_ */
