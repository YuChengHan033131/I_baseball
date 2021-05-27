/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/** ============================================================================
 *  @file       SensorICM20649.c
 *
 *  @brief      Driver for the InvenSense ICM20649 Motion Processing Unit.
 *  ============================================================================
 */
#ifndef SENSOR_ICM20649_H
#define SENSOR_ICM20649_H

#ifdef __cplusplus
extern "C" {
#endif

/* -----------------------------------------------------------------------------
 *                                          Includes
 * -----------------------------------------------------------------------------
 */
#include "stdint.h"
#include "stdbool.h"

/* -----------------------------------------------------------------------------
 *                                          Constants
 * -----------------------------------------------------------------------------
 */
// Accelerometer ranges
#define ACC_RANGE_4G        0
#define ACC_RANGE_8G        1
#define ACC_RANGE_16G       2
#define ACC_RANGE_30G       3
#define ACC_RANGE_INVALID   0xFF

// gyroscope ranges
#define GYRO_RANGE_500DPS   0
#define GYRO_RANGE_1000DPS  1
#define GYRO_RANGE_2000DPS  2
#define GYRO_RANGE_4000DPS  3
#define GYRO_RANGE_INVALID  0xFF

//DLPF Bandwidth of Accelerometer
#define ACC_BW_246_0       0
#define ACC_BW_246_X       1
#define ACC_BW_111_4       2
#define ACC_BW_50_4        3
#define ACC_BW_23_9        4
#define ACC_BW_11_5        5
#define ACC_BW_5_7         6
#define ACC_BW_473_0       7

//DLPF Bandwidth of gyroscope
#define GYRO_BW_196_6       0
#define GYRO_BW_151_8       1
#define GYRO_BW_119_5       2
#define GYRO_BW_51_2        3
#define GYRO_BW_23_9        4
#define GYRO_BW_11_6        5
#define GYRO_BW_5_7         6
#define GYRO_BW_361_4       7

// Axis bitmaps
#define ICM_AX_GYR        0x07
#define ICM_AX_ACC        0x38
#define ICM_AX_ALL        0x7F

// Interrupt status bit
#define ICM_DATA_READY    0x01
#define ICM_MOVEMENT      0x40

/* ----------------------------------------------------------------------------
 *                                           Typedefs
 * -----------------------------------------------------------------------------
*/
typedef void (*SensorICM20649CallbackFn_t)(void);

/* -----------------------------------------------------------------------------
 *                                          Functions
 * -----------------------------------------------------------------------------
 */
bool SensorICM20649_init(void);
void SensorICM20649_registerCallback(SensorICM20649CallbackFn_t);
bool SensorICM20649_test(void);

void sensorICM20649_enable(uint8_t axes);
void SensorICM20649_enable_int(void);
bool SensorICM20649_enableWom(uint8_t threshold);
uint8_t SensorICM20649_irqStatus(void);

bool sensorICM20649_accSetBW(uint8_t newRange);
bool sensorICM20649_accSetSMPLRT_DIV(uint16_t newRange);
bool SensorICM20649_accSetRange(uint8_t range);
uint8_t SensorICM20649_accReadRange(void);
bool SensorICM20649_accRead(uint8_t *data);
float SensorICM20649_accConvert(int16_t rawValue);

bool sensorICM20649_gyroSetBW(uint8_t newRange);
bool sensorICM20649_gyroSetSMPLRT_DIV(uint8_t newRange);
bool SensorICM20649_gyroSetRange(uint8_t range);
uint8_t SensorICM20649_gyroReadRange(void);
bool SensorICM20649_gyroRead(uint8_t *data);
float SensorICM20649_gyroConvert(int16_t rawValue);

extern bool readReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t len);
extern bool writeReg(uint8_t reg_addr, uint8_t reg_data);

/*******************************************************************************
*/
/* -----------------------------------------------------------------------------
*  Constants and macros
* ------------------------------------------------------------------------------
*/
/*------------ ICM 20649 I2C address ------------*/
#define ICM20649_I2C_ADDR1            0x68 //AD0 is logic low
#define ICM20649_I2C_ADDR2            0x69 //AD0 is logic high

/*-------User Bank 0 Registers (ICM20649) -------*/
#define WHO_AM_I                      0x00 // R
#define USER_CTRL                     0x03 // R/W
#define LP_CONFIG                     0x05 // R/W
#define PWR_MGMT_1                    0x06 // R/W
#define PWR_MGMT_2                    0x07 // R/W
#define INT_PIN_CFG                   0x0F // R/W
#define INT_ENABLE                    0x10 // R/W
#define INT_ENABLE1                   0x11 // R/W
#define INT_ENABLE2                   0x12 // R/W
#define INT_ENABLE3                   0x13 // R/W
#define I2C_MST_STATUS                0x17 // R/C
#define INT_STATUS                    0x19 // R/C
#define INT_STATUS_1                  0x1A // R/C
#define INT_STATUS_2                  0x1B // R/C
#define INT_STATUS_3                  0x1C // R/C
#define DELAY_TIMEH                   0x28 // R
#define DELAY_TIMEL                   0x28 // R
#define ACCEL_XOUT_H                  0x2D // R
#define ACCEL_XOUT_L                  0x2E // R
#define ACCEL_YOUT_H                  0x2F // R
#define ACCEL_YOUT_L                  0x30 // R
#define ACCEL_ZOUT_H                  0x31 // R
#define ACCEL_ZOUT_L                  0x32 // R
#define GYRO_XOUT_H                   0x33 // R
#define GYRO_XOUT_L                   0x34 // R
#define GYRO_YOUT_H                   0x35 // R
#define GYRO_YOUT_L                   0x36 // R
#define GYRO_ZOUT_H                   0x37 // R
#define GYRO_ZOUT_L                   0x38 // R
#define TEMP_OUT_H                    0x39 // R
#define TEMP_OUT_L                    0x3A // R
#define EXT_SLV_SENS_DATA_00          0x3B // R
#define EXT_SLV_SENS_DATA_01          0x3C // R
#define EXT_SLV_SENS_DATA_02          0x3D // R
#define EXT_SLV_SENS_DATA_03          0x3E // R
#define EXT_SLV_SENS_DATA_04          0x3F // R
#define EXT_SLV_SENS_DATA_05          0x40 // R
#define EXT_SLV_SENS_DATA_06          0x41 // R
#define EXT_SLV_SENS_DATA_07          0x42 // R
#define EXT_SLV_SENS_DATA_08          0x43 // R
#define EXT_SLV_SENS_DATA_09          0x44 // R
#define EXT_SLV_SENS_DATA_10          0x45 // R
#define EXT_SLV_SENS_DATA_11          0x46 // R
#define EXT_SLV_SENS_DATA_12          0x47 // R
#define EXT_SLV_SENS_DATA_13          0x48 // R
#define EXT_SLV_SENS_DATA_14          0x49 // R
#define EXT_SLV_SENS_DATA_15          0x4A // R
#define EXT_SLV_SENS_DATA_16          0x4B // R
#define EXT_SLV_SENS_DATA_17          0x4C // R
#define EXT_SLV_SENS_DATA_18          0x4D // R
#define EXT_SLV_SENS_DATA_19          0x4E // R
#define EXT_SLV_SENS_DATA_20          0x4F // R
#define EXT_SLV_SENS_DATA_21          0x50 // R
#define EXT_SLV_SENS_DATA_22          0x51 // R
#define EXT_SLV_SENS_DATA_23          0x52 // R
#define FIFO_EN_1                     0x66 // R/W
#define FIFO_EN_2                     0x67 // R/W
#define FIFO_RST                      0x68 // R/W
#define FIFO_MODE                     0x69 // R/W
#define FIFO_COUNT_H                  0x70 // R
#define FIFO_COUNT_L                  0x71 // R
#define FIFO_R_W                      0x72 // R/W
#define DATA_RDY_STATUS               0x74 // R/C
#define FIFO_CFG                      0x76 // R/W

/*-------User Bank 1 Registers (ICM20649) -------*/
#define SELF_TEST_X_GYRO              0x02 // R/W
#define SELF_TEST_Y_GYRO              0x03 // R/W
#define SELF_TEST_Z_GYRO              0x04 // R/W
#define SELF_TEST_X_ACCEL             0x0E // R/W
#define SELF_TEST_Z_ACCEL             0x0F // R/W
#define SELF_TEST_Y_ACCEL             0x10 // R/W
#define XA_OFFSET_H                   0x14 // R/W
#define XA_OFFSET_L                   0x15 // R/W
#define YA_OFFSET_H                   0x17 // R/W
#define YA_OFFSET_L                   0x18 // R/W
#define ZA_OFFSET_H                   0x1A // R/W
#define ZA_OFFSET_L                   0x1B // R/W
#define TIMEBASE_CORRECTION_PLL       0x28 // R/W

/*-------User Bank 2 Registers (ICM20649) -------*/
#define GYRO_SMPLRT_DIV               0x00 // R/W
#define GYRO_CONFIG_1                 0x01 // R/W
#define GYRO_CONFIG_2                 0x02 // R/W
#define XG_OFFS_USRH                  0x03 // R/W
#define XG_OFFS_USRL                  0x04 // R/W
#define YG_OFFS_USRH                  0x05 // R/W
#define YG_OFFS_USRL                  0x06 // R/W
#define ZG_OFFS_USRH                  0x07 // R/W
#define ZG_OFFS_USRL                  0x08 // R/W
#define ODR_ALIGN_EN                  0x09 // R/W
#define ACCEL_SMPLRT_DIV_1            0x10 // R/W
#define ACCEL_SMPLRT_DIV_2            0x11 // R/W
#define ACCEL_INTEL_CTRL              0x12 // R/W
#define ACCEL_WOM_THR                 0x13 // R/W
#define ACCEL_CONFIG                  0x14 // R/W
#define ACCEL_CONFIG_2                0x15 // R/W
#define FSYNC_CONFIG                  0x52 // R/W
#define TEMP_CONFIG                   0x53 // R/W
#define MOD_CTRL_USR                  0x54 // R/W

/*-------User Bank 3 Registers (ICM20649) -------*/
#define I2C_MST_ODR_CONFIG            0x00 // R/W
#define I2C_MST_CTRL                  0x01 // R/W
#define I2C_MST_DELAY_CTRL            0x02 // R/W
#define I2C_SLV0_ADDR                 0x03 // R/W
#define I2C_SLV0_REG                  0x04 // R/W
#define I2C_SLV0_CTRL                 0x05 // R/W
#define I2C_SLV0_DO                   0x06 // R/W
#define I2C_SLV1_ADDR                 0x07 // R/W
#define I2C_SLV1_REG                  0x08 // R/W
#define I2C_SLV1_CTRL                 0x09 // R/W
#define I2C_SLV1_DO                   0x0A // R/W
#define I2C_SLV2_ADDR                 0x0B // R/W
#define I2C_SLV2_REG                  0x0C // R/W
#define I2C_SLV2_CTRL                 0x0D // R/W
#define I2C_SLV2_DO                   0x0E // R/W
#define I2C_SLV3_ADDR                 0x0F // R/W
#define I2C_SLV3_REG                  0x10 // R/W
#define I2C_SLV3_CTRL                 0x11 // R/W
#define I2C_SLV3_DO                   0x12 // R/W
#define I2C_SLV4_ADDR                 0x13 // R/W
#define I2C_SLV4_REG                  0x14 // R/W
#define I2C_SLV4_CTRL                 0x15 // R/W
#define I2C_SLV4_DO                   0x16 // R/W
#define I2C_SLV4_DI                   0x17 // R/W

#define REG_BANK_SEL                  0x7F // R/W

//Bank Selection
#define BANK_0                        0x00
#define BANK_1                        0x10
#define BANK_2                        0x20
#define BANK_3                        0x30

// Masks is mpuConfig valiable
#define ACC_CONFIG_MASK               0x38
#define GYRO_CONFIG_MASK              0x07

// Values PWR_MGMT_1
#define ICM_SLEEP                     0x4F  // Sleep + stop all clocks
#define ICM_WAKE_UP                   0x09  // Disable temp. + intern osc

// Values PWR_MGMT_2
#define ALL_AXES                      0x3F
#define GYRO_AXES                     0x07
#define ACC_AXES                      0x38

// Data sizes
#define DATA_SIZE                     6

// Output data rates
#define INV_LPA_0_3125HZ              0
#define INV_LPA_0_625HZ               1
#define INV_LPA_1_25HZ                2
#define INV_LPA_2_5HZ                 3
#define INV_LPA_5HZ                   4
#define INV_LPA_10HZ                  5
#define INV_LPA_20HZ                  6
#define INV_LPA_40HZ                  7
#define INV_LPA_80HZ                  8
#define INV_LPA_160HZ                 9
#define INV_LPA_320HZ                 10
#define INV_LPA_640HZ                 11
#define INV_LPA_STOPPED               255

// Bit values
#define BIT_ANY_RD_CLR                0x10
#define BIT_RAW_RDY_EN                0x01
#define BIT_WOM_EN                    0x40
#define BIT_LPA_CYCLE                 0x20
#define BIT_STBY_XA                   0x20
#define BIT_STBY_YA                   0x10
#define BIT_STBY_ZA                   0x08
#define BIT_STBY_XG                   0x04
#define BIT_STBY_YG                   0x02
#define BIT_STBY_ZG                   0x01
#define BIT_STBY_XYZA                 (BIT_STBY_XA | BIT_STBY_YA | BIT_STBY_ZA)
#define BIT_STBY_XYZG                 (BIT_STBY_XG | BIT_STBY_YG | BIT_STBY_ZG)

// User control register
#define BIT_LATCH_EN                  0x20
#define BIT_ACTL                      0x80

// INT Pin / Bypass Enable Configuration
#define BIT_BYPASS_EN                 0x02
#define BIT_AUX_IF_EN                 0x20


// Mode
#define MAG_MODE_OFF                  0x00
#define MAG_MODE_SINGLE               0x01
#define MAG_MODE_CONT1                0x02
#define MAG_MODE_CONT2                0x06
#define MAG_MODE_FUSE                 0x0F

// Resolution
#define MFS_14BITS                    0     // 0.6 mG per LSB
#define MFS_16BITS                    1     // 0.15 mG per LSB

#ifdef __cplusplus
};
#endif

#endif
