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
#ifndef SENSOR_ICM20649_I2C_H
#define SENSOR_ICM20649_I2C_H

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
bool SensorICM20649_init_I2C(void);
void SensorICM20649_registerCallback_I2C(SensorICM20649CallbackFn_t);
bool SensorICM20649_test_I2C(void);

void sensorICM20649_enable_I2C(uint8_t axes);
void SensorICM20649_enable_int_I2C(void);
bool SensorICM20649_enableWom_I2C(uint8_t threshold);
uint8_t SensorICM20649_irqStatus_I2C(void);

bool sensorICM20649_accSetBW_I2C(uint8_t newRange);
bool sensorICM20649_accSetSMPLRT_DIV_I2C(uint16_t newRange);
bool SensorICM20649_accSetRange_I2C(uint8_t range);
uint8_t SensorICM20649_accReadRange_I2C(void);
bool SensorICM20649_accRead_I2C(uint8_t *data);
float SensorICM20649_accConvert_I2C(int16_t rawValue);

bool sensorICM20649_gyroSetBW_I2C(uint8_t newRange);
bool sensorICM20649_gyroSetSMPLRT_DIV_I2C(uint8_t newRange);
bool SensorICM20649_gyroSetRange_I2C(uint8_t range);
uint8_t SensorICM20649_gyroReadRange_I2C(void);
bool SensorICM20649_gyroRead_I2C(uint8_t *data);
float SensorICM20649_gyroConvert_I2C(int16_t rawValue);

bool readReg_I2C(uint8_t reg_addr, uint8_t *reg_data, uint8_t len);
bool writeReg_I2C(uint8_t reg_addr, uint8_t reg_data);

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
#define WHO_AM_I_I2C                      0x00 // R
#define USER_CTRL_I2C                     0x03 // R/W
#define LP_CONFIG_I2C                     0x05 // R/W
#define PWR_MGMT_1_I2C                    0x06 // R/W
#define PWR_MGMT_2_I2C                    0x07 // R/W
#define INT_PIN_CFG_I2C                   0x0F // R/W
#define INT_ENABLE_I2C                    0x10 // R/W
#define INT_ENABLE1_I2C                   0x11 // R/W
#define INT_ENABLE2_I2C                   0x12 // R/W
#define INT_ENABLE3_I2C                   0x13 // R/W
#define I2C_MST_STATUS_I2C                0x17 // R/C
#define INT_STATUS_I2C                    0x19 // R/C
#define INT_STATUS_1_I2C                  0x1A // R/C
#define INT_STATUS_2_I2C                  0x1B // R/C
#define INT_STATUS_3_I2C                  0x1C // R/C
#define DELAY_TIMEH_I2C                   0x28 // R
#define DELAY_TIMEL_I2C                   0x28 // R
#define ACCEL_XOUT_H_I2C                  0x2D // R
#define ACCEL_XOUT_L_I2C                  0x2E // R
#define ACCEL_YOUT_H_I2C                  0x2F // R
#define ACCEL_YOUT_L_I2C                  0x30 // R
#define ACCEL_ZOUT_H_I2C                 0x31 // R
#define ACCEL_ZOUT_L_I2C                  0x32 // R
#define GYRO_XOUT_H_I2C                   0x33 // R
#define GYRO_XOUT_L_I2C                   0x34 // R
#define GYRO_YOUT_H_I2C                   0x35 // R
#define GYRO_YOUT_L_I2C                   0x36 // R
#define GYRO_ZOUT_H_I2C                   0x37 // R
#define GYRO_ZOUT_L_I2C                   0x38 // R
#define TEMP_OUT_H_I2C                    0x39 // R
#define TEMP_OUT_L_I2C                    0x3A // R
#define EXT_SLV_SENS_DATA_00_I2C          0x3B // R
#define EXT_SLV_SENS_DATA_01_I2C          0x3C // R
#define EXT_SLV_SENS_DATA_02_I2C          0x3D // R
#define EXT_SLV_SENS_DATA_03_I2C          0x3E // R
#define EXT_SLV_SENS_DATA_04_I2C          0x3F // R
#define EXT_SLV_SENS_DATA_05_I2C          0x40 // R
#define EXT_SLV_SENS_DATA_06_I2C          0x41 // R
#define EXT_SLV_SENS_DATA_07_I2C          0x42 // R
#define EXT_SLV_SENS_DATA_08_I2C          0x43 // R
#define EXT_SLV_SENS_DATA_09_I2C          0x44 // R
#define EXT_SLV_SENS_DATA_10_I2C          0x45 // R
#define EXT_SLV_SENS_DATA_11_I2C          0x46 // R
#define EXT_SLV_SENS_DATA_12_I2C          0x47 // R
#define EXT_SLV_SENS_DATA_13_I2C          0x48 // R
#define EXT_SLV_SENS_DATA_14_I2C          0x49 // R
#define EXT_SLV_SENS_DATA_15_I2C          0x4A // R
#define EXT_SLV_SENS_DATA_16_I2C          0x4B // R
#define EXT_SLV_SENS_DATA_17_I2C          0x4C // R
#define EXT_SLV_SENS_DATA_18_I2C          0x4D // R
#define EXT_SLV_SENS_DATA_19_I2C          0x4E // R
#define EXT_SLV_SENS_DATA_20_I2C          0x4F // R
#define EXT_SLV_SENS_DATA_21_I2C          0x50 // R
#define EXT_SLV_SENS_DATA_22_I2C          0x51 // R
#define EXT_SLV_SENS_DATA_23_I2C          0x52 // R
#define FIFO_EN_1_I2C                     0x66 // R/W
#define FIFO_EN_2_I2C                     0x67 // R/W
#define FIFO_RST_I2C                      0x68 // R/W
#define FIFO_MODE_I2C                     0x69 // R/W
#define FIFO_COUNT_H_I2C                  0x70 // R
#define FIFO_COUNT_L_I2C                  0x71 // R
#define FIFO_R_W_I2C                      0x72 // R/W
#define DATA_RDY_STATUS_I2C               0x74 // R/C
#define FIFO_CFG_I2C                      0x76 // R/W

/*-------User Bank 1 Registers (ICM20649) -------*/
#define SELF_TEST_X_GYRO_I2C              0x02 // R/W
#define SELF_TEST_Y_GYRO_I2C              0x03 // R/W
#define SELF_TEST_Z_GYRO_I2C              0x04 // R/W
#define SELF_TEST_X_ACCEL_I2C             0x0E // R/W
#define SELF_TEST_Z_ACCEL_I2C             0x0F // R/W
#define SELF_TEST_Y_ACCEL_I2C             0x10 // R/W
#define XA_OFFSET_H_I2C                   0x14 // R/W
#define XA_OFFSET_L_I2C                   0x15 // R/W
#define YA_OFFSET_H_I2C                   0x17 // R/W
#define YA_OFFSET_L_I2C                  0x18 // R/W
#define ZA_OFFSET_H_I2C                   0x1A // R/W
#define ZA_OFFSET_L_I2C                   0x1B // R/W
#define TIMEBASE_CORRECTION_PLL_I2C       0x28 // R/W

/*-------User Bank 2 Registers (ICM20649) -------*/
#define GYRO_SMPLRT_DIV_I2C               0x00 // R/W
#define GYRO_CONFIG_1_I2C                 0x01 // R/W
#define GYRO_CONFIG_2_I2C                 0x02 // R/W
#define XG_OFFS_USRH_I2C                  0x03 // R/W
#define XG_OFFS_USRL_I2C                  0x04 // R/W
#define YG_OFFS_USRH_I2C                  0x05 // R/W
#define YG_OFFS_USRL_I2C                  0x06 // R/W
#define ZG_OFFS_USRH_I2C                  0x07 // R/W
#define ZG_OFFS_USRL_I2C                  0x08 // R/W
#define ODR_ALIGN_EN_I2C                  0x09 // R/W
#define ACCEL_SMPLRT_DIV_1_I2C            0x10 // R/W
#define ACCEL_SMPLRT_DIV_2_I2C            0x11 // R/W
#define ACCEL_INTEL_CTRL_I2C              0x12 // R/W
#define ACCEL_WOM_THR_I2C                 0x13 // R/W
#define ACCEL_CONFIG_I2C                  0x14 // R/W
#define ACCEL_CONFIG_2_I2C                0x15 // R/W
#define FSYNC_CONFIG_I2C                  0x52 // R/W
#define TEMP_CONFIG_I2C                   0x53 // R/W
#define MOD_CTRL_USR_I2C                  0x54 // R/W

/*-------User Bank 3 Registers (ICM20649) -------*/
#define I2C_MST_ODR_CONFIG_I2C            0x00 // R/W
#define I2C_MST_CTRL_I2C                  0x01 // R/W
#define I2C_MST_DELAY_CTRL_I2C            0x02 // R/W
#define I2C_SLV0_ADDR_I2C                 0x03 // R/W
#define I2C_SLV0_REG_I2C                  0x04 // R/W
#define I2C_SLV0_CTRL_I2C                 0x05 // R/W
#define I2C_SLV0_DO_I2C                   0x06 // R/W
#define I2C_SLV1_ADDR_I2C                 0x07 // R/W
#define I2C_SLV1_REG_I2C                  0x08 // R/W
#define I2C_SLV1_CTRL_I2C                 0x09 // R/W
#define I2C_SLV1_DO_I2C                   0x0A // R/W
#define I2C_SLV2_ADDR_I2C                 0x0B // R/W
#define I2C_SLV2_REG_I2C                  0x0C // R/W
#define I2C_SLV2_CTRL_I2C                 0x0D // R/W
#define I2C_SLV2_DO_I2C                   0x0E // R/W
#define I2C_SLV3_ADDR_I2C                 0x0F // R/W
#define I2C_SLV3_REG_I2C                  0x10 // R/W
#define I2C_SLV3_CTRL_I2C                 0x11 // R/W
#define I2C_SLV3_DO_I2C                   0x12 // R/W
#define I2C_SLV4_ADDR_I2C                 0x13 // R/W
#define I2C_SLV4_REG_I2C                  0x14 // R/W
#define I2C_SLV4_CTRL_I2C                 0x15 // R/W
#define I2C_SLV4_DO_I2C                   0x16 // R/W
#define I2C_SLV4_DI_I2C                   0x17 // R/W

#define REG_BANK_SEL_I2C                  0x7F // R/W

//Bank Selection
#define BANK_0_I2C                        0x00
#define BANK_1_I2C                        0x10
#define BANK_2_I2C                        0x20
#define BANK_3_I2C                        0x30

// Masks is mpuConfig valiable
#define ACC_CONFIG_MASK_I2C               0x38
#define GYRO_CONFIG_MASK_I2C              0x07

// Values PWR_MGMT_1
#define ICM_SLEEP_I2C                     0x4F  // Sleep + stop all clocks
#define ICM_WAKE_UP_I2C                   0x09  // Disable temp. + intern osc

// Values PWR_MGMT_2
#define ALL_AXES_I2C                      0x3F
#define GYRO_AXES_I2C                     0x07
#define ACC_AXES_I2C                      0x38

// Data sizes
#define DATA_SIZE_I2C                     6

// Output data rates
#define INV_LPA_0_3125HZ_I2C              0
#define INV_LPA_0_625HZ_I2C               1
#define INV_LPA_1_25HZ_I2C                2
#define INV_LPA_2_5HZ_I2C                 3
#define INV_LPA_5HZ_I2C                   4
#define INV_LPA_10HZ_I2C                  5
#define INV_LPA_20HZ_I2C                  6
#define INV_LPA_40HZ_I2C                  7
#define INV_LPA_80HZ_I2C                  8
#define INV_LPA_160HZ_I2C                 9
#define INV_LPA_320HZ_I2C                 10
#define INV_LPA_640HZ_I2C                 11
#define INV_LPA_STOPPED_I2C               255

// Bit values
#define BIT_ANY_RD_CLR_I2C                0x10
#define BIT_RAW_RDY_EN_I2C                0x01
#define BIT_WOM_EN_I2C                    0x40
#define BIT_LPA_CYCLE_I2C                 0x20
#define BIT_STBY_XA_I2C                   0x20
#define BIT_STBY_YA_I2C                   0x10
#define BIT_STBY_ZA_I2C                   0x08
#define BIT_STBY_XG_I2C                   0x04
#define BIT_STBY_YG_I2C                   0x02
#define BIT_STBY_ZG_I2C                   0x01
#define BIT_STBY_XYZA_I2C                 (BIT_STBY_XA | BIT_STBY_YA | BIT_STBY_ZA)
#define BIT_STBY_XYZG_I2C                 (BIT_STBY_XG | BIT_STBY_YG | BIT_STBY_ZG)

// User control register
#define BIT_LATCH_EN_I2C                  0x20
#define BIT_ACTL_I2C                      0x80

// INT Pin / Bypass Enable Configuration
#define BIT_BYPASS_EN_I2C                 0x02
#define BIT_AUX_IF_EN_I2C                 0x20


// Mode
#define MAG_MODE_OFF_I2C                  0x00
#define MAG_MODE_SINGLE_I2C               0x01
#define MAG_MODE_CONT1_I2C                0x02
#define MAG_MODE_CONT2_I2C                0x06
#define MAG_MODE_FUSE_I2C                 0x0F

// Resolution
#define MFS_14BITS_I2C                    0     // 0.6 mG per LSB
#define MFS_16BITS_I2C                    1     // 0.15 mG per LSB

#ifdef __cplusplus
};
#endif

#endif
