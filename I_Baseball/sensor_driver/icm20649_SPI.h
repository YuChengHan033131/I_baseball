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
#include <ti/drivers/SPI.h>

/* -----------------------------------------------------------------------------
 *                                          Constants
 * -----------------------------------------------------------------------------
 */
#define ICM_Error               (-1)
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

//ACCEL_FCHOICE = 0
#define ICM_ACC4K              0x06
#define ICM_GYRO1K_BW57        0x37
#define ICM_ACC1K_BW57         0x37

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
bool SensorICM20649_accSetSample(uint8_t newRange);
uint8_t SensorICM20649_accReadRange(void);
bool SensorICM20649_accRead(uint8_t *data);
float SensorICM20649_accConvert(int16_t rawValue);

bool sensorICM20649_gyroSetBW(uint8_t newRange);
bool sensorICM20649_gyroSetSMPLRT_DIV(uint8_t newRange);
bool SensorICM20649_gyroSetRange(uint8_t range);
bool SensorICM20649_gyroSetSample(uint8_t newRange);
uint8_t SensorICM20649_gyroReadRange(void);
bool SensorICM20649_gyroRead(uint8_t *data);
float SensorICM20649_gyroConvert(int16_t rawValue);
void checkRegset(void);
extern unsigned char data_acc[7];
extern unsigned char data_gyro[7];
/*******************************************************************************
*/
enum {
    ICM_Success = 0,

};



#ifdef __cplusplus
};
#endif

#endif
