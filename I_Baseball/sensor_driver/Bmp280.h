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
 *  @file       SensorBmp280.h
 *
 *  @brief      Driver for the Bosch BMP280 Pressure Sensor.
 *
 *  ============================================================================
 */
#ifndef SENSOR_BMP280_H
#define SENSOR_BMP280_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "stdint.h"
#include "stdbool.h"

/*********************************************************************
 * CONSTANTS
 */
#define SENSOR_BMP280_DATASIZE           6

#define STANDBY_TIME_0_5ms               0
#define STANDBY_TIME_62_5ms              1
#define STANDBY_TIME_125ms               2
#define STANDBY_TIME_250ms               3
#define STANDBY_TIME_500ms               4
#define STANDBY_TIME_1000ms              5
#define STANDBY_TIME_2000ms              6
#define STANDBY_TIME_4000ms              7

#define FILTER_COE_FILTER_OFF            0
#define FILTER_COE_2                     1
#define FILTER_COE_4                     2
#define FILTER_COE_8                     3
#define FILTER_COE_16                    4

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * FUNCTIONS
 */
bool SensorBmp280_init(void);
void SensorBmp280_enable(bool enable);
void SensorBmp280_config(uint8_t t_sb, uint8_t filter);
bool SensorBmp280_read(uint8_t *pBuf);
void SensorBmp280_convert(uint8_t *raw, int32_t *temp, uint32_t *press);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_BMP280_H */
