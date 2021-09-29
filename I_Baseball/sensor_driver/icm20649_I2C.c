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
 *  @file       ICM20649.c
 *
 *  @brief      Driver for the InvenSense ICM20649 Motion Processing Unit.
 *  ============================================================================
 */

/* -----------------------------------------------------------------------------
*  Includes
* ------------------------------------------------------------------------------
*/
#include <ti/display/Display.h>
#include <sensor_driver/icm20649_I2C.h>
#include <ti/posix/ccs/unistd.h>
#include <ti/drivers/I2C.h>
#include "Board.h"

extern I2C_Handle i2cHandle;
extern Display_Handle displayOut;



/* -----------------------------------------------------------------------------
*  Local Functions
* ------------------------------------------------------------------------------
*/
static void sensorICM_Sleep_I2C(void);
static void sensorICM20649_WakeUp_I2C(void);
static void sensorICM20649_SelectAxes_I2C(void);


/* -----------------------------------------------------------------------------
*  Local Variables
* ------------------------------------------------------------------------------
*/
static uint8_t mpuConfig;
static uint8_t accBW;
static uint16_t accsmplrt_div;
static uint8_t accRange;
static uint8_t accRangeReg;
static uint8_t gyroBW;
static uint8_t gyrosmplrt_div;
static uint8_t gyroRange;
static uint8_t gyroRangeReg;
static uint8_t val;

/* -----------------------------------------------------------------------------
*  Public functions
* ------------------------------------------------------------------------------
*/

/*******************************************************************************
* @fn          sensorICM20649_init
*
* @brief       This function initializes the MPU abstraction layer.
*
* @return      True if success
*/
bool SensorICM20649_init_I2C(void)
{
    bool success, ret;

    accRange = ACC_RANGE_INVALID;
    gyroRange = GYRO_RANGE_INVALID;
    mpuConfig = 0;   // All axes off

    // Device reset
    val = 0x80;
    success = false;
    while (success != true) success = writeReg_I2C(REG_BANK_SEL_I2C, BANK_0_I2C);
    success = false;
    while (success != true) success = writeReg_I2C(PWR_MGMT_1_I2C, val);
    Display_print0(displayOut, 0, 0, "ICM20649 PWR_MGMT_1 success");

    ret = false;
    while(ret != true) ret = SensorICM20649_test_I2C();
    Display_print0(displayOut, 0, 0, "ICM20649 test success");

    // Power save
    sensorICM_Sleep_I2C();

    return ret;
}

/*******************************************************************************
 * @fn          sensorICM20649_test
 *
 * @brief       Run a sensor self-test
 *
 * @return      TRUE if passed, FALSE if failed
 */
bool SensorICM20649_test_I2C(void)
{
    bool success;

    // Make sure power is ramped up
    sleep(0.1);     //100ms

    // Check the WHO AM I register
    val = 0;
    success = false;
    while (success != true) success = writeReg_I2C(REG_BANK_SEL_I2C, BANK_0_I2C);
    success = false;
    while (success != true) success = readReg_I2C(WHO_AM_I_I2C, &val, 1);
    //Display_print1(displayOut, 0, 0, "WHO_AM_I : %d", val);

    if(val == 0xE1)
    {
        return true;
    }
    else
    {
        return false;
    }
}


/*******************************************************************************
* @fn          sensorICM20649_enableWom
*
* @brief       Enable Wake On Motion functionality
*
* @param       threshold - wake-up trigger threshold (unit: 4 mg, max 1020mg)
*
* @return      True if success
*/
bool sensorICM20649_enableWom_I2C(uint8_t threshold)
{
    bool success;

    // Select Bank
    success = writeReg_I2C(REG_BANK_SEL_I2C, BANK_0_I2C);
    if(success)
    {
        // Make sure accelerometer is running
        val = 0x09;
        if(writeReg_I2C(PWR_MGMT_1_I2C, val) == false) return false;

        // Enable accelerometer, disable gyro
        val = 0x07;
        if(writeReg_I2C(PWR_MGMT_2_I2C, val) == false) return false;
    }

    // Select Bank
    success = writeReg_I2C(REG_BANK_SEL_I2C, BANK_2_I2C);
    if(success)
    {
        // Set Accel LPF setting to 184 Hz Bandwidth
        val = 0x09;
        if(writeReg_I2C(ACCEL_CONFIG_I2C, val) == false) return false;
    }

    // Select Bank
    success = writeReg_I2C(REG_BANK_SEL_I2C, BANK_0_I2C);
    if(success)
    {
        // Make sure accelerometer is running
        val = 0x08;
        if(writeReg_I2C(INT_ENABLE_I2C, val) == false) return false;
    }

    // Select Bank
    success = writeReg_I2C(REG_BANK_SEL_I2C, BANK_2_I2C);
    if(success)
    {
        // Enable Accel Hardware Intelligence
        val = 0x03;
        if(writeReg_I2C(ACCEL_INTEL_CTRL_I2C, val) == false) return false;

        // Set Motion Threshold
        val = threshold;
        if(writeReg_I2C(ACCEL_WOM_THR_I2C, val) == false) return false;


    }
/*
    // Set Frequency of Wake-up
    val = INV_LPA_20HZ;
    ST_ASSERT(SensorI2C_writeReg(LP_ACCEL_ODR, &val, 1));

    // Enable Cycle Mode (Accel Low Power Mode)
    val = 0x29;
    ST_ASSERT(SensorI2C_writeReg(PWR_MGMT_1, &val, 1));

    // Select the current range
    ST_ASSERT(SensorI2C_writeReg(ACCEL_CONFIG, &accRangeReg, 1));

    // Clear interrupt
    SensorI2C_readReg(INT_STATUS,&val,1);
*/
    return success;
}
/*
bool sensorICM20649_enableWom(uint8_t threshold)
{
    ST_ASSERT(sensorICM20649_powerIsOn());

    if (!SENSOR_SELECT())
    {
        return false;
    }

    // Make sure accelerometer is running
    val = 0x09;
    ST_ASSERT(SensorI2C_writeReg(PWR_MGMT_1, &val, 1));

    // Enable accelerometer, disable gyro
    val = 0x07;
    ST_ASSERT(SensorI2C_writeReg(PWR_MGMT_2, &val, 1));

    // Set Accel LPF setting to 184 Hz Bandwidth
    val = 0x01;
    ST_ASSERT(SensorI2C_writeReg(ACCEL_CONFIG_2, &val, 1));

    // Enable Motion Interrupt
    val = BIT_WOM_EN;
    ST_ASSERT(SensorI2C_writeReg(INT_ENABLE, &val, 1));

    // Enable Accel Hardware Intelligence
    val = 0xC0;
    ST_ASSERT(SensorI2C_writeReg(ACCEL_INTEL_CTRL, &val, 1));

    // Set Motion Threshold
    val = threshold;
    ST_ASSERT(SensorI2C_writeReg(WOM_THR, &val, 1));

    // Set Frequency of Wake-up
    val = INV_LPA_20HZ;
    ST_ASSERT(SensorI2C_writeReg(LP_ACCEL_ODR, &val, 1));

    // Enable Cycle Mode (Accel Low Power Mode)
    val = 0x29;
    ST_ASSERT(SensorI2C_writeReg(PWR_MGMT_1, &val, 1));

    // Select the current range
    ST_ASSERT(SensorI2C_writeReg(ACCEL_CONFIG, &accRangeReg, 1));

    // Clear interrupt
    SensorI2C_readReg(INT_STATUS,&val,1);

    SENSOR_DESELECT();

    mpuConfig = 0;

    // Enable pin for wake-on-motion interrupt
    PIN_setInterrupt(hMpuPin, PIN_ID(Board_MPU_INT)|PIN_IRQ_POSEDGE);

    return true;
}
*/
/*******************************************************************************
* @fn          sensorICM20649_irqStatus
*
* @brief       Check whether a data or wake on motion interrupt has occurred
*
* @return      Return interrupt status
*/
uint8_t SensorICM20649_irqStatus_I2C(void)
{
    bool success = false;
    uint8_t intStatus;

    intStatus = 0;
    while (success != true) success = readReg_I2C(INT_STATUS_1_I2C,&intStatus,1);

    return intStatus;
}

/*******************************************************************************
* @fn          sensorICM20649_enable
*
* @brief       Enable accelerometer readout
*
* @param       Axes: Gyro bitmap [0:2]. (111 = gyroscope off, 000 = gyroscope on)
* @                  Acc  bitmap [3:5]. (111 = accelerometer off, 000 = accelerometer on)
*
* @return      None
*/
void sensorICM20649_enable_I2C(uint8_t axes)
{
    if (axes != 0x3F)
    {
        // Wake up the sensor if it was off
        sensorICM20649_WakeUp_I2C();
        //Display_print0(displayOut, 0, 0, "ICM20649 WakeUp");
    }
    else
    {
        sensorICM_Sleep_I2C();
        //Display_print0(displayOut, 0, 0, "ICM20649 Sleep");
    }

    mpuConfig = axes;

    // Enable gyro + accelerometer
    sensorICM20649_SelectAxes_I2C();
}

/*******************************************************************************
* @fn          sensorICM20649_enable_int
*
* @brief       Enable Interrupt 1
*
* @return      True if success
*/
void SensorICM20649_enable_int_I2C(void)
{
    bool success;

    val = 0x01;
    success = false;
    while (success != true) success = writeReg_I2C(REG_BANK_SEL_I2C, BANK_0_I2C);
    success = false;
    while (success != true) success = writeReg_I2C(INT_ENABLE1_I2C, val);

}

/*******************************************************************************
* @fn          sensorICM20649_accSetSMPLRT_DIV
*
* @brief       Set the sample rate divider of the accelerometer
*
* @param       newRange: 0~4095
*
* @return      true if write succeeded
*/
bool sensorICM20649_accSetSMPLRT_DIV_I2C(uint16_t newRange)
{
    bool success;

    if (newRange == accsmplrt_div) return true;

    success = false;
    while (success != true) success = writeReg_I2C(REG_BANK_SEL_I2C, BANK_2_I2C);

    if(newRange > 255)
    {
        accRangeReg = (newRange >> 8) & 0x000F;
        success = false;
        while (success != true) success = writeReg_I2C(ACCEL_SMPLRT_DIV_1_I2C, accRangeReg);
        accRangeReg = newRange & 0x00FF;
        success = false;
        while (success != true) success = writeReg_I2C(ACCEL_SMPLRT_DIV_2_I2C, accRangeReg);
        accsmplrt_div = newRange;
        return true;
    }
    else
    {
        accRangeReg = newRange & 0x00FF;
        success = false;
        while (success != true) success = writeReg_I2C(ACCEL_SMPLRT_DIV_1_I2C, 0x00);
        success = false;
        while (success != true) success = writeReg_I2C(ACCEL_SMPLRT_DIV_2_I2C, accRangeReg);
        accsmplrt_div = newRange;
        return true;
    }
}

/*******************************************************************************
* @fn          sensorICM20649_accSetBW
*
* @brief       Set the DLPF Bandwidth of the accelerometer
*
* @param       newRange: ACC_BW_246_0, ACC_BW_246_0, ACC_BW_111_4, ACC_BW_50_4,
*                        ACC_BW_23_9, ACC_BW_11_5, ACC_BW_5_7, ACC_BW_473_0
*
* @return      true if write succeeded
*/
bool sensorICM20649_accSetBW_I2C(uint8_t newRange)
{
    bool success = false;

    if (newRange == accBW) return true;

    success = false;
    while (success != true) success = writeReg_I2C(REG_BANK_SEL_I2C, BANK_2_I2C);
    success = false;
    while (success != true) success = readReg_I2C(ACCEL_CONFIG_I2C, &accRangeReg, 1);

    // Apply the range
    accRangeReg = (newRange << 3) | (accRangeReg & 0xC7);
    success = false;
    while (success != true) success = writeReg_I2C(ACCEL_CONFIG_I2C, accRangeReg);

    accBW = newRange;
    return true;
}
/*******************************************************************************
* @fn          sensorICM20649_accSetRange
*
* @brief       Set the range of the accelerometer
*
* @param       newRange: ACC_RANGE_4G, ACC_RANGE_8G, ACC_RANGE_16G, ACC_RANGE_30G
*
* @return      true if write succeeded
*/
bool SensorICM20649_accSetRange_I2C(uint8_t newRange)
{
    bool success;

    if (newRange == accRange) return true;

    success = false;
    while (success != true) success = writeReg_I2C(REG_BANK_SEL_I2C, BANK_2_I2C);
    success = false;
    while (success != true) success = readReg_I2C(ACCEL_CONFIG_I2C, &accRangeReg, 1);

    accRangeReg = (newRange << 1) | (accRangeReg & 0xf9) | 0x01;
    // Apply the range
    success = false;
    while (success != true) success = writeReg_I2C(ACCEL_CONFIG_I2C, accRangeReg);

    accRange = newRange;
    return true;
}

/*******************************************************************************
* @fn          sensorICM20649_accReadRange
*
* @brief       Apply the selected accelerometer range
*
* @param       none
*
* @return      range: ACC_RANGE_4G, ACC_RANGE_8G, ACC_RANGE_16G, ACC_RANGE_30G
*/
uint8_t SensorICM20649_accReadRange_I2C(void)
{
    bool success;

    // Apply the range
    success = false;
    while (success != true) success = writeReg_I2C(REG_BANK_SEL_I2C, BANK_2_I2C);
    success = false;
    while (success != true) success = readReg_I2C(ACCEL_CONFIG_I2C, &accRangeReg, 1);

    accRange = (accRangeReg >> 1) & 3;

    return accRange;
}

/*******************************************************************************
* @fn          sensorICM20649_gyroSetSMPLRT_DIV
*
* @brief       Set the sample rate divider of the gyroscope
*
* @param       newRange: 0~255
*
* @return      true if write succeeded
*/
bool sensorICM20649_gyroSetSMPLRT_DIV_I2C(uint8_t newRange)
{
    bool success;

    if (newRange == gyrosmplrt_div) return true;

    success = false;
    while (success != true) success = writeReg_I2C(REG_BANK_SEL_I2C, BANK_2_I2C);
    success = false;
    while (success != true) success = writeReg_I2C(GYRO_SMPLRT_DIV_I2C, newRange);

    gyrosmplrt_div = newRange;

    return true;
}

/*******************************************************************************
* @fn          sensorICM20649_gyroSetBW
*
* @brief       Set the DLPF Bandwidth of the gyroscope
*
* @param       newRange: GYRO_BW_196_6, GYRO_BW_151_8, GYRO_BW_119_5, GYRO_BW_51_2,
*                        GYRO_BW_23_9, GYRO_BW_11_6, GYRO_BW_5_7, GYRO_BW_361_4
*
* @return      true if write succeeded
*/
bool sensorICM20649_gyroSetBW_I2C(uint8_t newRange)
{
    bool success;

    if (newRange == gyroBW) return true;

    success = false;
    while (success != true) success = writeReg_I2C(REG_BANK_SEL_I2C, BANK_2_I2C);
    success = false;
    while (success != true) success = readReg_I2C(GYRO_CONFIG_1_I2C, &gyroRangeReg, 1);

    gyroRangeReg = (newRange << 3) | (gyroRangeReg & 0xC7);
    // Apply the range
    success = false;
    while (success != true) success = writeReg_I2C(GYRO_CONFIG_1_I2C, gyroRangeReg);

    gyroBW = newRange;
    return true;
}

/*******************************************************************************
* @fn          sensorICM20649_gyroSetRange
*
* @brief       Set the range of the gyroscope
*
* @param       newRange: GYRO_RANGE_500DPS, GYRO_RANGE_1000DPS, GYRO_RANGE_2000DPS, GYRO_RANGE_4000DPS
*
* @return      true if write succeeded
*/
bool SensorICM20649_gyroSetRange_I2C(uint8_t newRange)
{
    bool success;

    if (newRange == gyroRange) return true;

    success = false;
    while (success != true) success = writeReg_I2C(REG_BANK_SEL_I2C, BANK_2_I2C);
    success = false;
    while (success != true) success = readReg_I2C(GYRO_CONFIG_1_I2C, &gyroRangeReg, 1);

    gyroRangeReg = (newRange << 1) | (gyroRangeReg & 0xf9);
    // Apply the range
    success = false;
    while (success != true) success = writeReg_I2C(GYRO_CONFIG_1_I2C, gyroRangeReg);

    gyroRange = newRange;
    return true;
}

/*******************************************************************************
* @fn          sensorICM20649_gyroReadRange
*
* @brief       Apply the selected gyroscope range
*
* @param       none
*
* @return      range: GYRO_RANGE_500DPS, GYRO_RANGE_1000DPS, GYRO_RANGE_2000DPS, GYRO_RANGE_4000DPS
*/
uint8_t SensorICM20649_gyroReadRange_I2C(void)
{
    bool success;

    // Apply the range
    success = false;
    while (success != true) success = writeReg_I2C(REG_BANK_SEL_I2C, BANK_2_I2C);
    success = false;
    while (success != true) success = readReg_I2C(GYRO_CONFIG_1_I2C, &gyroRangeReg, 1);

    gyroRange = (gyroRangeReg >> 1) & 3;

    return gyroRange;
}

/*******************************************************************************
* @fn          sensorICM20649_accRead
*
* @brief       Read data from the accelerometer - X, Y, Z - 3 words
*
* @return      True if data is valid
*/
bool SensorICM20649_accRead_I2C(uint8_t *data)
{
    bool success = false;

    // Burst read of all accelerometer values
    while (success != true) success = readReg_I2C(ACCEL_XOUT_H_I2C, data, DATA_SIZE_I2C);

    return success;
}

/*******************************************************************************
* @fn          sensorICM20649_gyroRead
*
* @brief       Read data from the gyroscope - X, Y, Z - 3 words
*
* @return      TRUE if valid data, FALSE if not
*/
bool SensorICM20649_gyroRead_I2C(uint8_t *data)
{
    bool success = false;

    // Burst read of all gyroscope values
    while (success != true) success = readReg_I2C(GYRO_XOUT_H_I2C, data, DATA_SIZE_I2C);

    return success;
}

/*******************************************************************************
 * @fn          sensorICM20649_accConvert
 *
 * @brief       Convert raw data to G units
 *
 * @param       rawData - raw data from sensor
 *
 * @return      Converted value
 ******************************************************************************/
float SensorICM20649_accConvert_I2C(int16_t rawData)
{
    float v;

    switch (accRange)
    {
    case ACC_RANGE_4G:
        //-- calculate acceleration, unit G, range -4, +4
        v = (rawData * 1.0) / (32768/4);
        break;

    case ACC_RANGE_8G:
        //-- calculate acceleration, unit G, range -8, +8
        v = (rawData * 1.0) / (32768/8);
        break;

    case ACC_RANGE_16G:
        //-- calculate acceleration, unit G, range -16, +16
        v = (rawData * 1.0) / (32768/16);
        break;

    case ACC_RANGE_30G:
        //-- calculate acceleration, unit G, range -30, +30
        v = (rawData * 1.0) / (32768/30);
        break;
    }

    return v;
}

/*******************************************************************************
 * @fn          sensorICM20649_gyroConvert
 *
 * @brief       Convert raw data to deg/sec units
 *
 * @param       data - raw data from sensor
 *
 * @return      none
 ******************************************************************************/
float SensorICM20649_gyroConvert_I2C(int16_t rawData)
{
    float v;

    switch (gyroRange)
    {
    case GYRO_RANGE_500DPS:
        //-- calculate rotation, unit deg/s, range -500, +500
        v = (rawData * 1.0) / (32768/500);
        break;

    case GYRO_RANGE_1000DPS:
        //-- calculate rotation, unit deg/s, range -1000, +1000
        v = (rawData * 1.0) / (32768/1000);
        break;

    case GYRO_RANGE_2000DPS:
        //-- calculate rotation, unit deg/s, range -2000, +2000
        v = (rawData * 1.0) / (32768/2000);
        break;

    case GYRO_RANGE_4000DPS:
        //-- calculate rotation, unit deg/s, range -4000, +4000
        v = (rawData * 1.0) / (32768/4000);
        break;
    }

    return v;
}

/*******************************************************************************
* @fn          sensorMpuSleep
*
* @brief       Place the ICM in low power mode
*
* @return
*/
static void sensorICM_Sleep_I2C(void)
{
    bool success;

    val = ALL_AXES_I2C;
    success = false;
    while (success != true) success = writeReg_I2C(REG_BANK_SEL_I2C, BANK_0_I2C);
    success = false;
    while (success != true) success = writeReg_I2C(PWR_MGMT_2_I2C, val);

    val = ICM_SLEEP_I2C;
    success = false;
    while (success != true) success = writeReg_I2C(PWR_MGMT_1_I2C, val);
}

/*******************************************************************************
* @fn          sensorICM20649WakeUp
*
* @brief       Exit low power mode
*
* @return      none
*/
static void sensorICM20649_WakeUp_I2C(void)
{
    bool success;

    val = ICM_WAKE_UP_I2C;
    success = false;
    while (success != true) success = writeReg_I2C(REG_BANK_SEL_I2C, BANK_0_I2C);
    success = false;
    while (success != true) success = writeReg_I2C(PWR_MGMT_1_I2C, val);

    // All axis initially disabled
    val = ALL_AXES_I2C;
    success = false;
    while (success != true) success = writeReg_I2C(PWR_MGMT_2_I2C, val);
    mpuConfig = 0;

    // Restore the range
    /*
    success = false;
    while (success != true) success = writeReg(REG_BANK_SEL, BANK_2);
    success = false;
    while (success != true) success = writeReg(ACCEL_CONFIG, accRangeReg);
    success = false;
    while (success != true) success = writeReg(GYRO_CONFIG_1, gyroRangeReg);
    */

    // Clear interrupts
    success = false;
    while (success != true) success = readReg_I2C(INT_STATUS_I2C,&val,1);
}

/*******************************************************************************
* @fn          sensorICM20649_SelectAxes
*
* @brief       Select gyro, accelerometer
*
* @return      none
*/
static void sensorICM20649_SelectAxes_I2C(void)
{
    bool success;

    // Select gyro and accelerometer (3+3 axes, one bit each)
    val = mpuConfig;
    success = false;
    while (success != true) success = writeReg_I2C(REG_BANK_SEL_I2C, BANK_0_I2C);
    success = false;
    while (success != true) success = writeReg_I2C(PWR_MGMT_2_I2C, val);
}

bool readReg_I2C(uint8_t reg_addr, uint8_t *reg_data, uint8_t len)
{
    //uint8_t *reg_data;                    // holds byte of read data
    I2C_Transaction i2cTransaction;

    //Point I2C parameters to correct values. Set up to read "cnt" number of bytes.
    i2cTransaction.writeBuf     = &reg_addr;
    i2cTransaction.writeCount   = 1;
    i2cTransaction.readBuf      = reg_data;
    i2cTransaction.readCount    = len;
    i2cTransaction.slaveAddress = ICM20649_I2C_ADDR2;

    //Perform I2C read
    return I2C_transfer(i2cHandle, &i2cTransaction) == true;
}

bool writeReg_I2C(uint8_t reg_addr, uint8_t reg_data)
{

    I2C_Transaction i2cTransaction;
    uint8_t array[2];
    array[0] = reg_addr;
    array[1] = reg_data;

    i2cTransaction.writeBuf     = array;
    i2cTransaction.writeCount   = 2;
    i2cTransaction.readBuf      = NULL;
    i2cTransaction.readCount    = 0;
    i2cTransaction.slaveAddress = ICM20649_I2C_ADDR2;

    return I2C_transfer(i2cHandle, &i2cTransaction) == true;
}
