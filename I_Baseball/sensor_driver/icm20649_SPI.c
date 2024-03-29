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
#include <stdbool.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include <ti/drivers/dpl/ClockP.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/dpl/SemaphoreP.h>
#include <ti/display/Display.h>
#include <sensor_driver/icm20649_SPI.h>
#include <ti/posix/ccs/unistd.h>
//#include <ti/drivers/I2C.h>
#include <ti/drivers/SPI.h>
#include "Board.h"
#include <ti/drivers/GPIO.h>
#include "../Board.h"

//extern I2C_Handle i2cHandle;
static SPI_Handle spiHandle;
extern Display_Handle displayOut;
static inline void assertCS(void);
static inline void deassertCS(void);
static int_fast16_t readAcc(uint8_t reg_addr, size_t len);
static int_fast16_t readGyro(uint8_t reg_addr, size_t len);
static int_fast16_t readRegtry(uint8_t reg_addr, size_t len);
unsigned char writeReg(uint8_t reg_addr, uint8_t reg_data);
//static void bank_select(SPI_Handle spiHandle,uint8_t bank);
unsigned char data_acc[7] = {0};
unsigned char data_gyro[7] = {0};
int count = 0;
/* -----------------------------------------------------------------------------
*  Constants and macros
* ------------------------------------------------------------------------------
*/
/*------------ ICM 20649 I2C address ------------*/
//#define ICM20649_I2C_ADDR1            0x68 //AD0 is logic low
//#define ICM20649_I2C_ADDR2            0x69 //AD0 is logic high

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

/* -----------------------------------------------------------------------------
*  Local Functions
* ------------------------------------------------------------------------------
*/
static void sensorICM_Sleep(void);
static void sensorICM20649_WakeUp(void);
static void sensorICM20649_SelectAxes(void);
void checkRegset(void);

/* -----------------------------------------------------------------------------
*  Local Variables
* ------------------------------------------------------------------------------
*/
static uint8_t mpuConfig;
static uint8_t accBW;
static uint16_t accsmplrt_div;
static uint8_t accRange;
static uint8_t accSample;
static uint8_t accRangeReg;
static uint8_t gyroBW;
static uint8_t gyroSample;
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
bool SensorICM20649_init(void)
{
    bool success, ret;
    SPI_Params spiParams;
    uint8_t try;


    /* Initialize SPI handle as default master */


        SPI_Params_init(&spiParams);

        //spiParams.transferMode        = SPI_MODE_CALLBACK;   /*!< Blocking or Callback mode */
        //spiParams.transferTimeout     = SPI_WAIT_FOREVER;    /*!< Transfer timeout in system ticks */
        //spiParams.transferCallbackFxn = SPIcallback;         /*!< Callback function pointer */
        //spiParams.mode                = SPI_SLAVE;            /*!< Master or Slave mode */
       spiParams.bitRate             = 7000000;//100000;       /*!< SPI bit rate in Hz */ //1000000; for 500sps, must more than 40kHz(500*72)
        spiParams.dataSize            = 8;                   /*!< SPI data frame size in bits */
        spiParams.frameFormat         = SPI_POL0_PHA0;         /*!< SPI frame format */ //default: SPI_POL0_PHA0, ads1292r spi settings are CPOL=0 and CPHA=1

        spiHandle = SPI_open(Board_SPI1, &spiParams);


       if (spiHandle == NULL) {
          //      Display_printf(displayOut, 0, 0, "Error initializing ICM SPI");
            }
            else {
                Display_printf(displayOut, 0, 0, "ICM SPI initialized");
            }


   // Display_printf(displayOut, 0, 0, " ICM20649 initializing SPI0");

    accRange = ACC_RANGE_INVALID;
    gyroRange = GYRO_RANGE_INVALID;
    mpuConfig = 0;   // All axes off

    // Device reset
    val = 0x80;
    success = false;

  //  assertCS();
  //  try = readRegtry(PWR_MGMT_1,  1);
  //  deassertCS();
  // Display_print1(displayOut, 0, 0, "PWR_MGMT_1: %x", try);

   // usleep(10000);

    assertCS();
    while (success != true) success = writeReg(REG_BANK_SEL, BANK_0);
    deassertCS();
    success = false;

  //  usleep(1000);

    assertCS();
    while (success != true) success = writeReg(PWR_MGMT_1, val);
    deassertCS();
    success = false;

  //  usleep(1000);

    assertCS();
    while (success != true) success = writeReg(USER_CTRL, 0x90);
    deassertCS();
    success = false;

    ret = false;

  //  assertCS();
  //  try = readRegtry(spiHandle, WHO_AM_I,  1);
  // deassertCS();
  // Display_print1(displayOut, 0, 0, "WHO_AM_I_try : %x", try);



    while(ret != true) ret = SensorICM20649_test();




    // Power save

    sensorICM_Sleep();



    return ret;
}

/*******************************************************************************
 * @fn          sensorICM20649_test
 *
 * @brief       Run a sensor self-test
 *
 * @return      TRUE if passed, FALSE if failed
 */
bool SensorICM20649_test(void)
{
    bool success;
    uint8_t try;
    uint8_t test = 1;
    // Make sure power is ramped up
    sleep(0.1);     //100ms

    // Check the WHO AM I register
    val = 0;
    success = false;

  //  assertCS();
   // success = writeReg(REG_BANK_SEL, BANK_0);
   // deassertCS();

   // sleep(0.1);

    assertCS();
    try = readRegtry(PWR_MGMT_1,1);
    deassertCS();
    Display_print1(displayOut, 0, 0, "PWR_MGMT_1: %x", try);

    assertCS();
    try = readRegtry(USER_CTRL,1);
    deassertCS();
    Display_print1(displayOut, 0, 0, "USER_CTRL: %x", try);

    assertCS();
    try = readRegtry(WHO_AM_I,1);
    deassertCS();
    Display_print1(displayOut, 0, 0, "WHO_AM_I_try1 : %x", try);

    val = try;

    return true;

    if(val == 0xE1)
    {
        return true;
    }
    else
    {
        return false;
        Display_printf(displayOut, 0, 0, "WHO_AM_I ERROR");
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
bool sensorICM20649_enableWom(uint8_t threshold)
{
    bool success;

    // Select Bank
    assertCS();
    success = writeReg(REG_BANK_SEL, BANK_0);
    deassertCS();
    if(success)
    {
        // Make sure accelerometer is running
        assertCS();
        val = 0x09;
        if(writeReg(PWR_MGMT_1, val) == false) return false;
        deassertCS();

        // Enable accelerometer, disable gyro
        assertCS();
        val = 0x07;
        if(writeReg(PWR_MGMT_2, val) == false) return false;
        deassertCS();
    }

    // Select Bank
    success = writeReg(REG_BANK_SEL, BANK_2);
    if(success)
    {
        // Set Accel LPF setting to 184 Hz Bandwidth
        assertCS();
        val = 0x09;
        if(writeReg(ACCEL_CONFIG, val) == false) return false;
        deassertCS();
    }

    // Select Bank
    assertCS();
    success = writeReg(REG_BANK_SEL, BANK_0);
    deassertCS();
    if(success)
    {
        // Make sure accelerometer is running
        assertCS();
        val = 0x08;
        if(writeReg(INT_ENABLE, val) == false) return false;
        deassertCS();
    }

    // Select Bank
    assertCS();
    success = writeReg(REG_BANK_SEL, BANK_2);
    deassertCS();
    if(success)
    {
        // Enable Accel Hardware Intelligence
        assertCS();
        val = 0x03;
        if(writeReg(ACCEL_INTEL_CTRL, val) == false) return false;
        deassertCS();

        // Set Motion Threshold
        assertCS();
        val = threshold;
        if(writeReg(ACCEL_WOM_THR, val) == false) return false;
        deassertCS();


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
uint8_t SensorICM20649_irqStatus(void)
{
    bool success = false;
    uint8_t intStatus;

    intStatus = 0;
    assertCS();
    intStatus = readRegtry(INT_STATUS_1,1);
    deassertCS();

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
void sensorICM20649_enable(uint8_t axes)
{

    if (axes != 0x3F)
    {
        // Wake up the sensor if it was off
        assertCS();
        writeReg(ACCEL_CONFIG, 0x07);
        deassertCS();

        sensorICM20649_WakeUp();
        //Display_print0(displayOut, 0, 0, "ICM20649 WakeUp");
    }
    else
    {
        sensorICM_Sleep();
        //Display_print0(displayOut, 0, 0, "ICM20649 Sleep");
    }

    mpuConfig = axes;

    // Enable gyro + accelerometer
    sensorICM20649_SelectAxes();
}

/*******************************************************************************
* @fn          sensorICM20649_enable_int
*
* @brief       Enable Interrupt 1
*
* @return      True if success
*/
void SensorICM20649_enable_int(void)
{
    bool success;


    val = 0x01;
    success = false;
    assertCS();
    while (success != true) success = writeReg(REG_BANK_SEL, BANK_0);
    deassertCS();
    success = false;
    assertCS();
    while (success != true) success = writeReg(INT_ENABLE1, val);
    deassertCS();

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
bool sensorICM20649_accSetSMPLRT_DIV(uint16_t newRange)
{
    bool success;


    if (newRange == accsmplrt_div) return true;

    success = false;
    assertCS();
    while (success != true) success = writeReg(REG_BANK_SEL, BANK_2);
    deassertCS();
    if(newRange > 255)
    {
        accRangeReg = (newRange >> 8) & 0x000F;
        success = false;
        assertCS();
        while (success != true) success = writeReg(ACCEL_SMPLRT_DIV_1, accRangeReg);
        deassertCS();
        accRangeReg = newRange & 0x00FF;
        success = false;
        assertCS();
        while (success != true) success = writeReg(ACCEL_SMPLRT_DIV_2, accRangeReg);
        deassertCS();
        accsmplrt_div = newRange;
        return true;
    }
    else
    {
        accRangeReg = newRange & 0x00FF;
        success = false;
        assertCS();
        while (success != true) success = writeReg(ACCEL_SMPLRT_DIV_1, 0x00);
        deassertCS();
        success = false;
        assertCS();
        while (success != true) success = writeReg(ACCEL_SMPLRT_DIV_2, accRangeReg);
        deassertCS();
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
bool sensorICM20649_accSetBW(uint8_t newRange)
{
    bool success = false;


    if (newRange == accBW) return true;

    success = false;
    assertCS();
    success = writeReg(REG_BANK_SEL, BANK_2);
    deassertCS();
    success = false;
    assertCS();
    accRangeReg = readRegtry(ACCEL_CONFIG,  1);
    deassertCS();

    // Apply the range
    accRangeReg = (newRange << 3) | (accRangeReg & 0xC6);
    success = false;
    assertCS();
    success = writeReg(ACCEL_CONFIG, accRangeReg);
    deassertCS();

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
bool SensorICM20649_accSetRange(uint8_t newRange)
{
    bool success;


 //   if (newRange == accRange) return true;

    success = false;
    assertCS();
    success = writeReg(REG_BANK_SEL, BANK_2);
    deassertCS();
  //  success = false;
   // assertCS();
  //  accRangeReg = readRegtry(ACCEL_CONFIG, 1);
   // deassertCS();

 //   accRangeReg = (newRange << 1) | (accRangeReg & 0xf9) | 0x01;
    // Apply the range

    assertCS();
    success = writeReg(ACCEL_CONFIG, newRange);
    deassertCS();

    usleep(1000);

    accRange = newRange;
    return true;
}

bool SensorICM20649_accSetSample(uint8_t newRange)
{
    bool success;


    if (newRange == accSample) return true;

    success = false;
    assertCS();
    success = writeReg(REG_BANK_SEL, BANK_2);
    deassertCS();
    success = false;
    assertCS();
    accRangeReg = readRegtry(ACCEL_CONFIG, 1);
    deassertCS();

    accRangeReg = (newRange) | (accRangeReg & 0xf8);
    // Apply the range
    success = false;
    assertCS();
    success = writeReg(ACCEL_CONFIG, accRangeReg);
    deassertCS();

    accSample = newRange;
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
uint8_t SensorICM20649_accReadRange(void)
{
    bool success;


    // Apply the range
    success = false;
    assertCS();
    success = writeReg(REG_BANK_SEL, BANK_2);
    deassertCS();
    success = false;
    assertCS();
    accRangeReg = readRegtry(ACCEL_CONFIG, 1);
    deassertCS();

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
bool sensorICM20649_gyroSetSMPLRT_DIV(uint8_t newRange)
{
    bool success;


    if (newRange == gyrosmplrt_div) return true;

    success = false;
    assertCS();
    while (success != true) success = writeReg(REG_BANK_SEL, BANK_2);
    deassertCS();
    success = false;
    assertCS();
    while (success != true) success = writeReg(GYRO_SMPLRT_DIV, newRange);
    deassertCS();

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
bool sensorICM20649_gyroSetBW(uint8_t newRange)
{
    bool success;


    if (newRange == gyroBW) return true;

    success = false;
    assertCS();
    success = writeReg(REG_BANK_SEL, BANK_2);
    deassertCS();
    success = false;
    assertCS();
    gyroRangeReg = readRegtry(GYRO_CONFIG_1, 1);
    deassertCS();

    gyroRangeReg = (newRange << 3) | (gyroRangeReg & 0xC6);
    // Apply the range
    success = false;
    assertCS();
    success = writeReg(GYRO_CONFIG_1, gyroRangeReg);
    deassertCS();

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
bool SensorICM20649_gyroSetRange(uint8_t newRange)
{
    bool success;
    uint8_t try;

 //   if (newRange == gyroRange) return true;

    success = false;
    assertCS();
    success = writeReg(REG_BANK_SEL, BANK_2);
    deassertCS();
  //  success = false;
  //  assertCS();
  //  gyroRangeReg = readRegtry(GYRO_CONFIG_1, 1);
   // deassertCS();

   // gyroRangeReg = (newRange << 1) | (gyroRangeReg & 0xf9);
    // Apply the range
 //   success = false;
    assertCS();
    success = writeReg(GYRO_CONFIG_1, 0x37);
    deassertCS();


    assertCS();
    try = readRegtry(ACCEL_CONFIG,1);
    deassertCS();
    Display_print1(displayOut, 0, 0, "ACCEL_CONFIG: %x", try);

    assertCS();
    try = readRegtry(GYRO_CONFIG_1,1);
    deassertCS();
    Display_print1(displayOut, 0, 0, "GYRO_CONFIG_1: %x", try);

    assertCS();
    try = readRegtry(GYRO_SMPLRT_DIV,1);
    deassertCS();
    Display_print1(displayOut, 0, 0, "GYRO_SMPLRT_DIV: %x", try);

    assertCS();
    try = readRegtry(ACCEL_SMPLRT_DIV_2,1);
    deassertCS();
    Display_print1(displayOut, 0, 0, "ACCEL_SMPLRT_DIV_2: %x", try);

    assertCS();
    try = readRegtry(ACCEL_SMPLRT_DIV_1,1);
    deassertCS();
    Display_print1(displayOut, 0, 0, "ACCEL_SMPLRT_DIV_1: %x", try);

    assertCS();
    success = writeReg(REG_BANK_SEL, BANK_0);
    deassertCS();


    gyroRange = newRange;
    return true;
}


bool SensorICM20649_gyroSetSample(uint8_t newRange)
{
    bool success;


    if (newRange == gyroSample) return true;

    success = false;
    assertCS();
    while (success != true) success = writeReg(REG_BANK_SEL, BANK_2);
    deassertCS();
    success = false;
    assertCS();
    gyroRangeReg = readRegtry(GYRO_CONFIG_1,  1);
    deassertCS();

    gyroRangeReg = (newRange) | (gyroRangeReg & 0xf8);
    // Apply the range
    success = false;
    assertCS();
    while (success != true) success = writeReg(GYRO_CONFIG_1, gyroRangeReg);
    deassertCS();

    gyroSample = newRange;
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
uint8_t SensorICM20649_gyroReadRange(void)
{
    bool success;


    // Apply the range
    success = false;
    assertCS();
    while (success != true) success = writeReg(REG_BANK_SEL, BANK_2);
    deassertCS();
    success = false;
    assertCS();
    gyroRangeReg = readRegtry(GYRO_CONFIG_1, 1);
    deassertCS();

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
bool SensorICM20649_accRead(uint8_t *data)
{
    bool success = false;

//    uint8_t data_acc;


    // Burst read of all accelerometer values
    assertCS();
    success = readAcc(ACCEL_XOUT_H,DATA_SIZE);
    deassertCS();


 //   if(count == 112500)Display_print0(displayOut, 0, 0, "cnt = 112500!");

    return success;
}

/*******************************************************************************
* @fn          sensorICM20649_gyroRead
*
* @brief       Read data from the gyroscope - X, Y, Z - 3 words
*
* @return      TRUE if valid data, FALSE if not
*/
bool SensorICM20649_gyroRead(uint8_t *data)
{
    bool success = false;
//    uint8_t data_gyro;


    // Burst read of all gyroscope values
    assertCS();
    success = readGyro(GYRO_XOUT_H,DATA_SIZE);
    deassertCS();

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
float SensorICM20649_accConvert(int16_t rawData)
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
float SensorICM20649_gyroConvert(int16_t rawData)
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
static void sensorICM_Sleep(void)
{
    bool success;
    uint8_t try;

    val = ALL_AXES;
    success = false;
    assertCS();
    while (success != true) success = writeReg(REG_BANK_SEL, BANK_0);
    deassertCS();
    success = false;
    assertCS();
    while (success != true) success = writeReg(PWR_MGMT_2, val);
    deassertCS();

    val = ICM_SLEEP;
    success = false;
    assertCS();
    while (success != true) success = writeReg(PWR_MGMT_1, val);
    deassertCS();
/*
    assertCS();
    try = readRegtry(PWR_MGMT_1,1);
    deassertCS();
 //   Display_print1(displayOut, 0, 0, "PWR_MGMT_1: %x", try);

    assertCS();
    try = readRegtry(PWR_MGMT_2,1);
    deassertCS();
  //  Display_print1(displayOut, 0, 0, "PWR_MGMT_2: %x", try);
      */
}

/*******************************************************************************
* @fn          sensorICM20649WakeUp
*
* @brief       Exit low power mode
*
* @return      none
*/
static void sensorICM20649_WakeUp(void)
{
    bool success;


    val = ICM_WAKE_UP;
    success = false;
    assertCS();
    success = writeReg(REG_BANK_SEL, BANK_0);
    deassertCS();
    success = false;
    assertCS();
    success = writeReg(PWR_MGMT_1, val);
    deassertCS();

    // All axis initially disabled
    val = ALL_AXES;
    success = false;
    assertCS();
    success = writeReg(PWR_MGMT_2, val);
    deassertCS();
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
    assertCS();
    val = readRegtry(INT_STATUS,1);
    deassertCS();
}

/*******************************************************************************
* @fn          sensorICM20649_SelectAxes
*
* @brief       Select gyro, accelerometer
*
* @return      none
*/
static void sensorICM20649_SelectAxes(void)
{
    bool success;
    uint8_t try;

    // Select gyro and accelerometer (3+3 axes, one bit each)
    val = mpuConfig;
    success = false;
    assertCS();
    success = writeReg(REG_BANK_SEL, BANK_0);
    deassertCS();
    success = false;

    Display_print1(displayOut, 0, 0, "SelectAxes : %x", val);

    assertCS();
    success = writeReg(PWR_MGMT_2, val);
    deassertCS();
/*
    assertCS();
    try = readRegtry(PWR_MGMT_2, 1);
    deassertCS();
    Display_print1(displayOut, 0, 0, "SelectAxes : %x", try);
*/

}

static int_fast16_t readAcc(uint8_t reg_addr,  size_t len)
{
    //uint8_t *reg_data;                    // holds byte of read data
    SPI_Transaction transaction;
    int_fast16_t    status;
    uint8_t senddata[2];


    senddata[0] = (reg_addr | 0x80);
   // senddata[1] = ;


    transaction.count = len+1;
    transaction.txBuf = (void *)senddata;
    transaction.rxBuf = (void *)data_acc;
    transaction.arg   = NULL;



  //  Display_printf(displayOut, 0, 0, "read ICM register \n");

    status = SPI_transfer(spiHandle, &transaction); //0: FAIL 1: SUCCESS, Flash_Success = 0


  //  if(!status)
 //   Display_printf(displayOut, 0, 0, "read ICM register fail!\n");

  //  retVal[1] = 0;


        return (status);



}

static int_fast16_t readGyro(uint8_t reg_addr,  size_t len)
{
    //uint8_t *reg_data;                    // holds byte of read data
    SPI_Transaction transaction;
    int_fast16_t    status;
    uint8_t senddata[2];


    senddata[0] = (reg_addr | 0x80);
   // senddata[1] = ;


    transaction.count = len+1;
    transaction.txBuf = (void *)senddata;
    transaction.rxBuf = (void *)data_gyro;
    transaction.arg   = NULL;



  //  Display_printf(displayOut, 0, 0, "read ICM register \n");

    status = SPI_transfer(spiHandle, &transaction); //0: FAIL 1: SUCCESS, Flash_Success = 0


  //  if(!status)
  //  Display_printf(displayOut, 0, 0, "read ICM register fail!\n");

  //  retVal[1] = 0;


        return (status);



}


static int_fast16_t readRegtry(uint8_t reg_addr, size_t len)
{
    //uint8_t *reg_data;                    // holds byte of read data
    SPI_Transaction transaction;
    int_fast16_t    status;
    uint8_t senddata[2];
    uint8_t retVal[2]={0};
    uint8_t AAAA;

    senddata[0] = (reg_addr | 0x80);
   // senddata[1] = ;

   // senddata[0] = reg_addr;

  // bank_select(spiHandle,bank);

  //  assertCS();
    transaction.count =  1+ len;
    transaction.txBuf = (void *)senddata;
    transaction.rxBuf = (void *)retVal;
    transaction.arg   = NULL;

  //  Display_printf(displayOut, 0, 0, "AAAAf %x \n",AAAA);

  //  deassertCS();

 //   Display_printf(displayOut, 0, 0, "read ICM register \n");

    status = SPI_transfer(spiHandle, &transaction); //0: FAIL 1: SUCCESS, Flash_Success = 0


    if(!status)
    Display_printf(displayOut, 0, 0, "read ICM register fail!\n");

    //retVal[1] = 0;

  //  Display_printf(displayOut, 0, 0, "senddata[0] %x \n", senddata[0]);
  //  Display_printf(displayOut, 0, 0, "retVal0 %x \n",retVal[0]);
 //   Display_printf(displayOut, 0, 0, "retVal1 %x \n",retVal[1]);
    AAAA = retVal[1];
  //  Display_printf(displayOut, 0, 0, "AAAAb %x \n",AAAA);


        return AAAA;



}


unsigned char writeReg(uint8_t reg_addr, uint8_t reg_data)
{
  //  SPI_Handle handle;
    SPI_Transaction transaction;
    bool    status;
    uint8_t array[2];
    uint8_t rxBuffer[2]={0};

   // bank_select(spiHandle,bank);

    array[0] = (reg_addr & 0x7F);
    array[1] = reg_data;

  // Display_printf(displayOut, 0, 0, "reg_addr %x\n",array[0]);
  //  Display_printf(displayOut, 0, 0, "reg_data %x\n",array[1]);
    usleep(10000);

   // Display_printf(displayOut, 0, 0, "write ICM register !\n");
    transaction.count = 2;
    transaction.txBuf = (void *)array;
    transaction.rxBuf = (void *)NULL;
  //  transaction.arg    = NULL;
   // Display_printf(displayOut, 0, 0, "write ICM register !\n");
    status = SPI_transfer(spiHandle, &transaction) ; //0: FAIL 1: SUCCESS, Flash_Success = 0
          //  ICM_Success : ICM_Error;
    //Display_printf(displayOut, 0, 0, "write ICM register !\n");
  //  Display_printf(displayOut, 0, 0, "status = %d !\n",status);

    if(!status)
            Display_printf(displayOut, 0, 0, "write ICM register fail!\n");

  //  usleep(10000);

        return (status);
}

void checkRegset(void)
{
    bool success;
    uint8_t try;

    assertCS();
    success = writeReg(REG_BANK_SEL, BANK_0);
    deassertCS();

    assertCS();
    try = readRegtry(PWR_MGMT_1,1);
    deassertCS();
    Display_print1(displayOut, 0, 0, "PWR_MGMT_1: %x", try);

    assertCS();
    try = readRegtry(PWR_MGMT_2,1);
    deassertCS();
    Display_print1(displayOut, 0, 0, "PWR_MGMT_2: %x", try);

    assertCS();
    try = readRegtry(USER_CTRL,1);
    deassertCS();
    Display_print1(displayOut, 0, 0, "USER_CTRL: %x", try);

    assertCS();
    success = writeReg(REG_BANK_SEL, BANK_2);
    deassertCS();
    /*
    assertCS();
   // success = writeReg(ACCEL_CONFIG, accRangeReg);
    success = writeReg(ACCEL_CONFIG, 0x07);
    deassertCS();

    assertCS();
   // success = writeReg(GYRO_CONFIG_1, gyroRangeReg);
    success = writeReg(GYRO_CONFIG_1, 0x07);
    deassertCS();
*/

    assertCS();
    try = readRegtry(ACCEL_CONFIG,1);
    deassertCS();
    Display_print1(displayOut, 0, 0, "ACCEL_CONFIG: %x", try);

    assertCS();
    try = readRegtry(GYRO_CONFIG_1,1);
    deassertCS();
    Display_print1(displayOut, 0, 0, "GYRO_CONFIG_1: %x", try);





}


static inline void assertCS(void)
{
    GPIO_write(ICM20649_nCS, 0);  //orignal have
}

static inline void deassertCS(void)
{
    GPIO_write(ICM20649_nCS, 1);  //orignal have
}
