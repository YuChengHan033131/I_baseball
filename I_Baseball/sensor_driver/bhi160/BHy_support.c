/*!
  * Copyright (C) Robert Bosch. All Rights Reserved.
  *
  * <Disclaimer>
  * Common: Bosch Sensortec products are developed for the consumer goods
  * industry. They may only be used within the parameters of the respective valid
  * product data sheet.  Bosch Sensortec products are provided with the express
  * understanding that there is no warranty of fitness for a particular purpose.
  * They are not fit for use in life-sustaining, safety or security sensitive
  * systems or any system or device that may lead to bodily harm or property
  * damage if the system or device malfunctions. In addition, Bosch Sensortec
  * products are not fit for use in products which interact with motor vehicle
  * systems.  The resale and/or use of products are at the purchaser's own risk
  * and his own responsibility. The examination of fitness for the intended use
  * is the sole responsibility of the Purchaser.
  *
  * The purchaser shall indemnify Bosch Sensortec from all third party claims,
  * including any claims for incidental, or consequential damages, arising from
  * any product use not covered by the parameters of the respective valid product
  * data sheet or not approved by Bosch Sensortec and reimburse Bosch Sensortec
  * for all costs in connection with such claims.
  *
  * The purchaser must monitor the market for the purchased products,
  * particularly with regard to product safety and inform Bosch Sensortec without
  * delay of all security relevant incidents.
  *
  * Engineering Samples are marked with an asterisk (*) or (e). Samples may vary
  * from the valid technical specifications of the product series. They are
  * therefore not intended or fit for resale to third parties or for use in end
  * products. Their sole purpose is internal client testing. The testing of an
  * engineering sample may in no way replace the testing of a product series.
  * Bosch Sensortec assumes no liability for the use of engineering samples. By
  * accepting the engineering samples, the Purchaser agrees to indemnify Bosch
  * Sensortec from all claims arising from the use of engineering samples.
  *
  * Special: This software module (hereinafter called "Software") and any
  * information on application-sheets (hereinafter called "Information") is
  * provided free of charge for the sole purpose to support your application
  * work. The Software and Information is subject to the following terms and
  * conditions:
  *
  * The Software is specifically designed for the exclusive use for Bosch
  * Sensortec products by personnel who have special experience and training. Do
  * not use this Software if you do not have the proper experience or training.
  *
  * This Software package is provided `` as is `` and without any expressed or
  * implied warranties, including without limitation, the implied warranties of
  * merchantability and fitness for a particular purpose.
  *
  * Bosch Sensortec and their representatives and agents deny any liability for
  * the functional impairment of this Software in terms of fitness, performance
  * and safety. Bosch Sensortec and their representatives and agents shall not be
  * liable for any direct or indirect damages or injury, except as otherwise
  * stipulated in mandatory applicable law.
  *
  * The Information provided is believed to be accurate and reliable. Bosch
  * Sensortec assumes no responsibility for the consequences of use of such
  * Information nor for any infringement of patents or other rights of third
  * parties which may result from its use. No license is granted by implication
  * or otherwise under any patent or patent rights of Bosch. Specifications
  * mentioned in the Information are subject to change without notice.
  *
  *
  * @file              bhy_support.c
  *
  * @date              12/19/2016
  *
  * @brief             driver on MCU for bhy
  *
  *
  */


/********************************************************************************/
/*                                  HEADER FILES                                */
/********************************************************************************/
#include <unistd.h>
#include <ti/drivers/I2C.h>
#include "BHy_support.h"
/********************************************************************************/
/*                                STATIC VARIABLES                              */
/********************************************************************************/

/*! instantiates a BHY software instance structure which retains
* chip ID, internal sensors IDs, I2C address and pointers
* to required functions (bus read/write and delay functions) */
static struct bhy_t bhy;

/*! instantiates an I2C packet software instance structure which retains
* I2C slave address, data buffer and data length and is used to read/write
* data on the I2C bus */
extern I2C_Handle i2cHandle;
/********************************************************************************/
/*                         EXTERN FUNCTION DECLARATIONS                         */
/********************************************************************************/
//extern void mdelay(uint32_t ul_dly_ticks);

/********************************************************************************/
/*                             FUNCTION DECLARATIONS                            */
/********************************************************************************/
static bool i2cTransferFxn(I2C_Handle handle, I2C_Transaction transaction)
{
    if (I2C_transfer(handle, &transaction))
    {
        return (true);
    }
    else
    {
        return (false);
    }
}

/*!
* @brief        Sends data to BHY via I2C
*
* @param[in]    dev_addr    Device I2C slave address
* @param[in]    reg_addr    Address of destination register
* @param[in]    wr_buf   Pointer to data buffer to be sent
* @param[in]    wr_len    Length of the data to be sent
*
* @retval       0           BHY_SUCCESS
* @retval       -1          BHY_ERROR
*
*/
int8_t bhy_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *wr_buf, uint16_t wr_len)
{
    int8_t ret = BHY_SUCCESS;
    //uint8_t cnt = 0;
    uint8_t buffer[33]={0};
    uint8_t stringpos = 0;

    I2C_Transaction i2cTransaction;

    //cnt = wr_len/I2C_ONCE_READ_MAX_COUNT;
    buffer[0] = reg_addr;
    for (stringpos = 0; stringpos < wr_len; stringpos++) {
        buffer[stringpos + 1] = *(wr_buf + stringpos);
    }

    i2cTransaction.writeBuf = buffer;
    i2cTransaction.writeCount = wr_len + 1;
    i2cTransaction.readCount = 0;
    i2cTransaction.slaveAddress = dev_addr;

    if (!i2cTransferFxn(i2cHandle, i2cTransaction))
    {
        ret = BHY_ERROR;
    }

    return ret;
}

/*!
* @brief        Receives data from BHY on I2C
*
* @param[in]    dev_addr    Device I2C slave address
* @param[in]    reg_addr    Address of destination register
* @param[out]   rd_buf  Pointer to data buffer to be received
* @param[in]    rd_len    Length of the data to be received
*
* @retval       0           BHY_SUCCESS
* @retval       -1          BHY_ERROR
*
*/
int8_t bhy_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *rd_buf, uint16_t rd_len)
{
    int8_t ret = BHY_SUCCESS;
    //uint8_t cnt = 0;
    I2C_Transaction i2cTransaction;

    //cnt = rd_len/I2C_ONCE_READ_MAX_COUNT;
    i2cTransaction.writeBuf = &reg_addr;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rd_buf;
    i2cTransaction.readCount = rd_len;//cnt;
    i2cTransaction.slaveAddress = dev_addr;

    if (!i2cTransferFxn(i2cHandle, i2cTransaction))
    {
        ret = BHY_ERROR;
    }
    return ret;
}

/*!
* @brief        Initializes BHY smart sensor and its required connections
*
*/
int8_t bhy_initialize_support(void)
{
    uint8_t tmp_retry = RETRY_NUM;

    bhy.bus_write   = &bhy_i2c_write;
    bhy.bus_read    = &bhy_i2c_read;
    bhy.delay_msec  = &bhy_delay_msec;
    bhy.device_addr = BHY_I2C_SLAVE_ADDRESS;

    bhy_init(&bhy);

    bhy_reset();

    while(tmp_retry--)
    {
        bhy_get_product_id(&bhy.product_id);

        if(PRODUCT_ID_7183 == bhy.product_id)
        {
            return BHY_SUCCESS;
        }

        bhy_delay_msec(BHY_PARAMETER_ACK_DELAY);
    }

    return BHY_PRODUCT_ID_ERROR;
}

/*!
* @brief        Initiates a delay of the length of the argument in milliseconds
*
* @param[in]    msec    Delay length in terms of milliseconds
*
*/
void bhy_delay_msec(uint32_t msec)
{
    usleep(msec * 1000);
}

/*!
* @brief        Resets the BHY chip
*
*/
void bhy_reset(void)
{
    bhy_set_reset_request(BHY_RESET_ENABLE);
}

/** @}*/
