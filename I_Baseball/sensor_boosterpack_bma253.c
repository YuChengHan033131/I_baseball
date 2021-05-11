/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
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
 * --/COPYRIGHT--*/
/*******************************************************************************
 *                                INCLUDES
 ******************************************************************************/
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <semaphore.h>
#include <mqueue.h>
#include <pthread.h>
#include <ti/drivers/I2C.h>
#include <ti/display/Display.h>
#include <Profile/baseball3xs_service.h>
#include <sensor_boosterpack_bma253.h>
#include "sensor_driver/bma2x2.h"
#include "sensor_configuration.h"
#include "Board.h"
#include "BLEsend.h"
#include "flashctrl.h"

/*******************************************************************************
 *                             VARIABLES
 ******************************************************************************/
/* Task setup */
pthread_t bma253Task;
pthread_t bmaSensorTask;
static pthread_mutex_t lock;
static pthread_cond_t cond;
static volatile bool sampleData;

/* Sensor Parameters */
extern volatile bool Baseball3xs;
extern volatile bool Baseball6xs;
extern volatile bool store_en;

static uint8_t sensorConfig;
static uint16_t sensorPeriod;


/* Sensor Objects */
extern I2C_Handle i2cHandle;
extern Display_Handle displayOut;
static struct bma2x2_t bma2x2;
static BMA2x2_RETURN_FUNCTION_TYPE com_rslt;
static sem_t bma253Sem;
static sem_t publishDataSem;
static struct bma2x2_accel_data accelxyz_mov = {0, 0, 0};
static uint8_t sendData[BASEBALL3xs_DATA_LEN] = {0x25,0x3F,0};

/*******************************************************************************
 *                                  LOCAL FUNCTIONS
 ******************************************************************************/
static int8_t BMA2x2_I2C_bus_write(uint8_t dev_addr, uint8_t reg_addr,
                                   uint8_t *reg_data, uint8_t cnt);
static int8_t BMA2x2_I2C_bus_read(uint8_t dev_addr, uint8_t reg_addr,
                           uint8_t *reg_data, uint8_t cnt);
static void BMA2x2_delay_msek(uint32_t msek);
static bool i2cTransferFxn(I2C_Handle handle, I2C_Transaction transaction);
static void *movementTaskFxn(void *arg0);
static void *bmaInterruptHandlerTask(void *arg0);
static void bma253Callback(uint_least8_t index);

/*******************************************************************************
 *                                   FUNCTIONS
 ******************************************************************************/

/*******************************************************************************
 * @fn      SensorBPMov_createTask
 *
 * @brief   Task creation function for the movement task
 *
 * @param   none
 *
 * @return  none
 ******************************************************************************/
void SensorBMA253_createTask(void)
{
    pthread_attr_t pAttrs;
    struct sched_param priParam;
    int retc;
    int detachState;

    pthread_attr_init(&pAttrs);
    priParam.sched_priority = MOVEMENT_TASK_PRIORITY;

    detachState = PTHREAD_CREATE_DETACHED;
    retc = pthread_attr_setdetachstate(&pAttrs, detachState);

    if (retc != 0)
    {
        while(1);
    }

    pthread_attr_setschedparam(&pAttrs, &priParam);

    retc |= pthread_attr_setstacksize(&pAttrs, MOVEMENT_TASK_STACK_SIZE);

    if (retc != 0)
    {
        while(1);
    }

    retc = pthread_create(&bma253Task, &pAttrs, movementTaskFxn, NULL);

    if (retc != 0)
    {
        while(1);
    }

    retc = pthread_create(&bmaSensorTask, &pAttrs, bmaInterruptHandlerTask, NULL);

    if (retc != 0)
    {
        while (1);
    }

    /* Initializing the semaphore */
    sem_init(&bma253Sem,0,0);
    sem_init(&publishDataSem,0,0);
}

/*******************************************************************************
 * @fn      SensorBPMov_processCharChangeEvt
 *
 * @brief   SensorTag Movement event handling
 *
 ******************************************************************************/
void SensorBMA253_Deactivate(void)
{
    /* Deactivate task */
    pthread_mutex_lock(&lock);
    sampleData = false;
    /* Setting up the sensor */
    bma2x2_set_power_mode(BMA2x2_MODE_DEEP_SUSPEND);
    pthread_mutex_unlock(&lock);
}

void SensorBMA253_Activate(void)
{
    /* Activate task */
    pthread_mutex_lock(&lock);
    /* Setting up the sensor */
    bma2x2_soft_rst();
    usleep(1000);//wait 1ms
    sampleData = true;
    pthread_cond_signal(&cond);
    pthread_mutex_unlock(&lock);
}
/*******************************************************************************
 * @fn      movementTaskInit
 *
 * @brief   Initialization function for the SensorTag humidity sensor
 *
 ******************************************************************************/
static void movementTaskInit(void)
{
    /* Initialize characteristics and sensor driver */
    sensorPeriod = MOVBMA253_DEFAULT_PERIOD;

    com_rslt = BMA2x2_INIT_VALUE;
    bma2x2.bus_write = BMA2x2_I2C_bus_write;
    bma2x2.bus_read = BMA2x2_I2C_bus_read;
    bma2x2.delay_msec = BMA2x2_delay_msek;
    bma2x2.dev_addr = BMA2x2_I2C_ADDR1;

    com_rslt += bma2x2_init(&bma2x2);

    bma2x2_set_power_mode(BMA2x2_MODE_DEEP_SUSPEND);

    /* Setting GPIO Callbacks */
    GPIO_setCallback(BMA253_INT, &bma253Callback);
    GPIO_clearInt(BMA253_INT);
    GPIO_enableInt(BMA253_INT);

    /* Initializing the mutex */
    pthread_mutex_init(&lock, NULL);
    pthread_cond_init(&cond, NULL);

    Display_print0(displayOut, 0, 0, "BMA253 initial success");
}

/*******************************************************************************
 * @fn      movementTaskFxn
 *
 * @brief   The task loop of the movement readout task
 *
 * @return  none
 ******************************************************************************/
static void *movementTaskFxn(void *arg0)
{

    /* Initialize the task */
    movementTaskInit();

    /* Deactivate task (active only when measurement is enabled) */
    sampleData = false;

    while(1)
    {
        pthread_mutex_lock(&lock);
        while(!sampleData)
        {
            pthread_cond_wait(&cond, &lock);
        }
        pthread_mutex_unlock(&lock);
        //Display_print0(displayOut, 0, 0, "BMA253 enabled");

        bma2x2_set_bw(BMA2x2_BW_250HZ);//0x10 PMU_BW update time = 2ms
        bma2x2_set_range(BMA2x2_RANGE_4G);

        /* Setup Interrupt */
        bma2x2_set_new_data(BMA2x2_ACCEL_INTR1_NEWDATA,INTR_ENABLE);
        bma2x2_set_intr_enable(BMA2x2_DATA_ENABLE,INTR_ENABLE);

        /* Waiting for data to become available from the sensor */
        sem_wait(&publishDataSem);
        enqueue(sendData);
        /*
        if(store_en == true)
        {
            sendtoStore(sendData);
        }
        */
    }
}

/*******************************************************************************
 * @fn      bmaInterruptHandlerTask
 *
 * @brief   The task loop of the sensor data processing task. This task is
 *          use to process the data from the BMI160's FIFO
 *
 * @return  none
 ******************************************************************************/
static void *bmaInterruptHandlerTask(void *arg0)
{
    uint8_t sensorData[6] = {0};
    uint8_t cnt = 0;

    while (1)
    {
        sem_wait(&bma253Sem);
        //Display_print0(displayOut, 0, 0, "BMA253 Interrup!");

        if(bma2x2_read_accel_xyz(&accelxyz_mov) == BMA2x2_INIT_VALUE)
        {
            //Display_printf(displayOut, 0, 0, "get acc data!!!!");
            //data_u8[0] - x->LSB
            //data_u8[1] - x->MSB
            //data_u8[2] - y->LSB
            //data_u8[3] - y->MSB
            //data_u8[4] - z->LSB
            //data_u8[5] - z->MSB

            memcpy(sensorData, &accelxyz_mov, 6);

            if(cnt==2 || cnt==11)
            {
                sendData[cnt++] = ((sensorData[1]&(0x0F))<<4) | ((sensorData[0]&(0xF0))>>4);
                sendData[cnt++] = ((sensorData[0]&(0x0F))<<4) | (sensorData[3]&(0x0F));
                sendData[cnt++] = sensorData[2];
                sendData[cnt++] = ((sensorData[5]&(0x0F))<<4) | ((sensorData[4]&(0xF0))>>4);
                sendData[cnt] = (sensorData[5]&(0x0F))<<4;
            }
            else
            {
                sendData[cnt++] = sendData[cnt] | (sensorData[1]&(0x0F));
                sendData[cnt++] = sensorData[0];
                sendData[cnt++] = ((sensorData[3]&(0x0F))<<4) | ((sensorData[2]&(0xF0))>>4);
                sendData[cnt++] = ((sensorData[2]&(0x0F))<<4) | (sensorData[5]&(0x0F));
                sendData[cnt++] = sensorData[4];
            }
        }
        if(cnt == 3){
            //Display_print0(displayOut, 0, 0, "BMA253 cnt = 3");
            cnt = 0;
            sem_post(&publishDataSem);
        }
    }
}

/*******************************************************************************
 * @fn      bma253Callback
 *
 * @brief   Callback value that signifies a BMI160 reading has happened
 *
 * @param   index - Index of BMI160 Sensor
 *
 * @return  none
 ******************************************************************************/
static void bma253Callback(uint_least8_t index)
{
    sem_post(&bma253Sem);
}

/*
 * Sensor interface functions. These functions are used to interact with the
 * SAIL BMA253 module and use for generic I2C write/reads. You shouldn't have
 * to edit these.
 */
static int8_t BMA2x2_I2C_bus_write(uint8_t dev_addr, uint8_t reg_addr,
                                   uint8_t *reg_data, uint8_t cnt)
{
    int32_t iError = BMA2x2_INIT_VALUE;
    uint8_t array[I2C_BUFFER_LEN];
    uint8_t stringpos = BMA2x2_INIT_VALUE;
    I2C_Transaction i2cTransaction;

    array[BMA2x2_INIT_VALUE] = reg_addr;
    for (stringpos = BMA2x2_INIT_VALUE; stringpos < cnt; stringpos++) {
        array[stringpos + BMA2x2_BUS_READ_WRITE_ARRAY_INDEX] =
        *(reg_data + stringpos);
    }

    i2cTransaction.writeBuf = array;
    i2cTransaction.writeCount = cnt + 1;
    i2cTransaction.readCount = 0;
    i2cTransaction.slaveAddress = dev_addr;

    //Perform I2C read
    if (!i2cTransferFxn(i2cHandle, i2cTransaction))
    {
        iError = -1; /* In case I2C transfer operation fails and returns false*/
    }

    return (int8_t)iError;
}

static int8_t BMA2x2_I2C_bus_read(uint8_t dev_addr, uint8_t reg_addr,
                                  uint8_t *reg_data, uint8_t cnt)
{
    int32_t iError = BMA2x2_INIT_VALUE;
    I2C_Transaction i2cTransaction;

    //Point I2C parameters to correct values. Set up to read "cnt" number of bytes.
    i2cTransaction.writeBuf = &reg_addr;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = reg_data;
    i2cTransaction.readCount = cnt;
    i2cTransaction.slaveAddress = dev_addr;

    //Perform I2C read
    if (!i2cTransferFxn(i2cHandle, i2cTransaction))
    {
        iError = -1; /* In case I2C transfer operation fails and returns false*/
    }

    return (uint8_t)iError;
}

static void BMA2x2_delay_msek(uint32_t msek)
{
    usleep(msek * 1000);
}

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
