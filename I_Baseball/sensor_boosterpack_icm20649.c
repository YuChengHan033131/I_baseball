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
#include <stdio.h>
#include <unistd.h>
#include <semaphore.h>
#include <mqueue.h>
#include <pthread.h>
#include <ti/drivers/I2C.h>
#include <ti/display/Display.h>
#include <Profile/baseball6xs_service.h>
#include <sensor_boosterpack_icm20649.h>
#include <sensor_driver/icm20649.h>
#include "flashctrl.h"
#include "BLEsend.h"
#include "sensor_configuration.h"
#include "Board.h"
//for timeStamp
#include <xdc/runtime/Timestamp.h>
#include <xdc/runtime/Types.h>

/*******************************************************************************
 *                             VARIABLES
 ******************************************************************************/
/* Task setup */
pthread_t icm20649Task;
pthread_t icmSensorTask;
static pthread_mutex_t lock;
static pthread_cond_t cond;
static volatile bool sampleData;

/* Sensor Parameters */
extern volatile bool Baseball3xs;
extern volatile bool Baseball6xs;
extern volatile bool store_en;

//static uint8_t sensorConfig;
//static uint16_t sensorPeriod;

/* Sensor Objects */
extern I2C_Handle i2cHandle;
extern Display_Handle displayOut;
static sem_t icm20649Sem;
static sem_t publishDataSem;
static uint8_t sendData[20] = {0x26,0x49,0};
//static uint8_t sendData_a[20] = {0x26,0x4a,0};
//static uint8_t sendData_g[20] = {0x26,0x4b,0};
static bool div_en = false;

/*TimeStamp*/
Types_Timestamp64 start_tick,tick;
xdc_runtime_Types_FreqHz freq;


/*******************************************************************************
 *                                  LOCAL FUNCTIONS
 ******************************************************************************/
static void *movementTaskFxn(void *arg0);
static void icm20649Setup(void);
static void *icmInterruptHandlerTask(void *arg0);
static void icm20649Callback(uint_least8_t index);

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
void SensorICM20649_createTask(void)
{
    pthread_attr_t pAttrs;
    struct sched_param priParam;
    int retc;
    int detachState;

    pthread_attr_init(&pAttrs);
    priParam.sched_priority = MOVEMENT_TASK_PRIORITY;

    detachState = PTHREAD_CREATE_DETACHED;
    retc = pthread_attr_setdetachstate(&pAttrs, detachState);

  //  printf("creatTask\n");

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

    retc = pthread_create(&icm20649Task, &pAttrs, movementTaskFxn, NULL);

    if (retc != 0)
    {
        while(1);
    }

    retc = pthread_create(&icmSensorTask, &pAttrs, icmInterruptHandlerTask, NULL);

    if (retc != 0)
    {
        while (1);
    }

    /* Initializing the semaphore */
    sem_init(&icm20649Sem,0,0);
    sem_init(&publishDataSem,0,0);
}

/*******************************************************************************
 * @fn      SensorBPMov_processCharChangeEvt
 *
 * @brief   SensorTag Movement event handling
 *
 ******************************************************************************/
void SensorICM20649_Deactivate(void)
{
  //  printf("icmclose\n");
    /* Deactivate task */
    pthread_mutex_lock(&lock);
    sampleData = false;
    /* Disable the ICM20649 sensor */
    sensorICM20649_enable(0x3F);
    pthread_mutex_unlock(&lock);
}

void SensorICM20649_Activate(bool div)
{
    /* Activate task */

   // printf("Active\n");
    pthread_mutex_lock(&lock);
    sampleData = true;
    div_en = div;
    /* Enable the ICM20649 sensor */
    sensorICM20649_enable(0);
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
    bool success;

 //   printf("Init\n");
    /* Initialize characteristics and sensor driver */
    //sensorPeriod = ICM20649_DEFAULT_PERIOD;
    success = SensorICM20649_init();
    if(success) Display_print0(displayOut, 0, 0, "ICM20649 initial success");

    /* Setting GPIO Callbacks */
    GPIO_setCallback(ICM20649_INT, &icm20649Callback);
    GPIO_clearInt(ICM20649_INT);
    GPIO_enableInt(ICM20649_INT);

    /* Initializing the mutex */
    pthread_mutex_init(&lock, NULL);
    pthread_cond_init(&cond, NULL);
}

/*******************************************************************************
 * @fn      icm2649Setup
 *
 * @brief   Set the parameter for sensor icm20649
 *
 ******************************************************************************/
static void icm20649Setup (void)
{
    /* Set the Interrupt if new data is ready */
    SensorICM20649_enable_int();
    //Display_print0(displayOut, 0, 0, "ICM20649 interrupt enable success");

    /* Set the sample rate divider of the accelerometer */
    sensorICM20649_accSetSMPLRT_DIV(0);
    /* Set the DLPF Bandwidth of the accelerometer */
    sensorICM20649_accSetBW(ACC_BW_5_7);
    /* Set the range of the accelerometer */
  //  SensorICM20649_accSetsample(0);
    SensorICM20649_accSetRange(ACC_RANGE_30G);
  //  SensorICM20649_accSetSample(0);



    /* Set the sample rate divider of the gyroscope */
    sensorICM20649_gyroSetSMPLRT_DIV(0);
    /* Set the DLPF Bandwidth of the gyroscope */
  sensorICM20649_gyroSetBW(GYRO_BW_5_7);
    /* Set the range of the gyroscope */
  //  SensorICM20649_gyroSetsample(0);
    SensorICM20649_gyroSetRange(GYRO_RANGE_4000DPS);
  //  SensorICM20649_accSetSample(0);



    //Display_print0(displayOut, 0, 0, "ICM20649 setup success");
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
  //  printf("movement\n");

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
        //Display_print0(displayOut, 0, 0, "ICM20649 enabled");

        icm20649Setup();

        /* Waiting for data to become available from the sensor */
        sem_wait(&publishDataSem);
        {

           enqueue(sendData);
           // printf("%x",sendData[0]);
            //sendtoStore(sendData);
        }

    }
}

/*******************************************************************************
 * @fn      icmInterruptHandlerTask
 *
 * @brief   The task loop of the sensor data processing task. This task is
 *          use to process the data from the ICM-20649's reg
 *
 * @return  none
 ******************************************************************************/
static void* icmInterruptHandlerTask(void *arg0)
{
    uint8_t ii;
    //uint8_t cnt = 2;
    uint8_t data_acc[6] = {0};
    uint8_t data_gyro[6] = {0};
    uint8_t counter = 0;

    while (1)
    {
     //   printf("hander\n");
        sem_wait(&icm20649Sem);
        //Display_print0(displayOut, 0, 0, "ICM20649 Interrup!");

        SensorICM20649_accRead(data_acc);
        SensorICM20649_gyroRead(data_gyro);

        //data_u8[0] - x->MSB
        //data_u8[1] - x->LSB
        //data_u8[2] - y->MSB
        //data_u8[3] - y->LSB
        //data_u8[4] - z->MSB
        //data_u8[5] - z->LSB
        for(ii = 0; ii < 6; ii++)
        {
            sendData[ii+2] = data_acc[ii];
            sendData[ii+8] = data_gyro[ii];
        }

        if(div_en)
        {
            if(counter == 0)
            {
                sem_post(&publishDataSem);
                counter++;
            }
            else
            {
                counter = 0;
            }
        }
        else
        {
            sem_post(&publishDataSem);
        }


        /*
        for(ii = 0; ii < 6; ii++)
        {
            sendData_a[cnt]     = data_acc[ii];
            sendData_g[cnt++]   = data_gyro[ii];
        }
        if(cnt == 20)
        {
            sem_post(&publishDataSem);
            cnt = 2;
        }
        */
    }
}

/*******************************************************************************
 * @fn      bmi160Callback
 *
 * @brief   Callback value that signifies a BMI160 reading has happened
 *
 * @param   index - Index of BMI160 Sensor
 *
 * @return  none
 ******************************************************************************/
static void icm20649Callback(uint_least8_t index)
{
    //printf("callback\n");
    sem_post(&icm20649Sem);

}
/*********************************************************************
 * for testing
 *
 *
 *
 */
 void test_SensorICM20649_createTask(void)
{
    pthread_attr_t pAttrs;
    struct sched_param priParam;
    int detachState;
    pthread_attr_init(&pAttrs);
    priParam.sched_priority = MOVEMENT_TASK_PRIORITY;
    detachState = PTHREAD_CREATE_DETACHED;
    pthread_attr_setdetachstate(&pAttrs, detachState);
    pthread_attr_setschedparam(&pAttrs, &priParam);
    pthread_attr_setstacksize(&pAttrs, MOVEMENT_TASK_STACK_SIZE);

    pthread_create(&icm20649Task, &pAttrs, test_icm2049_TaskFxn, NULL);

    /* Initializing the semaphore */
    sem_init(&icm20649Sem,0,0);
    //sem_init(&publishDataSem,0,0);
}
/*************************************************************
 * function use in test_SensorICM20649_createTask(void)
 */
 void* test_icm2049_TaskFxn(void *arg0){
     /* Initialize the task */
     movementTaskInit();

     uint8_t data[20]={0xbb,0xbb,0xbb,0xbb,0xbb,0};
     enqueue(data);
     uint8_t j;
     sleep(1);
     for(j=0;j<10;j=j+1){
          sleep(1);
          data[0]=j;
          enqueue(data);
     }
     static uint8_t sensordata[20] = {0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20};
     uint8_t data_acc[6] = {1};
     uint8_t data_gyro[6] = {1};
     uint8_t i;
     while (1)
     {
         //sleep(1);
         //set sample rate
         test_icm20649Setup(0);
         sem_wait(&icm20649Sem);
         if(!(SensorICM20649_accRead(data_acc)&&SensorICM20649_gyroRead(data_gyro))){
             //if one of them failed
             static uint8_t error[20] = {0xee,0xee,0};
             enqueue(error);
         }

         for(i = 0; i < 6; i=i+1)
         {
             sensordata[i+2] = data_acc[i];
             sensordata[i+8] = data_gyro[i];
         }
         enqueue(sensordata);

     }

 }
/*************************************************************************
 * setting sensor sample rate ,digital low pass filter bandwidth, range
 *
 */
 void test_icm20649Setup (uint8_t sampleRateDivider)
 {
     /* Set the Interrupt if new data is ready */
     SensorICM20649_enable_int();
     //Display_print0(displayOut, 0, 0, "ICM20649 interrupt enable success");

     /* Set the sample rate divider of the accelerometer */
     sensorICM20649_accSetSMPLRT_DIV(sampleRateDivider);
     /* Set the DLPF Bandwidth of the accelerometer */
     sensorICM20649_accSetBW(ACC_BW_5_7);
     /* Set the range of the accelerometer */
   //  SensorICM20649_accSetsample(0);
     SensorICM20649_accSetRange(ACC_RANGE_30G);
   //  SensorICM20649_accSetSample(0);



     /* Set the sample rate divider of the gyroscope */
     sensorICM20649_gyroSetSMPLRT_DIV(sampleRateDivider);
     /* Set the DLPF Bandwidth of the gyroscope */
   sensorICM20649_gyroSetBW(GYRO_BW_5_7);
     /* Set the range of the gyroscope */
   //  SensorICM20649_gyroSetsample(0);
     SensorICM20649_gyroSetRange(GYRO_RANGE_4000DPS);
   //  SensorICM20649_accSetSample(0);



     //Display_print0(displayOut, 0, 0, "ICM20649 setup success");
 }
