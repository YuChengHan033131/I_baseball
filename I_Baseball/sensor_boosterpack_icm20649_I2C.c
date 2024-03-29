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
#define DISPLAY_DISABLE
#include <Profile/baseball6xs_service.h>
#include <sensor_boosterpack_icm20649_I2C.h>
#include <sensor_driver/icm20649_I2C.h>
#include "flashctrl.h"
#include "BLEsend.h"
#include "sensor_configuration.h"
#include "Board.h"
//for timeStamp
#include <xdc/runtime/Timestamp.h>
#include <xdc/runtime/Types.h>

#define NO_MOTION_THRESHOLD 1//i.e. +-0.92mg, threshold for sleep if no motion

/*******************************************************************************
 *                             VARIABLES
 ******************************************************************************/
/* GLOBAL variable*/
extern sem_t BLEconnected;
/* Task setup */
pthread_t icm20649Task_I2C;
pthread_t icmSensorTask_I2C;
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
xdc_runtime_Types_FreqHz freq;

extern bool BLEisConnected;
extern bool sensorOn;


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
void SensorICM20649_createTask_I2C(void)
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

    retc = pthread_create(&icm20649Task_I2C, &pAttrs, movementTaskFxn, NULL);

    if (retc != 0)
    {
        while(1);
    }

    retc = pthread_create(&icmSensorTask_I2C, &pAttrs, icmInterruptHandlerTask, NULL);

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
void SensorICM20649_Deactivate_I2C(void)
{
  //  printf("icmclose\n");
    /* Deactivate task */
    pthread_mutex_lock(&lock);
    /* Disable the ICM20649 sensor */
    sensorICM20649_enable_I2C(0x3F);
    pthread_mutex_unlock(&lock);
}

void SensorICM20649_Activate_I2C(void)
{
    /* Activate task */

   // printf("Active\n");
    pthread_mutex_lock(&lock);
    /* Enable the ICM20649 sensor */
    sensorICM20649_enable_I2C(0);
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
    success = SensorICM20649_init_I2C();
#ifndef DISPLAY_DISABLE
    if(success) Display_print0(displayOut, 0, 0, "ICM20649 initial success");
#endif

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
    SensorICM20649_enable_int_I2C();
#ifndef DISPLAY_DISABLE
    Display_print0(displayOut, 0, 0, "ICM20649 interrupt enable success");
#endif
    /* Set the sample rate divider of the accelerometer */
    sensorICM20649_accSetSMPLRT_DIV_I2C(0);
    /* Set the DLPF Bandwidth of the accelerometer */
    sensorICM20649_accSetBW_I2C(ACC_BW_5_7);
    /* Set the range of the accelerometer */
  //  SensorICM20649_accSetsample(0);
    SensorICM20649_accSetRange_I2C(ACC_RANGE_30G);
  //  SensorICM20649_accSetSample(0);



    /* Set the sample rate divider of the gyroscope */
    sensorICM20649_gyroSetSMPLRT_DIV_I2C(0);
    /* Set the DLPF Bandwidth of the gyroscope */
  sensorICM20649_gyroSetBW_I2C(GYRO_BW_5_7);
    /* Set the range of the gyroscope */
  //  SensorICM20649_gyroSetsample(0);
    SensorICM20649_gyroSetRange_I2C(GYRO_RANGE_4000DPS);
  //  SensorICM20649_accSetSample(0);


#ifndef DISPLAY_DISABLE
    Display_print0(displayOut, 0, 0, "ICM20649 setup success");
#endif
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
#ifndef DISPLAY_DISABLE
        Display_print0(displayOut, 0, 0, "ICM20649 enabled");
#endif

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
#ifndef DISPLAY_DISABLE
        Display_print0(displayOut, 0, 0, "ICM20649 Interrup!");
#endif

        SensorICM20649_accRead_I2C(data_acc);
        SensorICM20649_gyroRead_I2C(data_gyro);

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
    sem_post(&icm20649Sem);
}
/*********************************************************************
 * for testing*/
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

    pthread_create(&icm20649Task_I2C, &pAttrs, test_icm2049_TaskFxn, NULL);

    /* Initializing the semaphore */
    sem_init(&icm20649Sem,0,0);

    /* Initialize mutex*/


}
/*************************************************************
 * function use in test_SensorICM20649_createTask(void)
 */
 void* test_icm2049_TaskFxn(void *arg0){
     Display_printf(displayOut,0,0,"icm taskFxn start");

     /*general variables*/
     static uint8_t sensordata[20] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
     uint16_t i, j;
     uint8_t seqNum=0;
     uint8_t data;

     /*variable of sleep if no motion*/
     bool sample = true;
     bool smaller = true;
     uint16_t cnt=0;
     int16_t current_data;
     int32_t diff_data;//32 bytes in case of overflow in difference
     int16_t previous_data[3]={0,0,0};

     /* Initialize the task */
     movementTaskInit(); //include initialize icm20649 register, sleep mode
     SensorICM20649_Activate_I2C();//leave sleep mode & activate acc, gyro

     uint8_t sampleRateDivider=0;
     /* Set the sample rate divider of the accelerometer */
     sensorICM20649_accSetSMPLRT_DIV_I2C(sampleRateDivider);
     /* Set the DLPF Bandwidth of the accelerometer */
     sensorICM20649_accSetBW_I2C(ACC_BW_5_7);
     /* Set the range of the accelerometer */
     SensorICM20649_accSetRange_I2C(ACC_RANGE_30G);
     /* Set the sample rate divider of the gyroscope */
     sensorICM20649_gyroSetSMPLRT_DIV_I2C(sampleRateDivider);
     /* Set the DLPF Bandwidth of the gyroscope */
     sensorICM20649_gyroSetBW_I2C(GYRO_BW_5_7);
     /* Set the range of the gyroscope */
     SensorICM20649_gyroSetRange_I2C(GYRO_RANGE_4000DPS);
     /* The accel sensor needs max 30ms, the gyro max 35ms to fully start */
     usleep(50000);

     /*enable FIFO*/
     writeReg_I2C(REG_BANK_SEL_I2C, BANK_0_I2C);
     writeReg_I2C(USER_CTRL_I2C, 0x40);

     //wake-on motion threshold
     writeReg_I2C(REG_BANK_SEL_I2C, BANK_2_I2C);
     writeReg_I2C(ACCEL_WOM_THR_I2C, 0x04);//LSB=4mg ,range=0~1020mg, i.e. 16mg

     while(1){
         //enable wake-on motion interrupt
         writeReg_I2C(REG_BANK_SEL_I2C, BANK_0_I2C);
         writeReg_I2C(INT_ENABLE_I2C, 0x08);

         //enable wake-on motion logic & setting
         writeReg_I2C(REG_BANK_SEL_I2C, BANK_2_I2C);
         writeReg_I2C(ACCEL_INTEL_CTRL_I2C, 0x03);//logic on, setting compare to previous value

         //turn on low power mode
         writeReg_I2C(REG_BANK_SEL_I2C, BANK_0_I2C);
         readReg_I2C(PWR_MGMT_1_I2C,&data,1);
         data |= 0x20;
         writeReg_I2C(PWR_MGMT_1_I2C, data);

         do{
             Display_printf(displayOut,0,0,"sleeping");
             if(!sensorOn){
                 sleep(5);
             }
             sem_wait(&icm20649Sem);//wait for wake-on motion interrupt
             Display_printf(displayOut,0,0,"wake up");
             /*reset value in semaphore*///since may have multiple WOM interrupt
              sem_getvalue(&icm20649Sem,&data);
              for(i=0;i<data;i++){
                  sem_wait(&icm20649Sem);
              }
         }while(!sensorOn);//back to sleeping state if sensor is turn off by BLE command

         //test: see interrupt status
         /*writeReg_I2C(REG_BANK_SEL, BANK_0);
         readReg_I2C(INT_STATUS, &data,1);*/

         //disable wake-on motion interrupt
         writeReg_I2C(REG_BANK_SEL_I2C, BANK_0_I2C);
         writeReg_I2C(INT_ENABLE_I2C, 0x00);
         //disable wake-on motion logic
         writeReg_I2C(REG_BANK_SEL_I2C, BANK_2_I2C);
         writeReg_I2C(ACCEL_INTEL_CTRL_I2C, 0x00);

         //turn off low power mode
         writeReg_I2C(REG_BANK_SEL_I2C, BANK_0_I2C);
         readReg_I2C(PWR_MGMT_1_I2C,&data,1);
         data &= 0xDF;
         writeReg_I2C(PWR_MGMT_1_I2C,data);

         /*reset value in semaphore*///since may have multiple WOM interrupt
         sem_getvalue(&icm20649Sem,&data);
         for(i=0;i<data;i++){
             sem_wait(&icm20649Sem);
         }

         openflash();

         /*reset FIFO*/
         writeReg_I2C(REG_BANK_SEL_I2C, BANK_0_I2C);
         writeReg_I2C(FIFO_RST_I2C, 0x0f);//not sure which value can actually reset
         writeReg_I2C(REG_BANK_SEL_I2C, BANK_0_I2C);
         writeReg_I2C(FIFO_RST_I2C, 0x00);

         /*enable acc & gyro write to FIFO*/
         writeReg_I2C(REG_BANK_SEL_I2C, BANK_0_I2C);
         writeReg_I2C(FIFO_EN_2_I2C, 0x1e);//acc & gyr

         sample = true;
         while(sample){
             //enable FIFO water mark to interrupt 1
             writeReg_I2C(REG_BANK_SEL_I2C, BANK_0_I2C);
             writeReg_I2C(INT_ENABLE3_I2C, 0x01);

             sem_wait(&icm20649Sem);
             //test: disable data write after reach FIFO water mark
             /*disable acc & gyro write to FIFO*/
             // writeReg_I2C(REG_BANK_SEL, BANK_0);
             // writeReg_I2C(FIFO_EN_2, 0x00);

             for(j=0;j<2052;j=j+12){//2052=6 axis sensor*2 bytes*171 times of sample = at least amount of data in FIFO
                 /*useless
                  * read data number in FIFO
                writeReg_I2C(REG_BANK_SEL, BANK_0);
                readReg_I2C(FIFO_COUNT_H, &sensordata[16], 2);*/

                writeReg_I2C(REG_BANK_SEL_I2C, BANK_0_I2C);
                readReg_I2C(FIFO_R_W_I2C, &sensordata[0], 12);
                //order: acc x_H//acc x_L//acc y_H//acc y_L//acc z_H//acc z_L///gyr x_H//gyr x_L//gyr y_H;//gyr y_L//gyr z_H//gyr z_L
                //usleep(10000);
                //enqueue(sensordata);
                //add seqNum
                sensordata[12]=seqNum;
                if(seqNum==255){
                    seqNum=0;
                }else{
                    seqNum+=1;
                }
                sendtoStore(sensordata);
                //Display_printf(displayOut,0,0,"in:%d",sensordata[14]*256+sensordata[15]);//adding this will cause FIFO overflow

                /*end data collection if no motion for 10 second
                 * algorithm: difference between previous sensordata < threshold*/
                smaller = true;
                for(i=0;i<3;i++){
                    //turn sensordata from uint8_t*2 to int16_t
                    current_data = sensordata[i*2]*256+sensordata[i*2+1];
                    //Display_printf(displayOut,0,0,"sensordata:%d",sensordata[i*2]*256+sensordata[i*2+1]);
                    //Display_printf(displayOut,0,0,"current_data:%d",current_data);
                    //get difference
                    diff_data=current_data-previous_data[i];
                    //Display_printf(displayOut,0,0,"diff_data:%d",diff_data);

                    //store current data
                    previous_data[i]=current_data;

                    //compare difference with threshold
                    if(diff_data > NO_MOTION_THRESHOLD || diff_data < -NO_MOTION_THRESHOLD){
                        //larger than threshold
                        smaller =false;
                        break;
                    }

                }
                if(smaller){
                    cnt+=1;
                }else{
                    cnt = 0;
                }
                //sample rate *5 second or sensor turn off by BLE command
                if(cnt>1125/(1+sampleRateDivider)*5||!sensorOn){
                    sample = false;
                    Display_printf(displayOut,0,0,"sleep");
                    break;
                }
             }
             //Display_printf(displayOut,0,0,"cnt:%d",cnt);
         }

         closeflash();
         Display_printf(displayOut,0,0,"done data collection");

         /*reset value in semaphore*///ignore data left in FIFO
         sem_getvalue(&icm20649Sem,&data);
         for(i=0;i<data;i++){
             sem_wait(&icm20649Sem);
         }

         /*disable acc & gyro write to FIFO*/
         writeReg_I2C(REG_BANK_SEL_I2C, BANK_0_I2C);
         writeReg_I2C(FIFO_EN_2_I2C, 0x00);//acc & gyr

         //sem_wait(&BLEconnected);
         //send all of the flash data through BLE
         //outputflashdata((total_set_number()-1));//test, actually should be in AP_task
         //Display_printf(displayOut,0,0,"done data output");
     }

     Display_printf(displayOut,0,0,"end");


 }

