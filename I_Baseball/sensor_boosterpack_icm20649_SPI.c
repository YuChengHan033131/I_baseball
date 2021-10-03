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
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <semaphore.h>
#include <mqueue.h>
#include <pthread.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/SPI.h>
#include <ti/display/Display.h>
#include <Profile/baseball6xs_service.h>
#include <sensor_boosterpack_icm20649_SPI.h>
#include <sensor_driver/icm20649_SPI.h>
#include "flashctrl.h"
#include "BLEsend.h"
#include "sensor_configuration.h"
#include "Board.h"
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>
#include <time.h>
#include <ti/drivers/Timer.h>
#include <sensor_boosterpack_A301.h>

/*******************************************************************************
 *                             VARIABLES
 ******************************************************************************/
/* Task setup */
pthread_t icm20649Task;
pthread_t icmSensorTask;
static pthread_mutex_t lock;
static pthread_cond_t cond;
static volatile bool sampleData;
static Timer_Handle timer1;

/* Sensor Parameters */
extern volatile bool Baseball3xs;
extern volatile bool Baseball6xs;
extern volatile bool store_en;

//static uint8_t sensorConfig;
//static uint16_t sensorPeriod;

/* Sensor Objects */
//extern I2C_Handle i2cHandle;
static SPI_Handle      spiHandle;
extern Display_Handle displayOut;
static sem_t icm20649Sem;
static sem_t publishDataSem;
static uint8_t sendData[16] = {0x26,0x49,0};
static uint8_t sendData_a[10] = {0x26,0x4a,0};
static uint8_t sendData_g[10] = {0x26,0x4b,0};
static bool div_en = false;
extern unsigned char data_acc[7];
extern unsigned char data_gyro[7];
bool time_par = false;
uint32_t cnt = 0;
uint16_t cnt_gyo = 0;
uint32_t mod_cnt = 0;


/*******************************************************************************
 *                                  LOCAL FUNCTIONS
 ******************************************************************************/
static void *movementTaskFxn(void *arg0);
static void icm20649Setup(void);
static void *icmInterruptHandlerTask(void *arg0);
static void icm20649Callback(uint_least8_t index);
static void ICM_timerHandler(Timer_Handle handle);
static void ICM_timerTask(uint16_t period);

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

    printf("creatTask\n");

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
  //  Display_print0(displayOut, 0, 0, "SensorICM20649_Deactivate!");
    /* Deactivate task */
    pthread_mutex_lock(&lock);
    sampleData = false;
    /* Disable the ICM20649 sensor */
    sensorICM20649_enable(0x3F);
    pthread_mutex_unlock(&lock);
    Timer_close(timer1);
    time_par = false;
    cnt = 0;
    cnt_gyo = 0;
}

void SensorICM20649_Activate(bool div)
{
    /* Activate task */
    if(!store_en){
    ICM_timerTask(6222); // 1/160.7
    sensorICM20649_accSetSMPLRT_DIV(6);
    SensorICM20649_accSetRange(ICM_ACC1K_BW57);
    sensorICM20649_gyroSetSMPLRT_DIV(6);
    SensorICM20649_gyroSetRange(ICM_GYRO1K_BW57);
    }
    else if(div == false){
    ICM_timerTask(888);  // 1/1125 = 0.000888
    icm20649Setup();
    }
    else{
    ICM_timerTask(222);  // 1/4500 = 0.000222
    SensorICM20649_accSetRange(ICM_ACC4K);
    SensorICM20649_gyroSetRange(ICM_GYRO1K_BW57);
    }

    div_en = div;
    pthread_mutex_lock(&lock);
    sampleData = true;
    /* Enable the ICM20649 sensor */
    sensorICM20649_enable(0);
    SensorICM20649_enable_int();
    pthread_cond_signal(&cond);
    pthread_mutex_unlock(&lock);

  //  Display_print0(displayOut, 0, 0, "SensorICM20649_Activate!");
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


    /* Initialize characteristics and sensor driver */
    //sensorPeriod = ICM20649_DEFAULT_PERIOD;


    success = SensorICM20649_init();
  //  if(success) Display_printf(displayOut, 0, 0, " ICM20649 initializing SPI success");



    /* Setting GPIO Callbacks */
    GPIO_setCallback(ICM20649_INT3, &icm20649Callback);
    GPIO_clearInt(ICM20649_INT3);
    GPIO_enableInt(ICM20649_INT3);

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
  //  SensorICM20649_enable_int();
    //Display_print0(displayOut, 0, 0, "ICM20649 interrupt enable success");

    /* Set the sample rate divider of the accelerometer */
    sensorICM20649_accSetSMPLRT_DIV(0);
    /* Set the DLPF Bandwidth of the accelerometer */
  //  sensorICM20649_accSetBW(ACC_BW_5_7);
    /* Set the range of the accelerometer */
  //  SensorICM20649_accSetsample(0);
    SensorICM20649_accSetRange(ICM_ACC1K_BW57);
  //  SensorICM20649_accSetSample(0);



    /* Set the sample rate divider of the gyroscope */
    sensorICM20649_gyroSetSMPLRT_DIV(0);
    /* Set the DLPF Bandwidth of the gyroscope */
  //  sensorICM20649_gyroSetBW(GYRO_BW_5_7);
    /* Set the range of the gyroscope */
  //  SensorICM20649_gyroSetsample(0);
    SensorICM20649_gyroSetRange(ICM_GYRO1K_BW57);
  //  SensorICM20649_accSetSample(0);




   // Display_print0(displayOut, 0, 0, "ICM20649 setup success");
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
  //  uint8_t data_acc[6] = {0};
 //   uint8_t data_gyro[6] = {0};

    uint32_t cnt_mov = 0;
    uint32_t mod_cnt_mov = 0;
    uint8_t i;
    /* Initialize the task */
    movementTaskInit();



    sampleData = false;

    sensorICM20649_enable(0);

 //   icm20649Setup();

    while(1)
    {
        pthread_mutex_lock(&lock);
        while(!sampleData)
        {
            pthread_cond_wait(&cond, &lock);
        }
        pthread_mutex_unlock(&lock);


        if(!time_par){
        Timer_start(timer1);
        time_par = true;
        }


        // Waiting for data to become available from the sensor
        sem_wait(&publishDataSem);


        if(store_en  == true)
        {
            sendtoStore(sendData_a);
            if(mod_cnt_mov == 0 || !div_en ){
            sendtoStore(sendData_g);
            }
        }
        else
        {
            for(i=0;i<8;i++){
            sendData[i] = sendData_a[i];
            sendData[i+8] = sendData_g[i+2];
            }
            enqueue(sendData);
        }

        cnt_mov++;
        mod_cnt_mov = cnt_mov%4;
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
    uint8_t i;
 //   uint32_t cnt = 0;
  //  uint8_t data_acc[6] = {0};
  //  uint8_t data_gyro[6] = {0};
 //   uint16_t cnt_gyo = 0;
  //  uint32_t mod_cnt = 0;




    while (1)
    {
        sem_wait(&icm20649Sem);




        SensorICM20649_accRead(data_acc);

        for(ii = 0; ii < 6; ii++)
        {
            sendData_a[ii+2] = data_acc[ii+1];
        }

        if(mod_cnt == 0 || !div_en){
        SensorICM20649_gyroRead(data_gyro);

        for(ii = 0; ii < 6; ii++)
        {
            sendData_g[ii+2] = data_gyro[ii+1];
        }
        cnt_gyo++;

        sendData_g[8] = cnt_gyo>>8;
        sendData_g[9] = cnt_gyo;
        }


        cnt++;
        mod_cnt = cnt%4;

        //data_u8[0] - x->MSB
        //data_u8[1] - x->LSB
        //data_u8[2] - y->MSB
        //data_u8[3] - y->LSB
        //data_u8[4] - z->MSB
        //data_u8[5] - z->LSB
        /*
        for(ii = 0; ii < 6; ii++)
        {
            sendData[ii+2] = data_acc[ii+1];
            sendData[ii+8] = data_gyro[ii+1];
        }

        for(i=0;i<14;i++)
        {
        sendData[2+i] = cnt+i;
        }

        cnt = cnt + 14;
*/
    //    sendData[16] = cnt>>24;
    //    sendData[17] = cnt>>16;
    //    sendData[18] = cnt>>8;

        sendData_a[8] = cnt>>8;
        sendData_a[9] = cnt;
/*
        if(cnt == 4500 && store_en == true){
            SensorA301_Activate(false);
        }
*/

        uint32_t sensor_par;
        if(cnt == sensor_par+1 && store_en == true){

            SensorICM20649_Deactivate();
            Timer_close(timer1);
            time_par = false;
            //ble_act = 1;
          //  Display_printf(displayOut, 0, 0, "%d",cnt);
            Display_printf(displayOut, 0, 0, "SensorICM20649_Deactivate_SPI");
            cnt = 0;
            cnt_gyo = 0;
        //    SensorA301_Activate(false);

          //  outputflashdata();

     //   Display_printf(displayOut, 0, 0, "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x",
     //                        sendData[0],sendData[1],sendData[2],sendData[3],sendData[4],sendData[5],sendData[6],sendData[7],sendData[8],sendData[9],
      //                        sendData[10],sendData[11],sendData[12],sendData[13],sendData[14],sendData[15],sendData[16],sendData[17],sendData[18],sendData[19]);

     //   Display_printf(displayOut, 0, 0, "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x",
     //                         sendData[2],sendData[3],sendData[4],sendData[5],sendData[6],sendData[7],sendData[14],sendData[15],sendData[16],sendData[17],sendData[18],sendData[19]);
        }

        /*
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
*/
        sem_post(&publishDataSem);

   //     }

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
 * @fn      icm20649Callback
 *
 * @brief   Callback value that signifies a icm20649 reading has happened
 *
 * @param   index - Index of icm20649 Sensor
 *
 * @return  none
 ******************************************************************************/
static void icm20649Callback(uint_least8_t index)
{
    //Display_print0(displayOut, 0, 0, "Callback!");
  //  sem_post(&icm20649Sem);
}
static void ICM_timerHandler(Timer_Handle handle)
{
    /* Toggle a new conversion */
  //  Display_print0(displayOut, 0, 0, "TIMER1!");
    sem_post(&icm20649Sem);
}

static void ICM_timerTask(uint16_t period)
{

    Timer_Params params1;

    Timer_Params_init(&params1);
    params1.period = period;
    params1.periodUnits = TIMER_PERIOD_US;
    params1.timerMode = TIMER_CONTINUOUS_CB;
    params1.timerCallback = ICM_timerHandler;

    timer1 = Timer_open(Board_Timer1, &params1);
}

