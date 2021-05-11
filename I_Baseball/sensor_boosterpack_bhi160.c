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
#include <Profile/baseball6xs_service.h>
#include <sensor_boosterpack_bhi160.h>
#include "sensor_driver/bhi160/BHIfw.h"
#include "sensor_driver/bhi160/bhy.h"
#include "sensor_driver/bhi160/bhy_uc_driver.h"
#include "sensor_driver/bhi160/bhy_uc_driver_types.h"
#include "sensor_configuration.h"
#include "Board.h"
#include "BLEsend.h"
#include "flashctrl.h"

/*******************************************************************************
 *                             VARIABLES
 ******************************************************************************/
/* Task setup */
pthread_t bhi160Task;
pthread_t bhiSensorTask;
static pthread_mutex_t lock;
static pthread_cond_t cond;
static volatile bool sampleData;

/* Sensor Parameters */
extern volatile bool Baseball3xs;
extern volatile bool Baseball6xs;
extern volatile bool store_en;
/*
static uint8_t sensorConfig;
static uint16_t sensorPeriod;
*/

/* Sensor Objects */
extern Display_Handle displayOut;
struct bhy_t bhy;
static BHY_RETURN_FUNCTION_TYPE com_rslt;
static sem_t bhi160Sem;
static sem_t publishDataSem;
static uint8_t sensorData[BASEBALL6xs_DATA_LEN] = {0x16,0x0F,0};

/*******************************************************************************
 *                                  LOCAL FUNCTIONS
 ******************************************************************************/
static void *movementTaskFxn(void *arg0);
static void *bhiInterruptHandlerTask(void *arg0);
static void bhi160Callback(uint_least8_t index);

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
void SensorBHI160_createTask(void)
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

    retc = pthread_create(&bhi160Task, &pAttrs, movementTaskFxn, NULL);

    if (retc != 0)
    {
        while (1);
    }

    retc = pthread_create(&bhiSensorTask, &pAttrs, bhiInterruptHandlerTask, NULL);

    if (retc != 0)
    {
        while (1);
    }

    /* Initializing the semaphore */
    sem_init(&bhi160Sem,0,0);
    sem_init(&publishDataSem,0,0);
}

/*******************************************************************************
 * @fn      SensorBPMov_processCharChangeEvt
 *
 * @brief   SensorTag Movement event handling
 *
 ******************************************************************************/
void SensorBHI160_Deactivate(void)
{
    /* Deactivate task */
    //com_rslt += bhy_disable_virtual_sensor(VS_TYPE_ACCELEROMETER,VS_NON_WAKEUP);
    com_rslt += bhy_disable_virtual_sensor(VS_TYPE_LINEAR_ACCELERATION,VS_NON_WAKEUP);
    com_rslt += bhy_disable_virtual_sensor(VS_TYPE_GYROSCOPE,VS_NON_WAKEUP);
    com_rslt += bhy_disable_virtual_sensor(VS_TYPE_GEOMAGNETIC_FIELD,VS_NON_WAKEUP);
    pthread_mutex_lock(&lock);
    sampleData = false;
    pthread_mutex_unlock(&lock);
}

void SensorBHI160_Activate(void)
{
    /* Activate task */
    //com_rslt += bhy_enable_virtual_sensor(VS_TYPE_ACCELEROMETER, VS_NON_WAKEUP, 200, 0, VS_FLUSH_NONE, 0, 16);//dynamic range default is 4g
    com_rslt += bhy_enable_virtual_sensor(VS_TYPE_LINEAR_ACCELERATION, VS_NON_WAKEUP, 200, 0, VS_FLUSH_NONE, 0, 16);
    com_rslt += bhy_enable_virtual_sensor(VS_TYPE_GYROSCOPE, VS_NON_WAKEUP, 200, 0, VS_FLUSH_NONE, 0, 2000);
    com_rslt += bhy_enable_virtual_sensor(VS_TYPE_GEOMAGNETIC_FIELD, VS_NON_WAKEUP, 200, 0, VS_FLUSH_NONE, 0, 0);//VS_ID_MAGNETOMETER
    pthread_mutex_lock(&lock);
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
    /* Initializing the sensor */
    com_rslt = BHY_INIT_VALUE;
    com_rslt += bhy_driver_init(_bhi_fw);
    usleep(50000);
    if(com_rslt == BHY_SUCCESS)
        Display_printf(displayOut, 0, 0, "bhi160 flash firmware init!");
    else
        Display_printf(displayOut, 0, 0, "bhi160 flash firmware fail!");

    /* Setting GPIO Callbacks */
    GPIO_setCallback(BHI160_INT, &bhi160Callback);
    GPIO_clearInt(BHI160_INT);
    GPIO_enableInt(BHI160_INT);

    /* Initializing the mutex */
    pthread_mutex_init(&lock, NULL);
    pthread_cond_init(&cond, NULL);
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
        /* Waiting for data to become available from the sensor */
        sem_wait(&publishDataSem);
        if(store_en == true)
        {
            sendtoStore(sensorData);
        }
        enqueue(sensorData);
    }
}

/*******************************************************************************
 * @fn      bhiInterruptHandlerTask
 *
 * @brief   The task loop of the sensor data processing task. This task is
 *          use to process the data from the BMI160's FIFO
 *
 * @return  none
 ******************************************************************************/
void* bhiInterruptHandlerTask(void *arg0)
{
    uint8_t array[69] = {0};
    uint8_t *fifoptr  = NULL;
    uint8_t accel[6]  = {0};
    uint8_t gyro[6]   = {0};
    uint8_t mag[6]    = {0};

    uint8_t  bytes_left_in_fifo = 0;
    uint16_t bytes_remaining    = 0; //the bytes left in fifo which have not been read
    uint16_t bytes_read         = 0; //the bytes in fifo which have been read
    uint8_t  new_data           = 0;

    uint8_t len   = 0;
    BHY_RETURN_FUNCTION_TYPE error = 0;

    while (1)
    {
        sem_wait(&bhi160Sem);
        //wait until the interrupt fires unless we already know there are bytes remaining in the fifo

        readfifo:
        error       = bhy_read_fifo(array + bytes_left_in_fifo, 69 - bytes_left_in_fifo, &bytes_read, &bytes_remaining);
        bytes_read += bytes_left_in_fifo;
        fifoptr     = array; //point to the first index of the array

        //parse the fifo packets
        while((bytes_read > (bytes_remaining ? 18 : 0)) && !error)
        {
            switch(*fifoptr)//array[index]
            {
/*
                case VS_ID_ROTATION_VECTOR:
                case VS_ID_ROTATION_VECTOR_WAKEUP:
                case VS_ID_GAME_ROTATION_VECTOR:
                case VS_ID_GAME_ROTATION_VECTOR_WAKEUP:
                case VS_ID_GEOMAGNETIC_ROTATION_VECTOR:
                case VS_ID_GEOMAGNETIC_ROTATION_VECTOR_WAKEUP: len=11;  break;
*/
                case VS_ID_LINEAR_ACCELERATION:
                case VS_ID_LINEAR_ACCELERATION_WAKEUP:
                case VS_ID_ACCELEROMETER:
                case VS_ID_ACCELEROMETER_WAKEUP:
                    memcpy(accel, fifoptr+1, 6);
                    len=8;new_data++;
                break;
                case VS_ID_GYROSCOPE:
                case VS_ID_GYROSCOPE_WAKEUP:
                    memcpy(gyro, fifoptr+1, 6);
                    len=8;new_data++;
                break;
                case VS_ID_MAGNETOMETER:
                case VS_ID_MAGNETOMETER_WAKEUP:
                    memcpy(mag, fifoptr+1, 6);
                    len=8;new_data++;
                break;
/*
                case VS_ID_ORIENTATION:
                case VS_ID_ORIENTATION_WAKEUP:
                case VS_ID_GRAVITY:
                case VS_ID_GRAVITY_WAKEUP:
                case VS_ID_LINEAR_ACCELERATION:
                case VS_ID_LINEAR_ACCELERATION_WAKEUP:  len=8;  break;

                case VS_ID_SIGNIFICANT_MOTION:
                case VS_ID_SIGNIFICANT_MOTION_WAKEUP:
                case VS_ID_STEP_DETECTOR:
                case VS_ID_STEP_DETECTOR_WAKEUP:
                case VS_ID_TILT_DETECTOR:
                case VS_ID_TILT_DETECTOR_WAKEUP:
                case VS_ID_WAKE_GESTURE:
                case VS_ID_WAKE_GESTURE_WAKEUP:
                case VS_ID_GLANCE_GESTURE:
                case VS_ID_GLANCE_GESTURE_WAKEUP:
                case VS_ID_PICKUP_GESTURE:
                case VS_ID_PICKUP_GESTURE_WAKEUP:
                case VS_ID_HEART_RATE:
                case VS_ID_HEART_RATE_WAKEUP:     len=2;  break;

                case VS_ID_TEMPERATURE:
                case VS_ID_TEMPERATURE_WAKEUP:
                case VS_ID_AMBIENT_TEMPERATURE:
                case VS_ID_AMBIENT_TEMPERATURE_WAKEUP:
                case VS_ID_LIGHT:
                case VS_ID_LIGHT_WAKEUP:
                case VS_ID_PROXIMITY:
                case VS_ID_PROXIMITY_WAKEUP:
                case VS_ID_HUMIDITY:
                case VS_ID_HUMIDITY_WAKEUP:
                case VS_ID_STEP_COUNTER:
                case VS_ID_STEP_COUNTER_WAKEUP:
                case VS_ID_ACTIVITY:
                case VS_ID_ACTIVITY_WAKEUP:
*/
                case VS_ID_TIMESTAMP_LSW:
                case VS_ID_TIMESTAMP_MSW:
                case VS_ID_TIMESTAMP_LSW_WAKEUP:
                case VS_ID_TIMESTAMP_MSW_WAKEUP:  len=3;  break;

                //case VS_ID_BAROMETER:
                //case VS_ID_BAROMETER_WAKEUP:
                case VS_ID_META_EVENT:
                case VS_ID_META_EVENT_WAKEUP:     len=4;  break;
/*
                case VS_ID_UNCALIBRATED_MAGNETOMETER:
                case VS_ID_UNCALIBRATED_MAGNETOMETER_WAKEUP:
                case VS_ID_UNCALIBRATED_GYROSCOPE:
                case VS_ID_UNCALIBRATED_GYROSCOPE_WAKEUP:
                case VS_ID_DEBUG:                 len=14;  break;

                case VS_ID_BSX_C:
                case VS_ID_BSX_B:
                case VS_ID_BSX_A:                 len=17;  break;
*/
                default:
                   error = BHY_OUT_OF_RANGE;
                break;
            }
            fifoptr    += len;
            bytes_read -= len;

            if(new_data == 3)
            {
                sensorData[2]  = accel[1];
                sensorData[3]  = accel[0];
                sensorData[4]  = accel[3];
                sensorData[5]  = accel[2];
                sensorData[6]  = accel[5];
                sensorData[7]  = accel[4];
                sensorData[8]  = gyro[1];
                sensorData[9]  = gyro[0];
                sensorData[10] = gyro[3];
                sensorData[11] = gyro[2];
                sensorData[12] = gyro[5];
                sensorData[13] = gyro[4];
                sensorData[14] = mag[1];
                sensorData[15] = mag[0];
                sensorData[16] = mag[3];
                sensorData[17] = mag[2];
                sensorData[18] = mag[5];
                sensorData[19] = mag[4];
                new_data = 0;
                sem_post(&publishDataSem);
            }
        }
        bytes_left_in_fifo = 0;
        if (bytes_remaining)
        {
            //shifts the remaining bytes to the beginning of the buffer
            while (bytes_left_in_fifo < bytes_read)
            {
                array[bytes_left_in_fifo++] = *(fifoptr++);
            }
            goto readfifo;
        }
    }
}

/*******************************************************************************
 * @fn      bhi160Callback
 *
 * @brief   Callback value that signifies a BMI160 reading has happened
 *
 * @param   index - Index of BMI160 Sensor
 *
 * @return  none
 ******************************************************************************/
static void bhi160Callback(uint_least8_t index)
{
    sem_post(&bhi160Sem);
}
