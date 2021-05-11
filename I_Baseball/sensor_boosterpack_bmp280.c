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
#include <pthread.h>
#include <unistd.h>
#include <ti/drivers/I2C.h>
#include <ti/display/Display.h>
#include <Profile/baseball3xs_service.h>
#include <Profile/baseball6xs_service.h>
#include <sensor_driver/Bmp280.h>
#include "sensor_boosterpack_bmp280.h"
#include "BLEsend.h"
#include "flashctrl.h"
#include "Board.h"
#include "sensor_configuration.h"

/*******************************************************************************
 *                             VARIABLES
 ******************************************************************************/
/* Task setup */
pthread_t bmp280Task;
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
extern Display_Handle displayOut;

/*******************************************************************************
 *                                  LOCAL FUNCTIONS
 ******************************************************************************/
static void *barometerTaskFxn(void *arg0);

/*******************************************************************************
 *                                   FUNCTIONS
 ******************************************************************************/

/*******************************************************************************
 * @fn      SensorBPBar_createTask
 *
 * @brief   Task creation function for barometer sensor
 *
 * @param   none
 *
 * @return  none
 ******************************************************************************/
void SensorBMP280_createTask(void)
{
    pthread_attr_t pAttrs;
    struct sched_param priParam;
    int retc;
    int detachState;

    pthread_attr_init(&pAttrs);
    priParam.sched_priority = BAROMETER_TASK_PRIORITY;

    detachState = PTHREAD_CREATE_DETACHED;
    retc = pthread_attr_setdetachstate(&pAttrs, detachState);

    if (retc != 0)
    {
        while(1);
    }

    pthread_attr_setschedparam(&pAttrs, &priParam);

    retc |= pthread_attr_setstacksize(&pAttrs, BAROMETER_TASK_STACK_SIZE);

    if (retc != 0)
    {
        while(1);
    }

    retc = pthread_create(&bmp280Task, &pAttrs, barometerTaskFxn, NULL);

    if (retc != 0)
    {
        while(1);
    }
}

/*******************************************************************************
 * @fn      SensorBPBar_processCharChangeEvt
 *
 * @brief   SensorBP Barometer event handling
 *
 ******************************************************************************/
void SensorBMP280_Deactivate(void)
{
    // Deactivate task
    pthread_mutex_lock(&lock);
    sampleData = false;
    pthread_mutex_unlock(&lock);
}

void SensorBMP280_Activate(void)
{
    // Activate task
    pthread_mutex_lock(&lock);
    sampleData = true;
    pthread_cond_signal(&cond);
    pthread_mutex_unlock(&lock);
}

/*******************************************************************************
 * @fn      barometerTaskInit
 *
 * @brief   Initialization function for the SensorTag barometer
 *
 ******************************************************************************/
static void barometerTaskInit(void)
{
    bool ret;

    /* Initialize the module state variables */

    /* Initialize characteristics and sensor driver */
    ret = SensorBmp280_init();
    if(ret == true) Display_print0(displayOut, 0, 0, "BMP280 initial success!");

    /* Initialize our mutex lock and condition */
    pthread_mutex_init(&lock, NULL);
    pthread_cond_init(&cond, NULL);
}

/*******************************************************************************
 * @fn      barometerTaskFxn
 *
 * @brief   The task loop of the humidity readout task
 *
 * @return  none
 ******************************************************************************/
static void *barometerTaskFxn(void *arg0)
{
    uint8_t data[SENSOR_BMP280_DATASIZE] = {0};
    uint8_t sendData[DATA_LEN] = {0x28,0x0F,0};
    uint8_t cnt = 0;
    int32_t temp;
    uint32_t press;
    bool success;

    uint32_t pauseTimeUS;

    /* Initialize the task */
    barometerTaskInit();

    /* Deactivate task (active only when measurement is enabled) */
    sampleData = false;

    while (1)
    {
        pthread_mutex_lock(&lock);
        while(!sampleData)
        {
            pthread_cond_wait(&cond, &lock);
        }
        pthread_mutex_unlock(&lock);

        /* Enable Sensor */
        SensorBmp280_enable(true);
        SensorBmp280_config(STANDBY_TIME_500ms, FILTER_COE_16);
        //Display_print0(displayOut, 0, 0, "BMP280 enable success!");

        /* Reading out the value */
        //usleep(BAROMETER_FSM_PERIOD * 1000);
        success = SensorBmp280_read(data);
        //if(success == true) Display_print0(displayOut, 0, 0, "BMP280 read success!");

        /* Disable Sensor */
        SensorBmp280_enable(false);
        //Display_print0(displayOut, 0, 0, "BMP280 disable success!");

        // Processing
        if (success)
        {
          SensorBmp280_convert(data,&temp,&press);
          //Display_print0(displayOut, 0, 0, "BMP280 convert success!");

          sendData[cnt+4] = (temp >> 16) & 0xFF;
          sendData[cnt+3] = (temp >> 8) & 0xFF;
          sendData[cnt+2] = temp & 0xFF;

          sendData[cnt+7] = (press >> 16) & 0xFF;
          sendData[cnt+6] = (press >> 8) & 0xFF;
          sendData[cnt+5] = press & 0xFF;

          cnt += 6;
        }

        /* Send data */
        if(cnt == 18)
        {
            cnt = 0;
            if(store_en == true)
            {
                sendtoStore(sendData);
            }
            enqueue(sendData);
            //Display_print0(displayOut, 0, 0, "BMP280 data has been sent");
            //Display_printf(displayOut, 0, 0, "temp = %d, press = %d\n", temp, press);
        }

        pauseTimeUS = sensorPeriod * 1000;

        /* Next cycle */
        if(pauseTimeUS >= 1000000)
        {
            sleep(pauseTimeUS/1000000);
        }
        else
        {
            usleep(pauseTimeUS);
        }
    }
}
