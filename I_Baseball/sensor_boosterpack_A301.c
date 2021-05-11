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
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/drivers/dpl/HwiP.h>
#include <pthread.h>
#include <semaphore.h>
#include <ti/display/Display.h>

#include <Profile/baseball3xs_service.h>
#include <Profile/baseball6xs_service.h>
#include <sensor_boosterpack_A301.h>
#include "flashctrl.h"
#include "BLEsend.h"

/*******************************************************************************
 *                             VARIABLES
 ******************************************************************************/
/* Task setup */
static pthread_t sensorTask;
static pthread_mutex_t lock;
static pthread_cond_t cond;
static bool down_en = false;
static volatile bool sampleData;
static sem_t forceSem;

/* Parameters */
extern volatile bool Baseball3xs;
extern volatile bool Baseball6xs;
extern volatile bool store_en;

static uint16_t resultsBuffer[2];
static uint8_t pin1_data[BASEBALL3xs_DATA_LEN] = {0xAD,0xC1};
static uint8_t pin2_data[BASEBALL3xs_DATA_LEN] = {0xAD,0xC2};

/* Used for log messages */
extern Display_Handle displayOut;

/*******************************************************************************
 *                                  LOCAL FUNCTIONS
 ******************************************************************************/
static void* sensorTaskFxn(void *arg0);
void Hwi_adc(uintptr_t arg0);

/*******************************************************************************
 *                                    FUNCTIONS
 ******************************************************************************/

/*******************************************************************************
 * @fn      TemperatureSensor_createTask
 *
 * @brief   Task creation function for the TemperatureSensor
 *
 * @param   none
 *
 * @return  none
 */
void ForceSensor_createTask(void)
{
    pthread_attr_t pAttrs;
    struct sched_param priParam;
    int retc;
    int detachState;

    pthread_attr_init(&pAttrs);
    priParam.sched_priority = SENSOR_TASK_PRIORITY;

    detachState = PTHREAD_CREATE_DETACHED;
    retc = pthread_attr_setdetachstate(&pAttrs, detachState);

    if (retc != 0)
    {
        while(1);
    }

    pthread_attr_setschedparam(&pAttrs, &priParam);

    retc |= pthread_attr_setstacksize(&pAttrs, SENSOR_TASK_STACK_SIZE);

    if (retc != 0)
    {
        while(1);
    }

    retc = pthread_create(&sensorTask, &pAttrs, sensorTaskFxn, NULL);

    if (retc != 0)
    {
        while(1);
    }

    sem_init(&forceSem, 1, 0);
}

/*******************************************************************************
 * @fn      sensorTaskInit
 *
 * @brief   Initialization function for the SensorBP IR temperature sensor
 *
 ******************************************************************************/
static void sensorTaskInit(void)
{
    HwiP_Params adcHwiParams;

    HwiP_Params_init(&adcHwiParams);
    adcHwiParams.arg = 0;
    adcHwiParams.priority = ~0;
    HwiP_create(40, &Hwi_adc, &adcHwiParams);
    HwiP_enableInterrupt(40);

    // Initializing ADC (MCLK/1/1) with temperature sensor routed */
    ADC14_enableModule();
    ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1, ADC_NOROUTE);

    // Configuring GPIOs (P5.5 - P5.4) A0 - A1
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN5 | GPIO_PIN4, GPIO_TERTIARY_MODULE_FUNCTION);

    // Configuring ADC Memory.
    ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM1, true);

    ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, //if use REF_A instead -->> ADC_VREFPOS_INTBUF_VREFNEG_VSS
            ADC_INPUT_A0, false);
    ADC14_configureConversionMemory(ADC_MEM1, ADC_VREFPOS_AVCC_VREFNEG_VSS,
            ADC_INPUT_A1, false);

    // Configuring the sample/hold time for TBD
    //ADC14_setSampleHoldTime(ADC_PULSE_WIDTH_192,ADC_PULSE_WIDTH_192);

    // Enabling sample timer in auto iteration mode and interrupts
    ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);
    ADC14_enableInterrupt(ADC_INT1);

    /* Initializing the mutex */
    pthread_mutex_init(&lock, NULL);
    pthread_cond_init(&cond, NULL);
}

void SensorA301_Deactivate(void)
{
    /* Deactivate task */
    pthread_mutex_lock(&lock);
    sampleData = false;
    pthread_mutex_unlock(&lock);
}

void SensorA301_Activate(bool down_sample)
{
    /* Activate task */
    pthread_mutex_lock(&lock);
    sampleData = true;
    down_en = down_sample;
    pthread_cond_signal(&cond);
    pthread_mutex_unlock(&lock);
}

/*******************************************************************************
 * @fn      sensorTaskFxn
 *
 * @brief   The task loop of the temperature readout task
 *
 * @return  none
 ******************************************************************************/
static void* sensorTaskFxn(void *arg0)
{
    int cnt = 2;

    /* Initializing the task */
    sensorTaskInit();

    /* Task loop */
    while(1)
    {
        pthread_mutex_lock(&lock);
        while(!sampleData)
        {
            pthread_cond_wait(&cond, &lock);
        }
        pthread_mutex_unlock(&lock);

        if(sem_wait(&forceSem) == 0)
        {
            pin1_data[cnt]  = resultsBuffer[0] >> 8;
            pin2_data[cnt++]= resultsBuffer[1] >> 8;
            pin1_data[cnt]  = resultsBuffer[0];
            pin2_data[cnt++]= resultsBuffer[1];

            if(cnt == 20)
            {
                cnt = 2;

                if(store_en == true)
                {
                    sendtoStore(pin1_data);
                    sendtoStore(pin2_data);
                }

                enqueue(pin1_data);
                enqueue(pin2_data);
            }
        }
    }
}

/* Hardware Interrupt for ADC */
void Hwi_adc(uintptr_t arg0)
{
    uint64_t status;
    int cnt = 0;

    status = MAP_ADC14_getEnabledInterruptStatus();
    MAP_ADC14_clearInterruptFlag(status);

    if (status & ADC_INT1)
    {
        if(down_en)
        {
            if(cnt == 0)
            {
                ADC14_getMultiSequenceResult(resultsBuffer);
                sem_post(&forceSem);
                cnt++;
            }
            else
            {
                cnt = 0;
            }
        }
        else
        {
            ADC14_getMultiSequenceResult(resultsBuffer);
            sem_post(&forceSem);
        }
    }
}

