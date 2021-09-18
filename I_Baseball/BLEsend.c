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
#include <mqueue.h>
#include <semaphore.h>

#include <ti/display/Display.h>

#include <Profile/baseball3xs_service.h>
#include <Profile/baseball6xs_service.h>
#include "Profile/profile_util.h"
#include "BLEsend.h"
#include "Board.h"
#include "sensor_configuration.h"

sem_t BLEconnected;
/*******************************************************************************
 *                             LOCAL VARIABLES
 ******************************************************************************/
/* Task setup */
static pthread_t BLEsendTask;

extern volatile bool Baseball3xs;
extern volatile bool Baseball6xs;
//extern pthread_mutex_t lock;

static pthread_mutex_t lockbuffer;

//static pthread_mutex_t lockble;
//static pthread_cond_t  condble;

/* Sensor Objects */
extern Display_Handle displayOut;

static sem_t spacesem;
static sem_t countsem;

static uint8_t ringbuffer[NUM][DATA_LEN];
static uint16_t in  = 0;
static uint16_t out = 0;
//static uint16_t waitting = 0;
//static volatile bool post = false;
/*******************************************************************************
 *                                  LOCAL FUNCTIONS
 ******************************************************************************/
static void *BLEsendTaskFxn(void *arg0);
static void dequeue(uint8_t *result);
/*******************************************************************************
 *                                   FUNCTIONS
 ******************************************************************************/

void BLEsend_createTask(void)
{
    pthread_attr_t pAttrs;
    struct sched_param priParam;
    int retc;
    int detachState;

    pthread_attr_init(&pAttrs);
    priParam.sched_priority = BLEsend_TASK_PRIORITY;

    detachState = PTHREAD_CREATE_DETACHED;
    retc = pthread_attr_setdetachstate(&pAttrs, detachState);

    if (retc != 0)
    {
        while(1);
    }

    pthread_attr_setschedparam(&pAttrs, &priParam);

    retc |= pthread_attr_setstacksize(&pAttrs, BLEsend_TASK_STACK_SIZE);

    if (retc != 0)
    {
        while(1);
    }


    retc = pthread_create(&BLEsendTask, &pAttrs, BLEsendTaskFxn, NULL);

    if (retc != 0)
    {
        while(1);
    }

    // Initializing the semaphore
    sem_init(&spacesem,0,NUM);
    sem_init(&countsem,0,0);
    sem_init(&BLEconnected,1,0);//allow other thread , initial value =0

}

void ringbuffer_init(void)
{
 //   Display_print0(displayOut,0,0,"Ring Buffer Initializing... ");
    pthread_mutex_init(&lockbuffer, NULL);
    //pthread_mutex_init(&lockqueue,NULL);
}

static void *BLEsendTaskFxn(void *arg0)
{
    //uint8_t ret = 0;
    uint8_t sendData[20];

    // Initialize the task
    ringbuffer_init();
    sem_wait(&BLEconnected);
    usleep(220000);
    while (1)
    {
        dequeue(sendData);
        //ret = 0;
/*
#ifdef debug

        //Display_printf(displayOut, 0, 0, "%02x%02x%02x%02x",
        //               sendData[0],sendData[1],sendData[2],sendData[3]);

        Display_printf(displayOut, 0, 0, "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x",
                       sendData[0],sendData[1],sendData[2],sendData[3],sendData[4],sendData[5],sendData[6],sendData[7],sendData[8],sendData[9],
                       sendData[10],sendData[11],sendData[12],sendData[13],sendData[14],sendData[15],sendData[16],sendData[17],sendData[18],sendData[19]);
#endif
*/

#ifdef debug
        //this should be closed when using, or it will cause surge output signal
        getTime();
#endif

     //sendagain:
        /*
        if(Baseball3xs)
        {
            Baseball3xs_setParameter(BASEBALL3xs_DATA, BASEBALL3xs_DATA_LEN, sendData);
            usleep(5100);//sleep 5.1ms
        }
        else if(Baseball6xs)
        {
            Baseball6xs_setParameter(BASEBALL6xs_DATA, BASEBALL6xs_DATA_LEN, sendData);
            usleep(5100);//sleep 5.1ms
        }
        */
        //Baseball3xs_setParameter(BASEBALL3xs_DATA, BASEBALL3xs_DATA_LEN, sendData);
        //Display_printf(displayOut,0,0,"BLE sendData:%d",sendData[12]*256+sendData[13]);
        Baseball6xs_setParameter(BASEBALL6xs_DATA, BASEBALL6xs_DATA_LEN, sendData);
        usleep(4001);

/*
        if(ret)
        {
            Display_printf(displayOut, 0, 0, "notify error !!");
            goto sendagain;
        }
*/
    }
}

void enqueue(uint8_t *value)
{
    // wait if there is no space left:
    sem_wait(&spacesem);

    pthread_mutex_lock(&lockbuffer);
    memcpy(ringbuffer + ((in++) & (NUM-1)), value, DATA_LEN);
    pthread_mutex_unlock(&lockbuffer);

    // increment the count of the number of items
    sem_post(&countsem);
}

static void dequeue(uint8_t *result)
{
    // Wait if there are no items in the buffer
    sem_wait(&countsem);

    pthread_mutex_lock(&lockbuffer);
    memcpy(result, ringbuffer + ((out++) & (NUM-1)), DATA_LEN);
    pthread_mutex_unlock(&lockbuffer);

    // Increment the count of the number of spaces
    sem_post(&spacesem);

}
