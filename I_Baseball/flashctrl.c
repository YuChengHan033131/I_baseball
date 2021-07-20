#include <file.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <mqueue.h>
#include <semaphore.h>
#include <pthread.h>
#include <unistd.h>
#include <ti/display/Display.h>
#include "BLEsend.h"
#include "flash_driver/MT29Flash.h"
#include "Board.h"
#include "flashctrl.h"

//Set this to the current UNIX time in seconds
const struct timespec flashts = {
    .tv_sec = 1470009600, //1469647026(2016/06/27/ 19:17:09)//1469923197
    .tv_nsec = 0
};

static SPI_Handle      spihandle;
static pthread_t       flashTask;

static volatile bool   Sampledata = false;
static volatile bool   write_init = true;

static pthread_mutex_t lockbuffer;
static pthread_mutex_t lockflash;
static pthread_cond_t  condflash;

static sem_t spaceFlashsem;
static sem_t countFlashsem;

static uint8_t  ringbuffer[NUMFLASH][DATA_LEN];


static uint16_t in  = 0;
static uint16_t out = 0;

/* Variables for the CIO functions */

static uint32_t day;
static uint32_t hour;
static uint32_t min;
static uint32_t sec;

extern Display_Handle displayOut;

static int_fast32_t udAddr = 0;
/*******************************************************************************
 *                                  LOCAL FUNCTIONS
 ******************************************************************************/
static void *flashTaskFxn(void *arg0);
static void getdata(uint8_t *result);
static void hextostr(uint8_t *value, char *result);
static void strtohex(char *value, uint8_t *result);
void flasheraseall(void);
//int_fast16_t FLASH_read(SPI_Handle handle);
//int_fast16_t FLASH_write(SPI_Handle handle, const void *buf, uint16_t count);

void flash_createTask(void)
{
    pthread_attr_t      attrs;
    struct sched_param  priParam;
    int                 retc;
    int                 detachState;

    /* Set priority and stack size attributes */
    pthread_attr_init(&attrs);
    priParam.sched_priority = FLASH_TASK_PRIORITY;

    detachState = PTHREAD_CREATE_DETACHED;
    retc = pthread_attr_setdetachstate(&attrs, detachState);
    if (retc != 0)
    {
        while (1);
    }

    pthread_attr_setschedparam(&attrs, &priParam);

    retc |= pthread_attr_setstacksize(&attrs, FLASH_TASK_STACK_SIZE);
    if (retc != 0)
    {
        while (1);
    }

    retc = pthread_create(&flashTask, &attrs, flashTaskFxn, NULL);
    if (retc != 0)
    {
        while (1);
    }

    // Initializing the semaphore
    sem_init(&spaceFlashsem,0,NUMFLASH);
    sem_init(&countFlashsem,0,0);
}

static void flashTaskInit(void)
{

    SPI_Params spiParams;

    SPI_Params_init(&spiParams);
    spiParams.frameFormat = SPI_POL0_PHA0;
    spiParams.bitRate = 2500000;//2.5M 2500000
    spihandle = SPI_open(Board_SPI0, &spiParams);
    if (spihandle == NULL) {
     //   Display_printf(displayOut, 0, 0, "Error Flash initializing SPI");
        while (1);
    }
    else {
      //  Display_printf(displayOut, 0, 0, "Flash SPI initialized");
    }
  //  usleep(250);//a minimum of 250us must elapse before issuing a RESET (FFh) command
    usleep(250);
    FLASH_initialize(spihandle);

    /*
    while(udAddr < NUM_BLOCKS * NUM_PAGE_BLOCK * PAGE_DATA_SIZE)
    {
        FlashBlockErase(handle, udAddr);
        udAddr =+ NUM_PAGE_BLOCK * PAGE_DATA_SIZE;
    }
    */

    udAddr = 0;

}

static void *flashTaskFxn(void *arg0)
{
    pthread_mutex_init(&lockbuffer, NULL);
    pthread_mutex_init(&lockflash, NULL);
    pthread_cond_init(&condflash, NULL);
    pthread_mutex_lock(&lockflash);

    /* Variables to keep track of the file copy progress */
    //unsigned int   bytesWritten = 0;
    uint8_t sendData[DATA_LEN];
    uint64_t  cnt = 0;

    bool empty = false;

    flashTaskInit();
    pthread_cond_wait(&condflash, &lockflash);
    pthread_mutex_unlock(&lockflash);
    while (1) {
        pthread_mutex_lock(&lockflash);
        while(!Sampledata)
        {
            pthread_cond_wait(&condflash, &lockflash);//wait pthread_cond_signal() to wake up
        }
        pthread_mutex_unlock(&lockflash);
/*#ifdef debug
        //this should be closed when using, or it will cause surge output signal
        getTime();
#endif*/
        getdata(sendData);
/*
#ifdef debug
        Display_printf(displayOut, 0, 0, "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x",
                       sendData[0],sendData[1],sendData[2],sendData[3],sendData[4],sendData[5],sendData[6],sendData[7],sendData[8],sendData[9],
                       sendData[10],sendData[11],sendData[12],sendData[13],sendData[14],sendData[15],sendData[16],sendData[17],sendData[18],sendData[19]);
#endif
*/
        FLASH_write(spihandle, &sendData, DATA_LEN);
        udAddr += DATA_LEN ;//address counter
        cnt = cnt + 1;

        if(cnt == 11250)
                {
            while(!empty)
                {
                    empty = FLASH_read(spihandle, sendData,DATA_LEN);
                  //  Display_printf(displayOut, 0, 0, "char = %s", array);
                    enqueue(sendData);
                    usleep(10000);
                }
               }
      //  cnt = 0;

    }
}

/*
 *  ======== fatfs_getFatTime ========
 */
int clock_init(void)
{
    int clock_set = 1;
    clock_set = clock_settime(CLOCK_REALTIME, &flashts);
    return clock_set;
}

void getTime(void)
{
    time_t seconds;
    struct tm *pTime;

    seconds = time(NULL);
    pTime = localtime(&seconds);


 //   Display_printf(displayOut, 0, 0, "%02d %02d:%02d:%02d",pTime->tm_mday,pTime->tm_hour,pTime->tm_min,pTime->tm_sec);


    day  = pTime->tm_mday;
    hour = pTime->tm_hour;
    min  = pTime->tm_min;
    sec  = pTime->tm_sec;
}

void openflash(void)
{

    //uint8_t  i = 1;
    //char *deletfile;
    char time[13];
    //write the begining time
 /*   if(write_init)  //orignal have
    {
        getTime();
        time[0]  = ' ';
        time[1]  = day/10  + '0';
        time[2]  = day%10-1  + '0';
        time[3]  = ' ';
        time[4]  = hour/10 + '0';
        time[5]  = hour%10 + '0';
        time[6]  = ':';
        time[7]  = min/10  + '0';
        time[8]  = min%10  + '0';
        time[9]  = ':';
        time[10] = sec/10  + '0';
        time[11] = sec%10  + '0';
        time[12] = '\n';
        //FlashPageProgram(handle, udAddr, *time, 13);
        FLASH_write(handle, &time, 13);
        udAddr += 13;
        write_init = false;
    }*/
///////////  orignal haven't  //////////////////////
    getTime();
    time[0]  = ' ';
    time[1]  = day/10  + '0';
    time[2]  = day%10-1  + '0';
    time[3]  = ' ';
    time[4]  = hour/10 + '0';
    time[5]  = hour%10 + '0';
    time[6]  = ':';
    time[7]  = min/10  + '0';
    time[8]  = min%10  + '0';
    time[9]  = ':';
    time[10] = sec/10  + '0';
    time[11] = sec%10  + '0';
    time[12] = '\n';
    //FLASH_write(spihandle, &time, 13);
    //udAddr += 13;
////////////////////////////////////
    pthread_mutex_lock(&lockflash);
    Sampledata = true;
    pthread_cond_signal(&condflash);
    pthread_mutex_unlock(&lockflash);
}

void closeflash(void)
{
    char time[13];

    pthread_mutex_lock(&lockflash);
    Sampledata = false;
    pthread_mutex_unlock(&lockflash);

    getTime();
    time[0] = ' ';
    time[1] = day/10  + '0';
    time[2] = day%10-1  + '0';
    time[3] = ' ';
    time[4] = hour/10 + '0';
    time[5] = hour%10 + '0';
    time[6] = ':';
    time[7] = min/10  + '0';
    time[8] = min%10  + '0';
    time[9] = ':';
    time[10] = sec/10  + '0';
    time[11] = sec%10  + '0';
    time[12] = '\n';

    //FlashPageProgram(handle, udAddr, *time, 13);
    FLASH_write(spihandle, &time, 2048-(udAddr%2048));//13
    udAddr += 2048-(udAddr%2048); //orignal haven't
    //FlashWriteDisable(handle);
    //udAddr += 13;
    //flash_change_flag = false;  //orignal haven't
    //write_init = true;  //orignal haven't
}

void sendtoStore(uint8_t *value)
{
    // wait if there is no space left:
    sem_wait(&spaceFlashsem);
    //Display_printf(displayOut, 0, 0,"sem_wait done");
    pthread_mutex_lock(&lockbuffer);
    //Display_printf(displayOut, 0, 0,"mutex_lock");
    memcpy(ringbuffer + ((in++) & (NUMFLASH-1)), value, 20);
    pthread_mutex_unlock(&lockbuffer);

    // increment the count of the number of items
    sem_post(&countFlashsem);
}

static void getdata(uint8_t *result)
{
    // Wait if there are no items in the buffer
    sem_wait(&countFlashsem);

    pthread_mutex_lock(&lockbuffer);
    memcpy(result, ringbuffer + ((out++) & (NUMFLASH-1)), 20);
    pthread_mutex_unlock(&lockbuffer);

    // Increment the count of the number of spaces
    sem_post(&spaceFlashsem);

}
static void hextostr(uint8_t *value, char *result)
{
    int i;
    uint8_t n0,n1;

    for(i = 0; i < DATA_LEN; i++)
    {
        n0 = (value[i]>>4) & 0x0f;
        n1 = value[i] & 0x0f;
        switch(n0)
        {
            case 0x00:
            result[2*i]='0';break;
            case 0x01:
            result[2*i]='1';break;
            case 0x02:
            result[2*i]='2';break;
            case 0x03:
            result[2*i]='3';break;
            case 0x04:
            result[2*i]='4';break;
            case 0x05:
            result[2*i]='5';break;
            case 0x06:
            result[2*i]='6';break;
            case 0x07:
            result[2*i]='7';break;
            case 0x08:
            result[2*i]='8';break;
            case 0x09:
            result[2*i]='9';break;
            case 0x0a:
            result[2*i]='a';break;
            case 0x0b:
            result[2*i]='b';break;
            case 0x0c:
            result[2*i]='c';break;
            case 0x0d:
            result[2*i]='d';break;
            case 0x0e:
            result[2*i]='e';break;
            case 0x0f:
            result[2*i]='f';break;
            default:
            result[2*i]='f';break;
        }
        switch(n1)
        {
            case 0x00:
            result[2*i+1]='0';break;
            case 0x01:
            result[2*i+1]='1';break;
            case 0x02:
            result[2*i+1]='2';break;
            case 0x03:
            result[2*i+1]='3';break;
            case 0x04:
            result[2*i+1]='4';break;
            case 0x05:
            result[2*i+1]='5';break;
            case 0x06:
            result[2*i+1]='6';break;
            case 0x07:
            result[2*i+1]='7';break;
            case 0x08:
            result[2*i+1]='8';break;
            case 0x09:
            result[2*i+1]='9';break;
            case 0x0a:
            result[2*i+1]='a';break;
            case 0x0b:
            result[2*i+1]='b';break;
            case 0x0c:
            result[2*i+1]='c';break;
            case 0x0d:
            result[2*i+1]='d';break;
            case 0x0e:
            result[2*i+1]='e';break;
            case 0x0f:
            result[2*i+1]='f';break;
            default:
            result[2*i+1]='f';break;
        }
    }
    result[2*DATA_LEN]   = '\n';
}
static void strtohex(char *value, uint8_t *result)
{
    int i;
    uint8_t n0,n1;

    for(i = 0; i < DATA_LEN*2; i=i+2)
    {
        n0 = value[i];
        n1 = value[i+1];
        result[i/2] = 0x00;
        switch(n0)
        {
            case '0':
                result[i/2] = result[i/2] | 0x00; break;
            case '1':
                result[i/2] = result[i/2] | 0x10; break;
            case '2':
                result[i/2] = result[i/2] | 0x20; break;
            case '3':
                result[i/2] = result[i/2] | 0x30; break;
            case '4':
                result[i/2] = result[i/2] | 0x40; break;
            case '5':
                result[i/2] = result[i/2] | 0x50; break;
            case '6':
                result[i/2] = result[i/2] | 0x60; break;
            case '7':
                result[i/2] = result[i/2] | 0x70; break;
            case '8':
                result[i/2] = result[i/2] | 0x80; break;
            case '9':
                result[i/2] = result[i/2] | 0x90; break;
            case 'a':
                result[i/2] = result[i/2] | 0xa0; break;
            case 'b':
                result[i/2] = result[i/2] | 0xb0; break;
            case 'c':
                result[i/2] = result[i/2] | 0xc0; break;
            case 'd':
                result[i/2] = result[i/2] | 0xd0; break;
            case 'e':
                result[i/2] = result[i/2] | 0xe0; break;
            case 'f':
                result[i/2] = result[i/2] | 0xf0; break;
            default:
                result[i/2] = result[i/2] | 0x00; break;
        }
        switch(n1)
        {
            case '0':
                result[i/2] = result[i/2] | 0x00; break;
            case '1':
                result[i/2] = result[i/2] | 0x01; break;
            case '2':
                result[i/2] = result[i/2] | 0x02; break;
            case '3':
                result[i/2] = result[i/2] | 0x03; break;
            case '4':
                result[i/2] = result[i/2] | 0x04; break;
            case '5':
                result[i/2] = result[i/2] | 0x05; break;
            case '6':
                result[i/2] = result[i/2] | 0x06; break;
            case '7':
                result[i/2] = result[i/2] | 0x07; break;
            case '8':
                result[i/2] = result[i/2] | 0x08; break;
            case '9':
                result[i/2] = result[i/2] | 0x09; break;
            case 'a':
                result[i/2] = result[i/2] | 0x0a; break;
            case 'b':
                result[i/2] = result[i/2] | 0x0b; break;
            case 'c':
                result[i/2] = result[i/2] | 0x0c; break;
            case 'd':
                result[i/2] = result[i/2] | 0x0d; break;
            case 'e':
                result[i/2] = result[i/2] | 0x0e; break;
            case 'f':
                result[i/2] = result[i/2] | 0x0f; break;
            default:
                result[i/2] = result[i/2] | 0x00; break;
        }
    }
}

void outputflashdata(void)
{
    //FLASH_read(spihandle);

    bool empty = false;

    uint8_t sendData[DATA_LEN];
    while(!empty)
    {
        empty = FLASH_read(spihandle, sendData,DATA_LEN);
        if(!empty){
            enqueue(sendData);
            //Display_printf(displayOut,0,0,"out:%d",sendData[2]*256+sendData[3]);
            //usleep(10000);
        }
    }
    //send remain data in flashbuff & readbuff




}
void flasheraseall(void)
{
    udAddr = 0;
    //uint8_t statusregister;
    while(udAddr < (NUM_BLOCKS-2) * NUM_PAGE_BLOCK * PAGE_DATA_SIZE)
    {
        FlashBlockErase(spihandle, udAddr);
        //FlashReadStatusRegister(handle, &statusregister);
        //Display_printf(display, 0, 0, "after %d erase, status = %02x",udAddr, statusregister);
        udAddr += NUM_PAGE_BLOCK * PAGE_DATA_SIZE;
    }
    Display_printf(displayOut, 0, 0, "erase finish!");
    udAddr = 0;
}
