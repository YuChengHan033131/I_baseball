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

static volatile bool   write_init = true;

static pthread_mutex_t lockbuffer;
static pthread_mutex_t lockflash;
static pthread_cond_t  condflash;

static sem_t spaceFlashsem;
static sem_t countFlashsem;
static sem_t open_flash;
static bool close_flash;

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

}

static void *flashTaskFxn(void *arg0)
{
    pthread_mutex_init(&lockbuffer, NULL);
    pthread_mutex_init(&lockflash, NULL);
    pthread_cond_init(&condflash, NULL);

    pthread_mutex_lock(&lockflash);

    sem_init(&open_flash,0,0);
    flashTaskInit();
    //flasheraseall();//should be remove

    pthread_mutex_unlock(&lockflash);

    uint8_t sendData[DATA_LEN];

    //wait if openflash() is not called
    sem_wait(&open_flash);
    Display_printf(displayOut,0,0,"flash opened");

    //get write address from flash
    get_writeaddress(spihandle);

    while (!close_flash) {

        // Wait if there are no items in the buffer
        sem_wait(&countFlashsem);

        //check if the post signal is for close flash
        if(close_flash){
            break;
        }

        getdata(sendData);

        FLASH_write(spihandle, &sendData, DATA_LEN);

    }
    Display_printf(displayOut,0,0,"flash closed");
    write_writeaddress(spihandle);


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
    //mutex for waiting if flash is initializing
    pthread_mutex_lock(&lockflash);
    sem_post(&open_flash);
    close_flash = false;
    pthread_mutex_unlock(&lockflash);
}

void closeflash(void)
{
    //wait until ringbuff empty
    uint8_t data;
    do{
        sem_getvalue(&countFlashsem,&data);
    }while(data>0);

    close_flash = true;

    //unlock wait before getdata()
    sem_post(&countFlashsem);
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

    pthread_mutex_lock(&lockbuffer);
    memcpy(result, ringbuffer + ((out++) & (NUMFLASH-1)), 20);
    pthread_mutex_unlock(&lockbuffer);

    // Increment the count of the number of spaces
    sem_post(&spaceFlashsem);

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
            //enqueue(sendData);
            //usleep(10000);
            Display_printf(displayOut,0,0,"out:%d",sendData[12]*256+sendData[13]);
            Display_printf(displayOut,0,0,"accx:%d",sendData[0]*256+sendData[1]);
            Display_printf(displayOut,0,0,"accy:%d",sendData[2]*256+sendData[3]);
            Display_printf(displayOut,0,0,"accz:%d",sendData[4]*256+sendData[5]);


        }
    }
    //send remain data in flashbuff & readbuff




}
void flasheraseall(void)
{
    udAddr = 0;
    //uint8_t statusregister;
    while(udAddr < (NUM_BLOCKS-2) * NUM_PAGE_BLOCK)
    {
        FlashBlockErase(spihandle, udAddr);
        //FlashReadStatusRegister(handle, &statusregister);
        //Display_printf(display, 0, 0, "after %d erase, status = %02x",udAddr, statusregister);
        udAddr += NUM_PAGE_BLOCK;
    }
    udAddr = 0;
}
