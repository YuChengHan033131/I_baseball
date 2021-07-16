#include "test.h"
#include <pthread.h>
#include "BLEsend.h"
#include <unistd.h>//for usleep
#include <stdbool.h>//for bool in icm20649 activate
#include "sensor_boosterpack_icm20649.h"
#include <ti/display/Display.h>// for display_print through serial port
#include <semaphore.h>
#include "flashctrl.h"



static pthread_t sensorTask;
extern Display_Handle displayOut;
extern sem_t BLEconnected;
extern sem_t BLEinitDone;


void test_createTask(void)
{
    //mpdify from void ForceSensor_createTask(void)

    pthread_attr_t pAttrs;
    struct sched_param priParam;
    int retc;
    int detachState;

    pthread_attr_init(&pAttrs);
    priParam.sched_priority = 1;//same as other task

    detachState = PTHREAD_CREATE_DETACHED;
    retc = pthread_attr_setdetachstate(&pAttrs, detachState);

    if (retc != 0)
    {
        while(1);
    }

    pthread_attr_setschedparam(&pAttrs, &priParam);

    retc |= pthread_attr_setstacksize(&pAttrs, 1024);//1024=stack size

    if (retc != 0)
    {
        while(1);
    }

    retc = pthread_create(&sensorTask, &pAttrs, testFxn, NULL);

    if (retc != 0)
    {
        while(1);
    }

}
static void* testFxn(void *arg0){
    //sem_wait(&BLEinitDone);
   //wait until BLE connected
    //sem_wait(&BLEconnected);
    Display_printf(displayOut, 0, 0, "test_task");
    sleep(5);
    openflash();
    Display_clear(displayOut);
    uint16_t i;
    Display_printf(displayOut, 0, 0, "start");
    uint8_t data[20] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
    for(i=0;i<10000;i=i+1){
        data[0]=(uint8_t)(i>>8);
        data[1]=(uint8_t)i;
        Display_printf(displayOut,0,0,"in:%d",data[0]*256+data[1]);
        sendtoStore(data);
    }
    Display_printf(displayOut, 0, 0, "end");
    outputflashdata();
    Display_close(displayOut);
    return ;
}
