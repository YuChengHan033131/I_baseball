#include "test.h"
#include <pthread.h>
#include "BLEsend.h"
#include <unistd.h>//for usleep
#include <stdbool.h>//for bool in icm20649 activate
#include "sensor_boosterpack_icm20649.h"
#include <ti/display/Display.h>// for display_print through serial port
#include <semaphore.h>



static pthread_t sensorTask;


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
 extern Display_Handle displayOut;
static void* testFxn(void *arg0){
    //wait until icm20649 wakeup from sleep
    //sem_wait(&BLEinitDone);
   //wait until BLE connected
    //sem_wait(&BLEconnected);
    Display_clear(displayOut);
    uint8_t i;
    for(i=0;i<255;i=i+1){
        Display_printf(displayOut,0,0,"count: %d",i);
        usleep(500);

    }
    Display_close(displayOut);
    return ;
}
