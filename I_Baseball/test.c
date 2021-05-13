#include "test.h"
#include <pthread.h>
#include "BLEsend.h"
#include <unistd.h>//for usleep
#include <stdbool.h>//for bool in icm20649 activate
#include "sensor_boosterpack_icm20649.h"
#include <ti/display/Display.h>// for display_print through tera term


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
    uint8_t data[20]={0xcc,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA};//=0xAA=1010101010=170
    enqueue(data);
    uint64_t num64=1023;//bin=111111111="1"*9
    data[0]=(uint8_t)num64;
    num64=num64>>8;
    data[1]=(uint8_t)num64;
    num64=num64>>8;
    data[2]=(uint8_t)num64;
    num64=num64>>8;
    data[3]=(uint8_t)num64;
    while(1){
        enqueue(data);
        sleep(1);
    }





    //Baseball6xs_setParameter(0x00, 0x14, data);//param=0 for sending data,data bytes length,data

}
