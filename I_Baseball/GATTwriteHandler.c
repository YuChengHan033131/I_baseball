#include "test.h"
#include <pthread.h>
#include "BLEsend.h"
#include <unistd.h>//for usleep
#include <stdbool.h>
#include <ti/display/Display.h>// for display_print through serial port
#include <semaphore.h>
#include "flashctrl.h"
#include <Profile/baseball6xs_service.h>



static pthread_t sensorTask;
extern Display_Handle displayOut;
extern sem_t BLEconnected;
extern sem_t BLEinitDone;
sem_t writeCallback;//write callback thread

static void* handlerFxn(void *arg0);

void GATTwriteHandler_createTask(void)
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

    retc = pthread_create(&sensorTask, &pAttrs, handlerFxn, NULL);

    if (retc != 0)
    {
        while(1);
    }

}
static void* handlerFxn(void *arg0){
    uint16_t i;
    uint8_t data[DATA_LEN];
    uint8_t endsignal[DATA_LEN];
    for(i=0;i<DATA_LEN;i++){
        endsignal[i]=0xff;
        data[i]=0xff;
    }
    uint8_t command;
    sem_init(&writeCallback,1,0);//allow other thread , initial value =0
    while(1){
        sem_wait(&writeCallback);
        Baseball6xs_getParameter(BASEBALL6xs_CONF, &command);
        switch(command){
             case 1:
                 Display_printf(displayOut,0,0,"Choose data output");
                 break;
             case 2:
                 Display_printf(displayOut,0,0,"All data output");
                 //open until flash fixed
                 /*num=total_set_number();
                 Display_printf(displayOut,0,0,"total set number=%d",num);
                 for(i=0;i<num;i++){
                     outputflashdata(i);
                 }
                 Display_printf(displayOut,0,0,"All data output done");*/
                 break;
             case 3:
                 //open until flash fixed
                 //flasheraseall();
                 Display_printf(displayOut,0,0,"Erase data done");
                 break;
             case 4:
                 Display_printf(displayOut,0,0,"Exit");
                 break;
             case 5:
                 Display_printf(displayOut,0,0,"test data output");
                 for(i=0;i<100;i++){
                     data[12]=(uint8_t)(i>>8);
                     data[13]=(uint8_t)(i);
                     enqueue(data);
                     //Display_printf(displayOut,0,0,"in=%d",i);
                 }
                 break;

             default:
                 Display_printf(displayOut,0,0,"invalid output");
                break;
        }
        for(i=0;i<10;i++){//temp need to read BLE.c to know buffer status, to decide endsignal amount
            enqueue(endsignal);
        }


    }

    //Display_printf(displayOut,0,0,"end");
    //Display_close(displayOut);

    return ;
}
