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
bool sensorOn = false;

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
    for(i=0;i<DATA_LEN;i++){
        data[i]=0xff;
    }
    uint32_t command;
    sem_init(&writeCallback,1,0);//allow other thread , initial value =0
    while(1){
        sem_wait(&writeCallback);
        Baseball6xs_getParameter(BASEBALL6xs_CONF, &command);
        Display_printf(displayOut,0,0,"command:%x",command);

        uint16_t num;
        switch(command>>24){//only first byte is command, others are parameters
             case 1:
                 switch((command&0x00ff0000)>>16){
                 case 1:
                     Display_printf(displayOut,0,0,"get set_number");
                     num=total_set_number();
                     data[0]=(uint8_t)(num>>8);
                     data[1]=(uint8_t)(num);
                     enqueue(data);
                     break;
                 case 2:
                     Display_printf(displayOut,0,0,"choose data output");
                     uint16_t set_number=(uint16_t)(command&0x0000ffff);
                     Display_printf(displayOut,0,0,"set_number:%d",set_number);
                     outputflashdata(set_number);
                     break;
                 default:
                     break;
                 }
                 break;
             case 2:
                 Display_printf(displayOut,0,0,"All data output");
                 num=total_set_number();
                 Display_printf(displayOut,0,0,"total set number=%d",num);
                 for(i=0;i<num;i++){
                     outputflashdata(i);
                 }
                 Display_printf(displayOut,0,0,"All data output done");
                 break;
             case 3:
                 flasheraseall();
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
             case 6://get sensor state
                 Display_printf(displayOut,0,0,"get sensor state");
                 for(i=0;i<DATA_LEN;i++){
                     data[i]=0xff;
                 }
                 //return sensor on or off
                 data[0]=(uint8_t)sensorOn;
                 enqueue(data);
                 break;
             case 10://sensor on
                 sensorOn = true;
                 Display_printf(displayOut,0,0,"sensor on");
                 break;
             case 11://sensor off
                 sensorOn = false;
                 Display_printf(displayOut,0,0,"sensor off");
                 break;
             default:
                 Display_printf(displayOut,0,0,"invalid output");
                break;
        }
        enqueue(ENDSIGNAL);
    }

    //Display_printf(displayOut,0,0,"end");
    //Display_close(displayOut);

    return ;
}
