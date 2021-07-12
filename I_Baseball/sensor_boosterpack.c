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
 *                              INCLUDES
 ******************************************************************************/
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <pthread.h>
#include <mqueue.h>
#include <semaphore.h>
#include <time.h>
#include <unistd.h>
#include <ti/display/Display.h>
#define DISPLAY_DISABLE//disable display in this file
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/Timer.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/sap/sap.h>
#include <ti/sbl/sbl.h>
#include <ti/sbl/sbl_image.h>

#include <Profile/baseball3xs_service.h>
#include <Profile/baseball6xs_service.h>
#include <Profile/profile_util.h>
#include "flashctrl.h"
#include "sensor_boosterpack_A301.h"
#include "sensor_boosterpack_bhi160.h"
#include "sensor_boosterpack_bmp280.h"
#include "sensor_boosterpack_bma253.h"
#include "sensor_boosterpack_icm20649.h"
#include "sensor_boosterpack.h"
#include "sensor_configuration.h"
#include "Board.h"
#include "platform.h"

extern sem_t BLEconnected;
/*******************************************************************************
 *                             VARIABLES
 ******************************************************************************/
/* Used to block SNP calls during a synchronous transaction. */
static mqd_t sensQueueRec;
static mqd_t sensQueueSend;

/* Clock instances for internal periodic events. */
static Timer_Handle timer0;

/* Task configuration */
static pthread_t sensTask;

/* SAP Parameters for opening serial port to SNP */
static SAP_Params sapParams;

/* Device Name */
static uint8_t snpDeviceName[] =
        { 'I', '_', 'B', 'a', 's', 'e', 'b', 'a', 'l', 'l' };

/* GAP - SCAN RSP data (max size = 31 bytes) */
static uint8_t scanRspData[] =
{
    /* Complete name */
    0x0a, /* Length */
    SAP_GAP_ADTYPE_LOCAL_NAME_COMPLETE, 'I', '_', 'B', 'a', 's', 'e', 'b', 'a', 'l', 'l',

    /* Connection interval range */
    0x05, /* Length */
    0x12,
    LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
    HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
    LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),
    HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

    /* TX power level */
    0x02, /* Length */
    0x0A,
    0
};

/* GAP - Advertisement data (max size = 31 bytes, though this is
   best kept short to conserve power while advertising) */
static uint8_t advertData[] =
{
    /* Flags; this sets the device to use limited discoverable
       mode (advertises for 30 seconds at a time) instead of general
       discoverable mode (advertises indefinitely) */
    0x02, /* Length */
    SAP_GAP_ADTYPE_FLAGS,
    DEFAULT_DISCOVERABLE_MODE | SAP_GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

    /* Manufacturer specific advertising data */
    0x06, 0xFF,
    LO_UINT16(TI_COMPANY_ID),
    HI_UINT16(TI_COMPANY_ID),
    TI_ST_DEVICE_ID,
    TI_ST_KEY_DATA_ID, 0x00
};

/* Connection Handle - only one device currently allowed to connect to SNP */
static uint16_t connHandle = AP_DEFAULT_CONN_HANDLE;

/* BD Addr of the NWP */
static char nwpstr[] = "NWP:  0xFFFFFFFFFFFF";
#define nwpstrIDX       8

// BD Addr of peer device in connection
static char peerstr[] = "Peer: 0xFFFFFFFFFFFF";
#define peerstrIDX       8

/* Used for log messages */
extern Display_Handle displayOut;

/* Used for exiting threads */
extern pthread_t bmp280Task;
extern pthread_t bma253Task;
extern pthread_t bhi160Task;
extern pthread_t icm20649Task;

/* Sensor Parameters */

static uint8_t Baseball6xsConfig;
volatile bool Baseball6xs = false;
static uint8_t Baseball3xsConfig;
volatile bool Baseball3xs = false;

volatile bool store_en = false;

/*******************************************************************************
 *                            FUNCTION PROTOTYPES
 ******************************************************************************/
static void AP_init(void);
static void *AP_taskFxn(void *arg0);
static void AP_initServices(void);
static void AP_keyHandler(void);
static void AP_bslKeyHandler(void);
static void AP_asyncCB(uint8_t cmd1, void *pParams);
static void AP_processSNPEventCB(uint16_t event, snpEventParam_t *param);
static void AP_timerHandler(Timer_Handle handle);

static void AP_processBaseball6xsChangeCB(uint8_t charID);
static void AP_processBaseball6xscccdCB(uint8_t charID, uint16_t value);
static void Baseball6xs_processCharChangeEvt(uint8_t paramID);
static void initBaseball6xsCharacteristicValue(uint8_t paramID, uint8_t value, uint8_t paramLen);

static void AP_processBaseball3xsChangeCB(uint8_t charID);
static void AP_processBaseball3xscccdCB(uint8_t charID, uint16_t value);
static void Baseball3xs_processCharChangeEvt(uint8_t paramID);
static void initBaseball3xsCharacteristicValue(uint8_t paramID, uint8_t value, uint8_t paramLen);

static void ALL_Sensor_Disable(void);
/*******************************************************************************
 *                                 PROFILE CALLBACKS
 ******************************************************************************/
/*
 * Characteristic value change callback
 */
static BLEProfileCallbacks_t AP_Baseball6xsSensorCBs =
{
    AP_processBaseball6xsChangeCB,
    AP_processBaseball6xscccdCB
};
static BLEProfileCallbacks_t AP_Baseball3xsSensorCBs =
{
    AP_processBaseball3xsChangeCB,
    AP_processBaseball3xscccdCB
};
/*******************************************************************************
 *                                 PUBLIC FUNCTIONS
 ******************************************************************************/
/*******************************************************************************
 * @fn      AP_createTask
 *
 * @brief   Task creation function for the Simple BLE Peripheral.
 *
 * @param   None.
 *
 * @return  None.
 ******************************************************************************/
void AP_createTask(void)
{
    pthread_attr_t pAttrs;
    struct sched_param priParam;
    int retc;
    int detachState;

    pthread_attr_init(&pAttrs);
    priParam.sched_priority = AP_TASK_PRIORITY;

    detachState = PTHREAD_CREATE_DETACHED;
    retc = pthread_attr_setdetachstate(&pAttrs, detachState);

    if (retc != 0)
    {
        while(1);
    }

    pthread_attr_setschedparam(&pAttrs, &priParam);

    retc |= pthread_attr_setstacksize(&pAttrs, AP_TASK_STACK_SIZE);

    if (retc != 0)
    {
        while(1);
    }

    retc = pthread_create(&sensTask, &pAttrs, AP_taskFxn, NULL);

    if (retc != 0)
    {
        while(1);
    }
}

/*******************************************************************************
 * @fn      AP_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 *
 * @param   None.
 *
 * @return  None.
 ******************************************************************************/
static void AP_init(void)
{
    Timer_Params params;
    struct mq_attr attr;

    /* Create RTOS Queue */
    attr.mq_flags = 0;
    attr.mq_maxmsg = 64;
    attr.mq_msgsize = sizeof(uint32_t);
    attr.mq_curmsgs = 0;

    sensQueueRec = mq_open("SensorHub", O_RDWR | O_CREAT, 0664, &attr);
    sensQueueSend = mq_open("SensorHub", O_RDWR | O_CREAT | O_NONBLOCK, 0664, &attr);

    // Register to receive notifications from temperature if characteristics have been written to
    Baseball6xs_registerAppCBs(&AP_Baseball6xsSensorCBs);
    Baseball3xs_registerAppCBs(&AP_Baseball3xsSensorCBs);

    // Initialize characteristics and sensor driver
    Baseball6xsConfig = SENSORBSP_CFG_SENSOR_DISABLE;
    initBaseball6xsCharacteristicValue(BASEBALL6xs_DATA, 0, BASEBALL6xs_DATA_LEN);
    initBaseball6xsCharacteristicValue(BASEBALL6xs_CONF, SENSORBSP_CFG_SENSOR_DISABLE, sizeof(uint8_t));

    Baseball3xsConfig = SENSORBSP_CFG_SENSOR_DISABLE;
    initBaseball3xsCharacteristicValue(BASEBALL3xs_DATA, 0, BASEBALL3xs_DATA_LEN);
    initBaseball3xsCharacteristicValue(BASEBALL3xs_CONF, SENSORBSP_CFG_SENSOR_DISABLE, sizeof(uint8_t));

    /* Setting up the timer in continuous callback mode that calls the callback
     * functions every 5s.
     */
    Timer_Params_init(&params);
    params.period = AP_PERIODIC_EVT_PERIOD;
    params.periodUnits = TIMER_PERIOD_US;
    params.timerMode = TIMER_CONTINUOUS_CB;
    params.timerCallback = AP_timerHandler;

    timer0 = Timer_open(Board_Timer0, &params);
#ifndef DISPLAY_DISABLE
    if (timer0 == NULL)
    {
        Display_print0(displayOut, 0, 0, "Failed to initialized Timer!\n");
    }
#endif

    /* Initialize clock of Flashctrl */
    clock_init();

    /* Register Key Handler */
    GPIO_setCallback(Board_BUTTON0, (GPIO_CallbackFxn) AP_keyHandler);
    GPIO_enableInt (Board_BUTTON0);
    GPIO_setCallback(Board_BUTTON1, (GPIO_CallbackFxn) AP_bslKeyHandler);
    GPIO_enableInt (Board_BUTTON1);


    /* Write to the UART. */
#ifndef DISPLAY_DISABLE
    Display_print0(displayOut,0,0,
            "--------- Sensor Booster Pack Example ---------");
    Display_print0(displayOut,0,0,"Application Processor Initializing... ");
#endif
}

/*******************************************************************************
 * @fn      AP_taskFxn
 *
 * @brief   Application task entry point for the Simple BLE Peripheral.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 ******************************************************************************/
static void *AP_taskFxn(void *arg0)
{
    uint32_t apEvent = 0;
    struct timespec ts;
    ap_States_t state = AP_RESET;
    uint8_t enableAdv = 1;
    uint8_t disableAdv = 0;
    uint32_t prio = 0;
    /*
    uint8_t sblSatus;
    SBL_Params params;
    SBL_Image image;
    */

    /* Initialize application */
    AP_init();
#ifndef DISPLAY_DISABLE
    Display_print0(displayOut,0,0,"Done!");
#endif
    sleep(1);//test
    while(1)
    {
        switch (state)
        {
        case AP_RESET:
        {
            /* Make sure CC26xx is not in BSL */
            GPIO_write(Board_RESET, Board_LED_OFF);
            GPIO_write(Board_MRDY, Board_LED_ON);

            usleep(10000);

            GPIO_write(Board_RESET, Board_LED_ON);

            /* Initialize UART port parameters within SAP parameters */
            SAP_initParams(SAP_PORT_REMOTE_UART, &sapParams);

            sapParams.port.remote.mrdyPinID = Board_MRDY;
            sapParams.port.remote.srdyPinID = Board_SRDY;
            sapParams.port.remote.boardID = Board_UART1;

            /* Setup NP module */
            SAP_open(&sapParams);

            /* Register Application thread's callback to receive
             * asynchronous requests from the NP.
             */
            SAP_setAsyncCB(AP_asyncCB);

            /* Reset the NP, and await a powerup indication.
               Clear any pending power indications received prior to this reset
               call */
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_sec += 1;

            mq_timedreceive(sensQueueRec, (void*) &apEvent, sizeof(uint32_t), &prio,
                    &ts);

            SAP_reset();

            GPIO_clearInt(Board_BUTTON1);
            GPIO_enableInt(Board_BUTTON1);

            do
            {
                apEvent = 0;
                mq_receive(sensQueueRec, (void*) &apEvent, sizeof(uint32_t),
                           &prio);

                if (apEvent != AP_EVT_PUI)
                {
#ifndef DISPLAY_DISABLE
                    Display_printf(displayOut, 0, 0,
                                   "[bleThread] Warning! Unexpected Event %lu",
                                   apEvent);
#endif
                }
            }
            while (apEvent != AP_EVT_PUI);
            GPIO_disableInt(Board_BUTTON1);
/*
            if (apEvent == AP_EVT_BSL_BUTTON)
            {
                state = AP_SBL;
            }
*/
            if (apEvent == AP_EVT_PUI)
            {
                /* Read BD ADDR */
                SAP_setParam(SAP_PARAM_HCI, SNP_HCI_OPCODE_READ_BDADDR, 0,
                                NULL);

                /* Setup Services - Service creation is blocking so
                 * no need to pend */
                AP_initServices();

                state = AP_IDLE;
            }

        }
            break;

        case AP_START_ADV:
        {
            /* Turn on user LED to indicate advertising */
            GPIO_write(Board_LED0, Board_LED_ON);
#ifndef DISPLAY_DISABLE
            Display_print0(displayOut,0,0, "Starting advertisement... ");
#endif

            /* Setting Advertising Name */
            SAP_setServiceParam(SNP_GGS_SERV_ID, SNP_GGS_DEVICE_NAME_ATT,
                                   sizeof(snpDeviceName), snpDeviceName);

            /* Set advertising data. */
            SAP_setParam(SAP_PARAM_ADV, SAP_ADV_DATA_NOTCONN,
                    sizeof(advertData), advertData);

            /* Set Scan Response data. */
            SAP_setParam(SAP_PARAM_ADV, SAP_ADV_DATA_SCANRSP,
                    sizeof(scanRspData), scanRspData);

            /* Enable Advertising and await NP response */
            SAP_setParam(SAP_PARAM_ADV, SAP_ADV_STATE, 1, &enableAdv);
            
            do
            {
                apEvent = 0;
                mq_receive(sensQueueRec, (void*) &apEvent, sizeof(uint32_t),
                           &prio);

                if (apEvent != AP_EVT_ADV_ENB)
                {
#ifndef DISPLAY_DISABLE
                    Display_printf(displayOut, 0, 0,
                                   "[bleThread] Warning! Unexpected Event %lu",
                                   apEvent);
#endif
                }
            }
            while (apEvent != AP_EVT_ADV_ENB);
#ifndef DISPLAY_DISABLE
            Display_print0(displayOut,0,0, "Done!");
            Display_print0(displayOut,0,0,
                    "Waiting for connection (or timeout)... ");
#endif
            /* Wait for connection or button press to cancel advertisement */
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_sec += 60;
            apEvent = 0;
            mq_timedreceive(sensQueueRec, (void*) &apEvent, sizeof(uint32_t),
                            &prio, &ts);

            if(apEvent == AP_EVT_CONN_EST)
            {
                state = AP_CONNECTED;
            }
            else
            {
                state = AP_CANCEL_ADV;
#ifndef DISPLAY_DISABLE
                Display_print0(displayOut, 0, 0,"Advertisement Timeout!");
#endif
            }
        }
            break;

        case AP_CONNECTED:
            /* Before connecting, NP will send the stop ADV message */
            do
            {
                apEvent = 0;
                mq_receive(sensQueueRec, (void*) &apEvent, sizeof(uint32_t),
                           &prio);

                if (apEvent != AP_EVT_ADV_END)
                {
#ifndef DISPLAY_DISABLE
                    Display_printf(displayOut, 0, 0,
                                   "[bleThread] Warning! Unexpected Event %lu",
                                   apEvent);
#endif
                }
            }
            while (apEvent != AP_EVT_ADV_END);
#ifndef DISPLAY_DISABLE
            /* Update State and Characteristic values on LCD */
            Display_print1(displayOut,0,0,"Peer connected! (%s)", peerstr);
#endif

            /* Events that can happen during connection - Client Disconnection
                                                        - AP Disconnection */
            do
            {
                apEvent = 0;
                mq_receive(sensQueueRec, (void*) &apEvent, sizeof(uint32_t),
                           &prio);

                if (apEvent != AP_EVT_CONN_TERM)
                {
#ifndef DISPLAY_DISABLE
                    Display_printf(displayOut, 0, 0,
                                   "[bleThread] Warning! Unexpected Event %lu",
                                   apEvent);
#endif
                }
            }
            while (apEvent != AP_EVT_CONN_TERM);

            /* Client has disconnected from server */
            SAP_setParam(SAP_PARAM_CONN, SAP_CONN_STATE, sizeof(connHandle),
                    (uint8_t *) &connHandle);

            do
            {
                apEvent = 0;
                mq_receive(sensQueueRec, (void*) &apEvent, sizeof(uint32_t),
                           &prio);

                if ((apEvent != AP_EVT_CONN_TERM)
                        && (apEvent != AP_EVT_ADV_ENB))
                {
#ifndef DISPLAY_DISABLE
                    Display_printf(displayOut, 0, 0,
                                   "[bleThread] Warning! Unexpected Event %lu",
                                   apEvent);
#endif
                }
            }
            while ((apEvent != AP_EVT_CONN_TERM)
                    && (apEvent != AP_EVT_ADV_ENB));

            state = AP_CANCEL_ADV;

            break;

        case AP_CANCEL_ADV:
#ifndef DISPLAY_DISABLE
            Display_print0(displayOut,0,0,"Advertisement has been canceled!");
#endif

            /* Cancel Advertisement */
            SAP_setParam(SAP_PARAM_ADV, SAP_ADV_STATE, 1, &disableAdv);

            do
            {
                apEvent = 0;
                mq_receive(sensQueueRec, (void*) &apEvent, sizeof(uint32_t),
                           &prio);

                if (apEvent != AP_EVT_ADV_END)
                {
#ifndef DISPLAY_DISABLE
                    Display_printf(displayOut, 0, 0,
                                   "[bleThread] Warning! Unexpected Event %lu",
                                   apEvent);
#endif
                }
            }
            while (apEvent != AP_EVT_ADV_END);

            state = AP_IDLE;
            break;

        case AP_IDLE:
            /* Turn off user LED to indicate stop advertising */
            GPIO_write(Board_LED0, Board_LED_OFF);
            /*
            store_en = false;
            SensorBMP280_Deactivate();
            SensorBMA253_Deactivate();
            //SensorBHI160_Deactivate();
            SensorICM20649_Deactivate();
            Timer_stop(timer0);
            HwiP_disableInterrupt(40);
            MAP_ADC14_disableConversion();
            */
#ifndef DISPLAY_DISABLE
            Display_print0(displayOut,0,0,"State set to idle.");
#endif
            GPIO_clearInt(Board_BUTTON1);
            GPIO_enableInt(Board_BUTTON1);
            ALL_Sensor_Disable();
            Baseball6xsConfig = 0x00;
            Baseball3xsConfig = 0x00;
            /* Key Press triggers state change from idle */
/*
            do
            {
                apEvent = 0;
                mq_receive(sensQueueRec, (void*) &apEvent, sizeof(uint32_t),
                           &prio);

                if ((apEvent != AP_EVT_BUTTON_RIGHT)
                        && (apEvent != AP_EVT_BSL_BUTTON))
                {
#ifndef DISPLAY_DISABLE
                    Display_printf(displayOut, 0, 0,
                                   "[bleThread] Warning! Unexpected Event %lu",
                                   apEvent);
#endif
                }
            }
            while ((apEvent != AP_EVT_BUTTON_RIGHT)
                    && (apEvent != AP_EVT_BSL_BUTTON));
*/
            GPIO_disableInt(Board_BUTTON1);
            state = AP_START_ADV;
            /*
            if (apEvent == AP_EVT_BUTTON_RIGHT)
            {
                state = AP_START_ADV;
            }
            else if (apEvent == AP_EVT_BSL_BUTTON)
            {
                state = AP_SBL;
            }
            */
            break;

        default:
            break;
        }
    }
}

/*******************************************************************************
 * @fn      AP_initServices
 *
 * @brief   Configure SNP and register services.
 *
 * @param   None.
 *
 * @return  None.
 ******************************************************************************/
static void AP_initServices(void)
{
    Baseball3xs_addService();
    Baseball6xs_addService();

    SAP_registerEventCB(AP_processSNPEventCB, 0xFFFF);
}

/*
 * This is a callback operating in the NPI task.
 * These are events this application has registered for.
 *
 */
static void AP_processSNPEventCB(uint16_t event, snpEventParam_t *param)
{
    uint32_t eventPend;

    switch (event)
    {
    case SNP_CONN_EST_EVT:
    {
        snpConnEstEvt_t * connEstEvt = (snpConnEstEvt_t *) param;

        /* Update Peer Addr String */
        connHandle = connEstEvt->connHandle;
        ProfileUtil_convertBdAddr2Str(&peerstr[peerstrIDX], connEstEvt->pAddr);

        /* Notify state machine of established connection */
        eventPend = AP_EVT_CONN_EST;
        mq_send(sensQueueSend, (void*)&eventPend, sizeof(uint32_t), 1);
    }
        break;

    case SNP_CONN_TERM_EVT:
    {
        connHandle = AP_DEFAULT_CONN_HANDLE;
        eventPend = AP_EVT_CONN_TERM;
        mq_send(sensQueueSend, (void*)&eventPend, sizeof(uint32_t), 1);

    }
        break;

    case SNP_ADV_STARTED_EVT:
    {
        snpAdvStatusEvt_t *advEvt = (snpAdvStatusEvt_t *) param;
        if (advEvt->status == SNP_SUCCESS)
        {
            /* Notify state machine of Advertisement Enabled */
            eventPend = AP_EVT_ADV_ENB;
            mq_send(sensQueueSend, (void*)&eventPend, sizeof(uint32_t), 1);
        }
        else
        {
            eventPend = AP_ERROR;
            mq_send(sensQueueSend, (void*)&eventPend, sizeof(uint32_t), 1);
        }
    }
        break;

    case SNP_ADV_ENDED_EVT:
    {
        snpAdvStatusEvt_t *advEvt = (snpAdvStatusEvt_t *) param;
        if (advEvt->status == SNP_SUCCESS)
        {
            /* Notify state machine of Advertisement Disabled */
            eventPend = AP_EVT_ADV_END;
            mq_send(sensQueueSend, (void*)&eventPend, sizeof(uint32_t), 1);
        }
    }
        break;

    default:
        break;
    }
}

/*
 * This is a callback operating in the NPI task.
 * These are Asynchronous indications.
  */
static void AP_asyncCB(uint8_t cmd1, void *pParams)
{
    uint32_t eventPend;

    switch (SNP_GET_OPCODE_HDR_CMD1(cmd1))
    {
    case SNP_DEVICE_GRP:
    {
        switch (cmd1)
        {
        case SNP_POWER_UP_IND:
        {
            eventPend = AP_EVT_PUI;
            mq_send(sensQueueSend, (void*)&eventPend, sizeof(uint32_t), 1);
            break;
        }
        case SNP_HCI_CMD_RSP:
        {
            snpHciCmdRsp_t *hciRsp = (snpHciCmdRsp_t *) pParams;
            switch (hciRsp->opcode)
            {
            case SNP_HCI_OPCODE_READ_BDADDR:
                ProfileUtil_convertBdAddr2Str(&nwpstr[nwpstrIDX],
                        hciRsp->pData);
              default:
                break;
            }
        }
            break;

        case SNP_EVENT_IND:
        {
        }
        default:
            break;
        }
    }
        break;

    default:
        break;
    }
}

/*******************************************************************************
 * @fn      AP_bslKeyHandler
 *
 * @brief   event handler function to notify the app to program the SNP
 *
 * @param   none
 *
 * @return  none
 ******************************************************************************/
void AP_bslKeyHandler(void)
{
    uint32_t delayDebounce = 0;
    uint32_t eventPend;

    GPIO_disableInt(Board_BUTTON1);

    /* Delay for switch debounce */
    for (delayDebounce = 0; delayDebounce < 20000; delayDebounce++);

    GPIO_clearInt(Board_BUTTON1);
    GPIO_enableInt(Board_BUTTON1);

    GPIO_toggle (Board_LED1);

    //eventPend = AP_EVT_BSL_BUTTON;
    mq_send(sensQueueSend, (void*)&eventPend, sizeof(uint32_t), 1);
}

/*******************************************************************************
 * @fn      AP_keyHandler
 *
 * @brief   Key event handler function
 *
 * @param   none
 *
 * @return  none
 ******************************************************************************/
void AP_keyHandler(void)
{
    uint32_t delayDebounce = 0;
    uint32_t eventPend;

    GPIO_disableInt (Board_BUTTON0);

    /* Delay for switch debounce */
    for (delayDebounce = 0; delayDebounce < 20000; delayDebounce++)
        ;

    GPIO_clearInt(Board_BUTTON0);
    GPIO_enableInt(Board_BUTTON0);

    eventPend = AP_EVT_BUTTON_RIGHT;
    mq_send(sensQueueSend, (void*)&eventPend, sizeof(uint32_t), 1);
}

/*
 * TimerHandler for the Force Sensor
 */
static void AP_timerHandler(Timer_Handle handle)
{
    /* Toggle a new conversion */
    MAP_ADC14_toggleConversionTrigger();
}

//Callbacks for the ECG9xs characteristic

static void AP_processBaseball6xsChangeCB(uint8_t charID)
{
    Baseball6xs_processCharChangeEvt(charID);
}

static void AP_processBaseball6xscccdCB(uint8_t charID, uint16_t value)
{
    switch (PROFILE_ID_CHAR(charID))
    {
    case BASEBALL6xs_DATA:
        switch (PROFILE_ID_CHARTYPE(charID))
        {
        case PROFILE_CCCD:
            /* If indication or notification flags are set */
            if (value & (SNP_GATT_CLIENT_CFG_NOTIFY | SNP_GATT_CLIENT_CFG_INDICATE))
            {
                if(Baseball3xs == false && Baseball6xs == false)
                {
                    Baseball6xs = true;
#ifndef DISPLAY_DISABLE
                    Display_print0(displayOut,0,0,"Baseball6xs update enabled!");
#endif
                }
            }
            else
            {
                if(Baseball6xs == true)
                {
                    Baseball6xs = false;
#ifndef DISPLAY_DISABLE
                    Display_print0(displayOut,0,0,"Baseball6xs update disabled!");
#endif
                }
            }
            break;

        default:
            break;
        }
        break;

    default:
        break;
    }
}
/*******************************************************************************
 * @fn      Baseball6xs_processCharChangeEvt
 *
 * @brief   SensorBP ECG9xs event handling
 *
 ******************************************************************************/

static void Baseball6xs_processCharChangeEvt(uint8_t paramID)
{
    uint8_t newValue;
    //uint32_t sendCmd;

    switch (paramID)
    {
    case BASEBALL6xs_CONF:
        if (Baseball6xsConfig != SENSORBSP_ERROR_DATA)
        {
            Baseball6xs_getParameter(BASEBALL6xs_CONF, &newValue);

            if (newValue == SENSORBSP_CFG_SENSOR_DISABLE)
            {
                /* Reset characteristics */
                initBaseball6xsCharacteristicValue(BASEBALL6xs_DATA, 0, BASEBALL6xs_DATA_LEN);

#ifndef DISPLAY_DISABLE
                Display_print0(displayOut, 0, 0, "ADC disabled!");
#endif

            }
            else
            {
                //SensorBMP280_Activate();
                //SensorBHI160_Activate();
                /*
                Timer_start(timer0);
                HwiP_enableInterrupt(40);
                MAP_ADC14_enableConversion();
                
                */
                if(Baseball6xsConfig != 0x11 && newValue == 0x11)
                {
                    openflash();
                    SensorA301_Deactivate();
                    Timer_stop(timer0);
                    HwiP_disableInterrupt(40);
                    MAP_ADC14_disableConversion();
                    SensorICM20649_Activate(false);
                    sem_post(&BLEconnected);
                }
                else if(Baseball6xsConfig != 0x12 && newValue == 0x12)
                {
                    openflash();
                    SensorICM20649_Deactivate();
                    Timer_start(timer0);
                    HwiP_enableInterrupt(40);
                    MAP_ADC14_enableConversion();
                    SensorA301_Activate(false);
                }
                else if(Baseball6xsConfig != 0x13 && newValue == 0x13)
                {
                    outputflashdata();
                }
               /*
                else if(Baseball6xsConfig == 0x10 && newValue == 0xFF)
                {
                    outputflashdata();
                }
                */
            //    Baseball6xsConfig = newValue;
            }
        } else
        {
            /* Make sure the previous characteristics value is restored */
            initBaseball6xsCharacteristicValue(BASEBALL6xs_CONF, Baseball6xsConfig, sizeof(uint8_t));
        }

        break;

    default:
        break;
    }
}
static void initBaseball6xsCharacteristicValue(uint8_t paramID, uint8_t value, uint8_t paramLen)
{
    uint8_t data[BASEBALL6xs_DATA_LEN];

    memset(data, value, paramLen);
    Baseball6xs_setParameter(paramID, paramLen, data);
}

//Callbacks for the ECG3xs characteristic

static void AP_processBaseball3xsChangeCB(uint8_t charID)
{
    Baseball3xs_processCharChangeEvt(charID);
}

static void AP_processBaseball3xscccdCB(uint8_t charID, uint16_t value)
{
    switch (PROFILE_ID_CHAR(charID))
    {
    case BASEBALL3xs_DATA:
        switch (PROFILE_ID_CHARTYPE(charID))
        {
        case PROFILE_CCCD:
            /* If indication or notification flags are set */
            if (value & (SNP_GATT_CLIENT_CFG_NOTIFY | SNP_GATT_CLIENT_CFG_INDICATE))
            {
                if(Baseball3xs == false && Baseball6xs == false)
                {
                    Baseball3xs = true;
#ifndef DISPLAY_DISABLE
                    Display_print0(displayOut,0,0,"Baseball3xs update enabled!");
#endif
                }
            }
            else
            {
                if(Baseball3xs == true)
                {
                    Baseball3xs = false;
#ifndef DISPLAY_DISABLE
                    Display_print0(displayOut,0,0,"Baseball3xs update disabled!");
#endif
                }
            }
            break;

        default:
            break;
        }
        break;

    default:
        break;
    }
}
/*******************************************************************************
 * @fn      Baseball3xs_processCharChangeEvt
 *
 * @brief   SensorBP ECG3xs event handling
 *
 ******************************************************************************/

static void Baseball3xs_processCharChangeEvt(uint8_t paramID)
{
    uint8_t newValue;

    switch (paramID)
    {
    case BASEBALL3xs_CONF:
        if (Baseball3xsConfig != SENSORBSP_ERROR_DATA)
        {
            Baseball3xs_getParameter(BASEBALL3xs_CONF, &newValue);

            if (newValue == SENSORBSP_CFG_SENSOR_DISABLE)
            {
                /* Reset characteristics */
                initBaseball3xsCharacteristicValue(BASEBALL3xs_DATA, 0, BASEBALL3xs_DATA_LEN);

                /* Deactivate task */
                SensorBMP280_Deactivate();
                SensorBMA253_Deactivate();
            }
            else
            {
                /* Activate task */
                SensorBMP280_Activate();
                SensorBMA253_Activate();
                Baseball3xsConfig = newValue;
            }
        } else
        {
            /* Make sure the previous characteristics value is restored */
            initBaseball3xsCharacteristicValue(BASEBALL3xs_CONF, Baseball3xsConfig, sizeof(uint8_t));
        }

        break;

    default:
        break;
    }
}
static void initBaseball3xsCharacteristicValue(uint8_t paramID, uint8_t value, uint8_t paramLen)
{
    uint8_t data[BASEBALL3xs_DATA_LEN];

    memset(data, value, paramLen);
    Baseball3xs_setParameter(paramID, paramLen, data);
}

static void ALL_Sensor_Disable(void)
{
    /* Deactivate task */
    //SensorBMP280_Deactivate();
    //SensorBMA253_Deactivate();
    //SensorBHI160_Deactivate();
    SensorICM20649_Deactivate();
    SensorA301_Deactivate();
    Timer_stop(timer0);
    HwiP_disableInterrupt(40);
    MAP_ADC14_disableConversion();
    if(store_en == true)
        {
            closeflash();
            store_en = false;
        }
}
