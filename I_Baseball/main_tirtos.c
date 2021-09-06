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

#include <sensor_boosterpack_A301.h>
#include <stdint.h>
#include <ti/display/Display.h>
#include <ti/sysbios/BIOS.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/Timer.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/SPI.h>
#include <semaphore.h>
#include "Board.h"
#include "flashctrl.h"
#include "BLEsend.h"
#include "sensor_boosterpack_bhi160.h"
#include "sensor_boosterpack_bma253.h"
#include "sensor_boosterpack_bmp280.h"
#include "sensor_boosterpack_icm20649.h"
#include "sensor_boosterpack.h"
#include "sensor_configuration.h"
#include "test.h"
#include "GATTwriteHandler.h"

/* Output display handle that will be used to print out all debug/log
 * statements
 */
Display_Handle displayOut;




sem_t BLEconnected;
void main()
{
    /* Call board initialization functions */
    Power_init();
    GPIO_init();
    UART_init();
    Timer_init();
    I2C_init();
    SPI_init();

    sem_init(&BLEconnected,1,0);

    /*display through UART*/
    displayOut = Display_open(Display_Type_UART, NULL);

    if(!SensorConfig_initI2C())
    {
        //while (1);
    }

    //flash_createTask();
    BLEsend_createTask();
    //ForceSensor_createTask();
    //SensorBMP280_createTask();
    //SensorBHI160_createTask();
    //SensorBMA253_createTask();
    //SensorICM20649_createTask();
    //test_SensorICM20649_createTask();
    GATTwriteHandler_createTask();

    //test_createTask();

    AP_createTask();

    /* Enable interrupts and start SYS/BIOS */
    BIOS_start();
}

