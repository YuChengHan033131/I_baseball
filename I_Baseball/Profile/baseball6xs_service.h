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
#ifndef PROFILE_Baseball6xs_SERVICE_H_
#define PROFILE_Baseball6xs_SERVICE_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "profile_util.h"

/*******************************************************************************
 *                                CONSTANTS
 ******************************************************************************/
/* Characteristic Types - These must be listed in order that they appear
                           in service */
#define BASEBALL6xs_DATA            0x00
#define BASEBALL6xs_CONF            0x01
#define BASEBALL6xs_PERI            0x02

#define BASEBALL6xs_DATA_ID         PROFILE_ID_CREATE(BASEBALL6xs_DATA,PROFILE_VALUE)
#define BASEBALL6xs_CONF_ID         PROFILE_ID_CREATE(BASEBALL6xs_CONF,PROFILE_VALUE)
#define BASEBALL6xs_PERI_ID         PROFILE_ID_CREATE(BASEBALL6xs_PERI,PROFILE_VALUE)

/* Service UUID */
#define BASEBALL6xs_SERV_UUID       0xAA80
#define BASEBALL6xs_DATA_UUID       0xAA81
#define BASEBALL6xs_CONF_UUID       0xAA82
#define BASEBALL6xs_PERI_UUID       0xAA83

/* Length of sensor data in bytes */
#define BASEBALL6xs_DATA_LEN        20

/* Data readout periods (range 100 - 2550 ms) */
#define BASEBALL6xs_MIN_UPDATE_PERIOD 100
#define BASEBALL6xs_PERIOD_RESOLUTION 10

/* Length of sensor data in bytes */
#define BASEBALL6xs_DATA_DESCR       "Baseball6xs Data"
#define BASEBALL6xs_CONFIG_DESCR     "Baseball6xs Conf."
#define BASEBALL6xs_PERIOD_DESCR     "Baseball6xs Period"


/*******************************************************************************
 *                                API FUNCTIONS
 ******************************************************************************/
/*
 * Movement_addService - Initializes the Sensor GATT Profile service
 *                      by registering GATT attributes with the GATT server.
 */
extern uint8_t Baseball6xs_addService(void);

/*
 * Movement_registerAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern uint8_t Baseball6xs_registerAppCBs(BLEProfileCallbacks_t *appCallbacks);

/*
 * Movement_setParameter - Set a Sensor GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to write
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 */
extern uint8_t Baseball6xs_setParameter(uint8_t param, uint8_t len, void *value);

/*
 * Movement_getParameter - Get a Sensor GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to read.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 */
extern uint8_t Baseball6xs_getParameter(uint8_t param, void *value);

#ifdef __cplusplus
}
#endif

#endif /* PROFILE_Baseball6xs_SERVICE_H_ */
