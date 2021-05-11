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
#ifndef TEMPSENSORTASK_H
#define TEMPSENSORTASK_H

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 *                                INCLUDES
 ******************************************************************************/
#include <stdint.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/*******************************************************************************
 *                                FUNCTIONS
 ******************************************************************************/
/*
 * Create the Force sensor task
 */
void ForceSensor_createTask(void);

/*
 * Task Event Processor for characteristic changes
 */
extern void ForceSensor_processCharChangeEvt(uint8_t paramID);
extern void SensorA301_Deactivate(void);
extern void SensorA301_Activate(bool down_sample);

/*
 * Task Event Processor for the BLE Application
 */
extern void ForceSensor_reset(void);

/*
 * Hardware interrupt for the ADC
 */
extern void Hwi_adc(uintptr_t arg0);

#define SENSOR_TASK_PRIORITY    1
#define SENSOR_TASK_STACK_SIZE  1024

#ifdef __cplusplus
}
#endif

#endif /* TEMPSENSORTASK_H */
