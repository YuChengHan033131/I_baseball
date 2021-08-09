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

#ifndef FLASH_H_
#define FLASH_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 *                                CONSTANTS
 ******************************************************************************/
/* Task configuration */
#define FLASH_TASK_PRIORITY    1
#define FLASH_TASK_STACK_SIZE  1024//1024


//buffer configuration
#define NUMFLASH    256 //2^x, ex. 16, 32, 64

/* String conversion macro */
#define STR_(n)             #n
#define STR(n)              STR_(n)

/* Drive number used for FatFs */
//#define DRIVE_NUM           0
//#define CPY_BUFF_SIZE       2048
/*******************************************************************************
 *                                FUNCTIONS
 ******************************************************************************/
/*
 * Create the sensor task
 */
void flash_createTask(void);
void openflash(void);
void closeflash(void);
extern bool outputflashdata(uint16_t set_number);
//void stopflash(void);
void sendtoStore(uint8_t *value);
extern void flasheraseall(void);
uint16_t total_set_number(void);
void getTime(void);
int clock_init(void);
#ifdef __cplusplus
}
#endif

#endif /* FLASH_H_ */
