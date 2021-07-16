//#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include <ti/drivers/dpl/ClockP.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/dpl/SemaphoreP.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/display/Display.h>

#include "MT29Flash.h"
#include "../Board.h"

#define ADDRESS_2_BLOCK(Address)    ((uint16_t) (Address >> 18))
#define ADDRESS_2_PAGE(Address)     ((uint8_t)  ((Address >> 12) & 0x3F))
#define ADDRESS_2_COL(Address)      ((uint16_t) (Address & 0x0FFF))

extern Display_Handle displayOut;
SemaphoreP_Handle lockSem;

//SPI_Handle SDSPI_open(SPI_Handle handle, SD_Params *params);
/*
static int_fast16_t  FLASH_control(SPI_Handle handle, uint_fast16_t cmd, void *arg);
static uint_fast32_t FLASH_getNumPages(SPI_Handle handle);
static uint_fast32_t FLASH_getPageSize(SPI_Handle handle);
static int_fast16_t FLASH_initialize(SPI_Handle handle);
*/

void FLASH_close(SPI_Handle handle);
static inline void assertCS(void);
static inline void deassertCS(void);

static int_fast16_t spiTransfer(SPI_Handle handle, void *rxBuf, void *txBuf, size_t count);
static void Build_Column_Stream(uint_fast32_t Addr, uint8_t cCMD, uint8_t *chars);
static void Build_Row_Stream(uint_fast32_t Addr, uint8_t cCMD, uint8_t *chars);

bool waitUntilReady(SPI_Handle handle);
bool FlashReadStatusRegister(SPI_Handle handle, uint8_t *rxBuf);
bool IsFlashBusy(SPI_Handle handle);
extern volatile bool flash_change_flag;  //orignal haven't
bool test_change = false;  //orignal haven't

//int_fast16_t FlashWriteEnable(SPI_Handle handle);
//int_fast16_t FlashWriteDisable(SPI_Handle handle);

static uint8_t          flashbuff[PAGE_SIZE];
static uint8_t          readbuff[PAGE_DATA_SIZE+4];
static uint8_t          readtemp[6];  //orignal haven't
static uint16_t         read_remain = 0;
static uint16_t         read_index = 4;
static uint16_t         write_buff_index = 0;
static uint_fast32_t    wirteaddress = 0;
static uint_fast32_t    readaddress = 0;

bool FlashReadDeviceIdentification(SPI_Handle handle, uint16_t *uwpDeviceIdentification)
{
    uint8_t  chars[4];
    uint8_t  pIdentification[4];
    assertCS();
    chars[0]= (uint8_t) MT29F_ReadDeviceID;
    chars[1]= 0x00; // dummy byte
    chars[2]= 0x00;
    chars[3]= 0x00;

    spiTransfer(handle, &pIdentification, &chars, 4);

    deassertCS();
    // Step 3: Device Identification is returned ( memory type + memory capacity )
    *uwpDeviceIdentification = pIdentification[2];
    *uwpDeviceIdentification <<= 8;
    *uwpDeviceIdentification |= pIdentification[3];

    return Flash_Success;
}

bool FlashReset(SPI_Handle handle)
{
    uint8_t    txBuf = MT29F_Reset;

    if(spiTransfer(handle, NULL, &txBuf, 1) != Flash_Success)
        return false;
    else
        return true;
}
bool FlashUnlockAll(SPI_Handle handle)
{
    uint8_t chars[3];

    chars[0]= (uint8_t) MT29F_SetFeature;
    chars[1]= (uint8_t) BLKLOCK_REG_ADDR;
    chars[2]= (uint8_t) 0x00; // unlock the whole flash

    if(spiTransfer(handle, NULL, &chars, 3) != Flash_Success)
        return false;
    else
        return true;
}
int_fast16_t FlashBlockErase(SPI_Handle handle, uint_fast32_t udBlockAddr)
{
    //CharStream char_stream_send;
    uint8_t         chars[4];
    uint8_t         status_reg;
    int_fast16_t    status = Flash_Success;

    // Step 1: Check whether any previous Write, Program or Erase cycle is on going
    if(IsFlashBusy(handle)) return Flash_OperationOngoing;

    // Step 2: Enable WE
    FlashWriteEnable(handle);
    FlashReadStatusRegister(handle, &status_reg);

    if (status_reg != SPI_NAND_WE)
        status = Flash_BlockEraseFailed;
    // Step 3: Initialize the data (Instruction & address) packet to be sent serially
    Build_Row_Stream(udBlockAddr, MT29F_BlockErase, chars);

    // Step 4: Send the packet (Instruction & address) serially
    assertCS();
    spiTransfer(handle, NULL, &chars, 4);
    deassertCS();

    // Step 5: Wait until the operation completes or a timeout occurs.
    if (!waitUntilReady(handle))
        status = Flash_OperationTimeOut;

    // Step 6: Check if the erase fails
    FlashReadStatusRegister(handle, &status_reg);
    if (status_reg & SPI_NAND_EF)
        status = Flash_BlockEraseFailed;


    return (status);
}

bool FlashReadStatusRegister(SPI_Handle handle, uint8_t *rxBuf)
{
    uint8_t      txBuf[3] = {0xff};
    uint8_t      revBuf[3];

    txBuf[0]= MT29F_GetFeature;
    txBuf[1]= STATUS_REG_ADDR;

    assertCS();
    if(spiTransfer(handle, &revBuf, &txBuf, 3) != Flash_Success)//SD_STATUS_SUCCESS = 0)
    {
        deassertCS();
        return false;
    }
    else
    {
        deassertCS();
        *rxBuf = revBuf[2];
        return true;
    }
}

bool IsFlashBusy(SPI_Handle handle)
{
    uint8_t ucSR;

    /* Step 1: Read the Status Register */
    FlashReadStatusRegister(handle, &ucSR);

    /* Step 2: Check the WIP bit */
    if(ucSR & SPI_NAND_OIP)
        return true;
    else
        return false;
}

int_fast16_t FlashWriteEnable(SPI_Handle handle)
{
    uint8_t  cWREN = MT29F_WriteEnable;
    uint8_t  ucSR;

    assertCS();
    spiTransfer(handle, &ucSR, &cWREN, 1);
    deassertCS();
    //do {
    //    FlashReadStatusRegister(handle, &ucSR);
    //} while(ucSR != SPI_NAND_WE);//while(~ucSR & SPI_NAND_WE);

    return Flash_Success;
}

int_fast16_t FlashWriteDisable(SPI_Handle handle)
{
    uint8_t  cWRDI = MT29F_WriteDisable;
    assertCS();
    // Step 1: Send the packet serially
    spiTransfer(handle, NULL, &cWRDI, 1);
    deassertCS();
    // Step 2: Read the Status Register.
    //do {
    //    FlashReadStatusRegister(handle, &ucSR);
    //} while(ucSR & SPI_NAND_WE);

    return Flash_Success;
}

static void Build_Column_Stream(uint_fast32_t Addr, uint8_t cCMD, uint8_t *chars)
{
    /*
    uint_fast32_t udAddr = Addr & 0x0FFF;
    chars[0] = (uint8_t) cCMD;
    //chars[1] = ( (((uint8_t) ((udAddr>>8) & 0x3))) | ((uint8_t) ((udAddr>>18) & 0x1)) );
    chars[1] = (uint8_t) (udAddr>>8);
    chars[2] = (uint8_t) (udAddr);
    */
    chars[0] = (uint8_t) cCMD;
    chars[1] = 0x00;//(uint8_t) (udAddr>>8);
    chars[2] = 0x00;//(uint8_t) (udAddr);
}
static void Build_Row_Stream(uint_fast32_t Addr, uint8_t cCMD, uint8_t *chars)
{
    /*
    uint_fast32_t udAddr = Addr & 0xFFFFF000; // row mask ///ul
    chars[0] = (uint8_t) cCMD;
    chars[1] = (uint8_t) (udAddr>>28); //8 dummy bits + 16 when page read, 6 dummy bits + 18 when page program??
    chars[2] = (uint8_t) (udAddr>>20);
    chars[3] = (uint8_t) (udAddr>>12);
    */

    uint_fast32_t udAddr = Addr & 0xFFFFF800;
    chars[0] = (uint8_t) cCMD;
    chars[1] = (uint8_t) (udAddr>>27);
    chars[2] = (uint8_t) (udAddr>>19);
    chars[3] = (uint8_t) (udAddr>>11);//page

}

void FLASH_close(SPI_Handle handle)
{
    //SDSPI_Object *object = handle->object;
    if (handle) {
        SPI_close(handle);
        handle = NULL;
    }
/*
    if (lockSem) {
        SemaphoreP_delete(lockSem);
        lockSem = NULL;
    }
*/
}

uint_fast32_t FLASH_getNumPages(SPI_Handle handle) //in flash the minmum size of read/write is page, not sector
{
    return (NUM_PAGE_BLOCK * NUM_BLOCKS);
}

uint_fast32_t FLASH_getPageSize(SPI_Handle handle)
{
    return (PAGE_DATA_SIZE);
}

void FLASH_initialize(SPI_Handle handle)
{
    //int_fast16_t         status;
    //SPI_Params           spiParams;
    //uint16_t             fid;

    lockSem = SemaphoreP_createBinary(1);
    //if (lockSem == NULL) {
        //object->isOpen = false;
        //return (NULL);
    //}

    SemaphoreP_pend(lockSem, SemaphoreP_WAIT_FOREVER);


    /*
    SPI_Params_init(&spiParams);
    //spiParams.frameFormat = SPI_POL0_PHA0;
    spiParams.bitRate = 2500000;//2.5M
    handle = SPI_open(Board_SPI0, &spiParams);
    status = (handle == NULL) ? Flash_Error : Flash_Success;

    usleep(250);//a minimum of 250us must elapse before issuing a RESET (FFh) command
    */

    assertCS();
    FlashReset(handle);
    deassertCS();

    usleep(1000);//1ms must elapse before issuing any other command

    //FlashReadDeviceIdentification(handle,&fid);

    assertCS();
    FlashUnlockAll(handle);
    deassertCS();
    usleep(250);
    SemaphoreP_post(lockSem);

    //return (status);
}

int_fast16_t FlashPageRead(SPI_Handle handle, uint_fast32_t udAddr, void *pArray)
{
    int_fast16_t status = Flash_Success;
    uint8_t chars[4];

    //SemaphoreP_pend(lockSem, SemaphoreP_WAIT_FOREVER);

    // Step 2: Initialize the data (i.e. Instruction) packet to be sent serially
    Build_Row_Stream(udAddr, MT29F_PageRead, chars);
    //Display_printf(displayOut, 0, 0, "page address = %02x", chars[3]);
    // Step 3: Send the packet serially, and fill the buffer with the data being returned
    assertCS();
    spiTransfer(handle, NULL, &chars, 4);
    deassertCS();
    usleep(2000);//tRD=80us
    // Step 4: Wait until the operation completes or a timeout occurs.
    if (!waitUntilReady(handle))
        status = Flash_OperationTimeOut;

    // Step 5: Initialize the data (i.e. Instruction) packet to be sent serially
    //Build_Column_Stream(udAddr, MT29F_ReadCacheIns, chars);
    chars[0] = MT29F_ReadCacheIns;
    chars[1] = 0x00;//dummy byte
    chars[2] = 0x00;//(uint8_t)(Addr>>8);
    chars[3] = 0x00;//(uint8_t)(Addr);

    // Step 6: Send the packet serially, and fill the buffer with the data being returned
    assertCS();
    status = spiTransfer(handle, pArray, &chars, PAGE_DATA_SIZE+4);//write command and read
    deassertCS();

    //SemaphoreP_post(lockSem);

    return status;

}

bool FLASH_read(SPI_Handle handle, void *Array, uint16_t count)
{
    bool empty = false;
    int_fast16_t status = Flash_Error;
    uint8_t      i,cnt;

    SemaphoreP_pend(lockSem, SemaphoreP_WAIT_FOREVER);

    //Display_printf(displayOut, 0, 0, "bf read_remain = %d, bf read_index = %d", read_remain, read_index);

    if(read_remain < count )
    {
        if(readaddress < wirteaddress)
        {
            memcpy(Array,readbuff+read_index,read_remain);
            status = FlashPageRead(handle, readaddress, &readbuff);
            readaddress += PAGE_DATA_SIZE;
/*
            for(i = 0; i<6; i++)
            {
                readtemp[i] = readbuff[i+1025];
            }
            Display_printf(displayOut, 0, 0, "%s", readbuff+4);
            Display_printf(displayOut, 0, 0, "%s", readtemp);
            Display_printf(displayOut, 0, 0, "%s", readbuff+1031);
*/

            cnt = count  - read_remain;
            memcpy(Array+read_remain, readbuff+4, cnt);
            read_index = 4 + cnt;
            read_remain = PAGE_DATA_SIZE - cnt;;
            //Display_printf(displayOut, 0, 0, "out:refill");
            empty = false;
        }
        else
        {
            read_remain = 0;
            empty = true;
        }
    }
    else
    {
        memcpy(Array, readbuff+read_index, count );
        read_index += count ;
        read_remain -= count ;
        empty = false;
    }

    //Display_printf(displayOut, 0, 0, "af read_remain = %d, af read_index = %d", read_remain, read_index);
/*
    while(readaddress < wirteaddress)
    {
        status = FlashPageRead(handle, readaddress, &readbuff);
        readaddress += PAGE_DATA_SIZE;

        for(i = 0; i<6; i++){
            readtemp[i] = readbuff[i+1025];
        }
        Display_printf(displayOut, 0, 0, "%s", readbuff+4);
        Display_printf(displayOut, 0, 0, "%s", readtemp);
        Display_printf(displayOut, 0, 0, "%s", readbuff+1031);
    }
*/
    SemaphoreP_post(lockSem);

    return (empty);
}

int_fast16_t FlashPageProgram(SPI_Handle handle, uint_fast32_t udAddr,const void *pArray, uint_fast32_t udNrOfElementsInArray)
{
    int_fast16_t    status = Flash_Success;
    uint8_t         chars[4];
    uint8_t         status_reg = 0;

    //SemaphoreP_pend(lockSem, SemaphoreP_WAIT_FOREVER);

    // Step 2: Check whether any previous Write, Program or Erase cycle is on going
    if (!waitUntilReady(handle))
        return Flash_OperationOngoing;

    // Step 4: Initialize the data (Instruction & address only) packet to be sent serially
    Build_Column_Stream(udAddr, MT29F_ProgramLoad, chars);//MT29F_ProgramLoad
    // Step 5: Send the packet serially, and fill the buffer with the data being returned
    assertCS();
    status = spiTransfer(handle, NULL, &chars, 3);
    // Step 6: Send the packet (data to be programmed) serially
    status = spiTransfer(handle, NULL, (void *)pArray, udNrOfElementsInArray);
    deassertCS();
    // Step 7: Disable Write protection
    FlashWriteEnable(handle);
    // Step 8: Initialize the data (i.e. Instruction) packet to be sent serially
    Build_Row_Stream(udAddr, MT29F_ProgramExecute, chars);
    //Display_printf(displayOut, 0, 0, "page address, char[3] is %02x", chars[3]);
    // Step 9: Send the packet (data to be programmed) serially
    assertCS();
    status = spiTransfer(handle, NULL, &chars, 4);
    deassertCS();

    usleep(600);
    // Step 9: Wait until the operation completes or a timeout occurs.
    if (!waitUntilReady(handle))
        status = Flash_OperationTimeOut;

    FlashWriteDisable(handle);

    // Step 10: Check if the program fails
    FlashReadStatusRegister(handle, &status_reg);
    if (status_reg & SPI_NAND_PF)
        status =  Flash_ProgramFailed;

    //SemaphoreP_post(lockSem);
    //Display_printf(displayOut, 0, 0, "write one page");
    return status;
}

int_fast16_t FLASH_write(SPI_Handle handle, const void *buf, uint16_t count)
{
    int_fast16_t    status = Flash_Success;

    uint16_t        writebytes = 0;
    uint16_t        remainbytes = 0;

    if(count > PAGE_DATA_SIZE)
        return Flash_AddressInvalid;
    if(wirteaddress > NUM_BLOCKS*NUM_PAGE_BLOCK*(PAGE_DATA_SIZE-1))
        return Flash_AddressInvalid;

    SemaphoreP_pend(lockSem, SemaphoreP_WAIT_FOREVER);
    if((write_buff_index + count) < PAGE_DATA_SIZE)
    {
        memcpy(flashbuff+write_buff_index, buf, sizeof(uint8_t)*count);
        write_buff_index += count;
        //Display_printf(displayOut, 0, 0, "if count: %d", count);
        //Display_printf(displayOut, 0, 0, "if write_buff_index: %d", write_buff_index);
    }
    else
    {
        writebytes  = PAGE_DATA_SIZE - write_buff_index;
        //Display_printf(displayOut, 0, 0, "writebytes: %d", writebytes);

        remainbytes = count + write_buff_index - PAGE_DATA_SIZE;
        //Display_printf(displayOut, 0, 0, "remainbytes: %d", remainbytes);

        memcpy(flashbuff+write_buff_index, buf, sizeof(uint8_t)*writebytes);

        if(!(wirteaddress & 0x1FFFF))
            FlashBlockErase(handle,wirteaddress);

        //Display_printf(displayOut, 0, 0, "page = %d", wirteaddress/2048);
        //Display_printf(displayOut, 0, 0, " current data:%s", buf);
        status = FlashPageProgram(handle, wirteaddress, flashbuff, PAGE_DATA_SIZE);

        //Display_printf(displayOut, 0, 0, "STATUS = %d", status);
        wirteaddress += PAGE_DATA_SIZE;

        if(remainbytes > 0)
        {
            memcpy(flashbuff, (uint8_t*)buf+writebytes, sizeof(uint8_t)*remainbytes);
            //write_buff_index = remainbytes;

        }
        write_buff_index = remainbytes;
    }
    SemaphoreP_post(lockSem);
    return status;
}

static inline void assertCS(void)
{
    GPIO_write(FLASH_nCS, 0);  //orignal have
}

static inline void deassertCS(void)
{
    GPIO_write(FLASH_nCS, 1);  //orignal have
}

static int_fast16_t spiTransfer(SPI_Handle handle, void *rxBuf,
    void *txBuf, size_t count) {
    int_fast16_t    status;
    SPI_Transaction transaction;

    transaction.rxBuf = rxBuf;
    transaction.txBuf = txBuf;
    transaction.count = count;

    status = (SPI_transfer(handle, &transaction)) ? //0: FAIL 1: SUCCESS, Flash_Success = 0
        Flash_Success : Flash_Error;

    return (status);
}


//Function to check if the FLASH is busy.
bool waitUntilReady(SPI_Handle handle)
{
    uint32_t     currentTime;
    uint32_t     startTime;
    uint32_t     timeout;

    /* Wait up to 1s for data packet */
    timeout = 1000000/ClockP_getSystemTickPeriod();
    startTime = ClockP_getSystemTicks();

    while(IsFlashBusy(handle))
    {
        currentTime = ClockP_getSystemTicks();
        if((currentTime - startTime) > timeout)
            return false;
    }

    return true;
}
