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
#include "BLEsend.h"

#define ADDRESS_2_BLOCK(Address)    ((uint16_t) (Address >> 18))
#define ADDRESS_2_PAGE(Address)     ((uint8_t)  ((Address >> 12) & 0x3F))
#define ADDRESS_2_COL(Address)      ((uint16_t) (Address & 0x0FFF))

extern Display_Handle displayOut;
SemaphoreP_Handle lockSem;
uint8_t EOD[12]={0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x04,0x80,0x01,0x80,0x01};

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
bool FlashECC(SPI_Handle handle,bool value);
bool FlashReadStatusRegister(SPI_Handle handle, uint8_t *rxBuf);
bool IsFlashBusy(SPI_Handle handle);
extern volatile bool flash_change_flag;  //orignal haven't
bool test_change = false;  //orignal haven't

//int_fast16_t FlashWriteEnable(SPI_Handle handle);
//int_fast16_t FlashWriteDisable(SPI_Handle handle);

static uint8_t          flashbuff[PAGE_DATA_SIZE];
static uint8_t          readbuff[PAGE_DATA_SIZE+4];
static uint8_t          page0[PAGE_DATA_SIZE+4];
static uint16_t         read_remain = 0;
static uint16_t         read_index = 4;
static uint16_t         write_buff_index = 0;
static uint_fast32_t    writeaddress = 1;
static uint_fast32_t    readaddress;
static uint32_t    readaddress_end;

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
;
    chars[0] = (uint8_t) cCMD;
    chars[1] = (uint8_t) (Addr>>16);
    chars[2] = (uint8_t) (Addr>>8);
    chars[3] = (uint8_t) (Addr);//page

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
    FlashECC(handle,false);
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
/****************************************************************
 * @fn:¡@FLASH_read
 *
 * @param:  SPI_Handle handle
 *
 * @param:  void *Array = input data pointer to store read data
 *
 * @param:  uint16_t len = length of read data in bytes
 *
 * @return: if data read is end of data then return false, otherwise true
 *
 * @caution: should only be called in output_flash_data()
 *
 * @description: read data from in order of readbuff, flash, flashbuff.
 * only when previous order are empty, then can read data from next order.
 * note: 1. readbuff and flashbuff stored in msp432
 *       2. when readbuff empty will refill from flash
 *       3. when flash empty will direct read from flashbuff
 */
bool FLASH_read(SPI_Handle handle, void *data, uint16_t len)
{
    uint16_t cnt;
    //cast all kind of data pointer to smallest length pointer uint8_t
    data = (uint8_t*)data;
    SemaphoreP_pend(lockSem, SemaphoreP_WAIT_FOREVER);

    if(read_remain < len )//don't have enough data to read in readbuff
    {
        if(readaddress <= readaddress_end)//have at least one page in flash
        {
            memcpy(data,readbuff+read_index,read_remain);
            FlashPageRead(handle, readaddress, &readbuff);
            readaddress += 1;

            cnt = len  - read_remain;
            memcpy(data+read_remain, readbuff+4, cnt);
            read_index = 4 + cnt;
            read_remain = PAGE_DATA_SIZE - cnt;;
            //Display_printf(displayOut, 0, 0, "out:refill");
        }
        else//readbuff don't have enough data(read_remain < len) and flash empty
        {
            Display_printf(displayOut, 0, 0, "shouldn't happen, data incomplete");
            return false;
        }
    }
    else//read from readbuff
    {
        memcpy(data, readbuff+read_index, len );
        read_index += len ;
        read_remain -= len ;

    }

    //check if is end of data or not
    uint8_t i;
    bool check=true;

    for(i=0;i<12;i++){
        if( ((uint8_t*)data)[i] != EOD[i]){
            check=false;
            break;
        }
    }

    SemaphoreP_post(lockSem);
    if(check){
        //reset readbuff
        read_remain = 0;
        return false;
    }else{
        return true;
    }

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

    //usleep(600);
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
    if(writeaddress > NUM_BLOCKS*NUM_PAGE_BLOCK)
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

        /*can be replace by erase all
         * if(!(writeaddress & 0x1FFFF))
            FlashBlockErase(handle,writeaddress);*/

        status = FlashPageProgram(handle, writeaddress, flashbuff, PAGE_DATA_SIZE);

        //Display_printf(displayOut, 0, 0, "STATUS = %d", status);
        writeaddress += 1;

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

/* @fn : FLASH_flush
 *
 * @brief : write flashbuff into flash no matter it's full or not
 *
 * @param :¡@SPI_Handle handle
 * */
bool FLASH_flush(SPI_Handle handle){

    FlashPageProgram(handle, writeaddress, flashbuff, PAGE_DATA_SIZE);
    write_buff_index = 0;
    return true;

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
/*******************************************
 * @fn: FlashECC(SPI_Handle handle, bool true)
 *
 * @description:enable or disable ECC,
 * each flash page will leave 64 bytes for error detection and correction
 *
 * */

bool FlashECC(SPI_Handle handle, bool value){
    uint8_t  chars[4];
    uint8_t  data;
    bool status;
    assertCS();
    chars[0]= (uint8_t) MT29F_GetFeature;
    chars[1]= OTP_REG_ADDR ;
    chars[2]= 0x00;
    status = spiTransfer(handle, data, &chars, 2);//true if success

    chars[0]= (uint8_t) MT29F_SetFeature;
    chars[1]= OTP_REG_ADDR ;
    if(value){
        //ECC enable
        chars[2]= data | 0x10;
    }else{
        //ECC disable
        chars[2]= data & 0xEF;
    }
    status = status & spiTransfer(handle, NULL, &chars, 3);//true if success
    deassertCS();
    return status;
}
/* @name : get_writeaddress
 *
 * @brief : get writeaddress store in flash page 1 and update it
 *
 * @return : true if success, false if page0 is full
 *
 * @description ¡G Page0 of flash is use to store the end address(writeaddress)
 *                  of each set of data.
 *                Each end address is stored in 3 bytes, so maximunm of 704 end address can be stored.
 *                The last end address will be 3 bytes before 0xff, since flash default is all 1 for all bit.
 *                New writeaddress will be the last end address +1.
 * */
bool get_writeaddress(SPI_Handle handle){
    //read page 0 of flash to know last write address
    FlashPageRead(handle, 0, page0);
    uint16_t i;
    //find the last record writeaddress
    for(i=4;i<=PAGE_DATA_SIZE+4;i=i+3){
        if(i==PAGE_DATA_SIZE+4){//page0 full
            Display_printf(displayOut,0,0,"page0 full",writeaddress);
            return false;
        }else if(page0[i]==0xff && page0[i+1]==0xff && page0[i+2]==0xff){
            //update writeaddress
            if(i==4){//no record writeaddress in flash
                writeaddress = 1;
            }else{
                writeaddress = (uint_fast32_t)page0[i-3]<<16 | (uint_fast32_t)page0[i-2]<<8 | (uint_fast32_t)page0[i-1];
                writeaddress+=1;
            }
            break;
        }
    }
    Display_printf(displayOut,0,0,"get_writeaddress=%d",writeaddress);//test
    return true;

}
/* @name : write_writeaddress
 *
 * @brief : store current writeaddress to flash
 *
 * @return : true if success, false if page0 full
 *
 * @description ¡G read page0 in flash, add in new writeaddress
 *              and store it back to flash
 * */
bool write_writeaddress(SPI_Handle handle){
    //read page 0 of flash to know where to write
    FlashPageRead(handle, 0, page0);


    //find the last record writeaddress
    uint16_t i;
    for(i=4;i<=PAGE_DATA_SIZE+4;i=i+3){
        if(i==PAGE_DATA_SIZE+4){//page0 full
            Display_printf(displayOut,0,0,"page0 full");
            return false;
        }else if(page0[i]==0xff && page0[i+1]==0xff && page0[i+2]==0xff){
            //write writeaddress in page0
            page0[i  ]=(uint8_t)(writeaddress>>16);
            page0[i+1]=(uint8_t)(writeaddress>>8);
            page0[i+2]=(uint8_t)writeaddress;
            break;
        }
    }

    //write page0 back to flash page 0
    FlashPageProgram(handle, 0, page0+4, PAGE_DATA_SIZE);
    Display_printf(displayOut,0,0,"write_writeaddress=%d",writeaddress);//test
    return true;
}
/* @name : output_flash_data
 *
 * @param : SPI_Handle handle
 *
 * @param : uint16_t set_number indicate which set of data to output in flash
 *
 * @return : return true if set_number exist in flash and successfully output, otherwise false
 *
 * @description ¡G read flash page0 to check the starting and ending page of data, then output
 *
 * @caution : should't be called while writing data to flash(i.e. between openflash(), closeflash())
 *            should be called after flash initialize.
 *
 * */
bool output_flash_data(SPI_Handle handle, uint16_t set_number){
    //read page 0 of flash to convert set_number to page address
    FlashPageRead(handle, 0, page0);

    //get end address of data by set_number
    readaddress_end = (uint32_t)(page0[set_number*3+4  ]<<16) |
                      (uint32_t)(page0[set_number*3+4+1]<<8) |
                      (uint32_t) page0[set_number*3+4+2];

    //check if set_number is valid
    if(readaddress_end==0x00ffffff){//invalid set_number
        return false;
    }

    //get start address of data by set_number
    if(set_number == 0){
        readaddress = 1;
    }else{
        readaddress = (uint32_t)(page0[set_number*3+4-3]<<16) |
                      (uint32_t)(page0[set_number*3+4-2]<<8) |
                      (uint32_t) page0[set_number*3+4-1];
        //previous end address +1
        readaddress += 1;
    }
    Display_printf(displayOut,0,0,"readdress:%d-%d",readaddress,readaddress_end);

    //output data in those pages
    uint8_t sendData[DATA_LEN];
    while(FLASH_read(handle, sendData,DATA_LEN)){
        enqueue(sendData);
        usleep(2200);
        /*Display_printf(displayOut,0,0,"out:%d",sendData[12]*256+sendData[13]);
        Display_printf(displayOut,0,0,"accx:%d",sendData[0]*256+sendData[1]);
        Display_printf(displayOut,0,0,"accy:%d",sendData[2]*256+sendData[3]);
        Display_printf(displayOut,0,0,"accz:%d",sendData[4]*256+sendData[5]);*/
    }

}
