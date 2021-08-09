
#ifndef MT29FLASH__include
#define MT29FLASH__include

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <ti/drivers/SPI.h>

#define Flash_Error               (-1)
#define SD_STATUS_UNDEFINEDCMD    (-2)

/* Definitions for SPI Flash command */
#define MT29F_BlockErase        0xD8
#define MT29F_GetFeature        0x0F
#define MT29F_PageRead          0x13
#define MT29F_ProgramExecute    0x10
#define MT29F_ProgramLoad       0x02
#define MT29F_ProgramLoadRD     0x84 //ProgramLoad random data
#define MT29F_ReadCacheIns      0x03
#define MT29F_ReadCacheIns2     0x0B
#define MT29F_ReadCacheX2Ins    0x3B
#define MT29F_ReadCacheX4Ins    0x6B
#define MT29F_ReadDeviceID      0x9F
#define MT29F_Reset             0xFF
#define MT29F_SetFeature        0x1F
#define MT29F_WriteEnable       0x06
#define MT29F_WriteDisable      0x04
#define Dummy_Byte              0xFF
#define BLKLOCK_REG_ADDR        0xA0
#define OTP_REG_ADDR            0xB0
#define STATUS_REG_ADDR         0xC0

#define FLASH_WIDTH             8               /* Flash data width */
#define DATA_LEN                20
#define PAGE_DATA_SIZE          2112            /* Page data size in bytes , can set to 2176 if ECC disable*/
#define NUM_BLOCKS              2048            /* Number of blocks*/
#define NUM_PAGE_BLOCK          64              /* Number of pages for block*/
//#define NUM_PLANES              1               /* Number of planes*/
#define FLASH_SIZE              PAGE_DATA_SIZE*NUM_PAGE_BLOCK*NUM_BLOCKS

extern uint8_t EOD[12];

bool waitUntilReady(SPI_Handle handle);
bool FlashReadStatusRegister(SPI_Handle handle, uint8_t *rxBuf);
uint_fast32_t FLASH_getNumPages(SPI_Handle handle);
uint_fast32_t FLASH_getPageSize(SPI_Handle handle);
bool FlashReadDeviceIdentification(SPI_Handle handle, uint16_t *uwpDeviceIdentification);

void  FLASH_initialize(SPI_Handle handle);
int_fast16_t  FlashBlockErase(SPI_Handle handle, uint_fast32_t udBlockAddr);

int_fast16_t FlashPageRead(SPI_Handle handle, uint_fast32_t udAddr, void *pArray);
int_fast16_t FlashPageProgram(SPI_Handle handle, uint_fast32_t udAddr,const void *pArray, uint_fast32_t udNrOfElementsInArray);
bool FLASH_read(SPI_Handle handle, void *data, uint16_t len);
int_fast16_t FLASH_write(SPI_Handle handle, const void *buf, uint16_t count);
bool FLASH_flush(SPI_Handle handle);

int_fast16_t FlashWriteEnable(SPI_Handle handle);
int_fast16_t FlashWriteDisable(SPI_Handle handle);
bool get_writeaddress(SPI_Handle handle);
bool write_writeaddress(SPI_Handle handle);
bool output_flash_data(SPI_Handle handle, uint16_t set_number);

enum {
    Flash_Success = 0,
    Flash_AddressInvalid,
	Flash_BlockEraseFailed,
    /*
    Flash_RegAddressInvalid,
    Flash_MemoryOverflow,
    Flash_PageNrInvalid,
    Flash_SubSectorNrInvalid,
    Flash_SectorNrInvalid,
    Flash_FunctionNotSupported,
    Flash_NoInformationAvailable,
    */
    Flash_OperationOngoing,
    Flash_OperationTimeOut,
    Flash_ProgramFailed,
    /*
    Flash_SectorProtected,
    Flash_SectorUnprotected,
    Flash_SectorProtectFailed,
    Flash_SectorUnprotectFailed,
    Flash_SectorLocked,
    Flash_SectorUnlocked,
    Flash_SectorLockDownFailed,
    Flash_WrongType
    */
};

enum
{
    SPI_NAND_PF   = 0x08, /* program fail */
    SPI_NAND_EF   = 0x04, /* erase fail */
    SPI_NAND_WE   = 0x02, /* write enable */
    SPI_NAND_OIP  = 0x01, /* operation in progress */
};

#ifdef __cplusplus
}
#endif

#endif /* MT29FLASH__include */
