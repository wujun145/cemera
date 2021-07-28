#include "bsp_flash.h"

#define FLASH_TIMEOUT_VALUE       50000U /* 50 s */

/*Variable used for Erase procedure*/
static FLASH_EraseInitTypeDef EraseInitStruct;

static void Error_Handler(void);

static uint32_t GetSectorSize(uint32_t Sector);

uint32_t STMFLASH_ReadWord(uint32_t faddr)
{
	return *(volatile uint32_t*)faddr;
}

static uint32_t GetSector(uint32_t Address)
{
	uint32_t sector = 0;

	if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
	{
		sector = FLASH_SECTOR_0;  
	}
	else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
	{
		sector = FLASH_SECTOR_1;  
	}
	else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
	{
		sector = FLASH_SECTOR_2;  
	}
	else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
	{
		sector = FLASH_SECTOR_3;  
	}
	else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
	{
		sector = FLASH_SECTOR_4;  
	}
	else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
	{
		sector = FLASH_SECTOR_5;  
	}
	else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
	{
		sector = FLASH_SECTOR_6;  
	}
	else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
	{
		sector = FLASH_SECTOR_7;  
	}
	else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
	{
		sector = FLASH_SECTOR_8;  
	}
	else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
	{
		sector = FLASH_SECTOR_9;  
	}
	else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
	{
		sector = FLASH_SECTOR_10;  
	}
	else /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_11) */
	{
		sector = FLASH_SECTOR_11;
	}

	return sector;
}

void STMFLASH_Write(uint32_t WriteAddr, uint32_t *pBuffer, uint32_t NumToWrite)
{
	uint32_t FirstSector = 0, NbOfSectors = 0, Address = 0;
	uint32_t SectorError = 0;

	//flash unlock 
	HAL_FLASH_Unlock();
	
	//1st erase flash
	FirstSector = GetSector(WriteAddr);
	
	NbOfSectors = GetSector(WriteAddr + NumToWrite*4) - FirstSector + 1;
	
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.NbSectors = NbOfSectors;
	EraseInitStruct.Sector = FirstSector;
	if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
	{
		Error_Handler();
	}
	
	__HAL_FLASH_DATA_CACHE_DISABLE();
	__HAL_FLASH_INSTRUCTION_CACHE_DISABLE();
	
	__HAL_FLASH_DATA_CACHE_RESET();
	__HAL_FLASH_INSTRUCTION_CACHE_RESET();
	
	__HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
	__HAL_FLASH_DATA_CACHE_ENABLE();
	
	Address = WriteAddr;
	
	while(Address < WriteAddr + NumToWrite*4)
	{
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, *pBuffer) == HAL_OK)
		{
			Address += 4;
			pBuffer ++;
		}
		else
		{
			Error_Handler();
		}
	}
	
	HAL_FLASH_Lock();
}

void STMFLASH_Read(uint32_t ReadAddr, uint32_t *pBuffer, uint32_t NumToRead)
{
	__IO uint32_t data32 = 0 , MemoryProgramStatus = 0;
	uint32_t Address = ReadAddr;
	
	uint32_t endAddress = Address + NumToRead*4;
	while(Address < endAddress)
	{
		data32 = *(__IO uint32_t*)Address;
		*pBuffer = data32;
		
		Address += 4;
		pBuffer ++;
	}
}

static uint32_t GetSectorSize(uint32_t Sector)
{
  uint32_t sectorsize = 0x00;

  if((Sector == FLASH_SECTOR_0) || (Sector == FLASH_SECTOR_1) || (Sector == FLASH_SECTOR_2) || (Sector == FLASH_SECTOR_3))
  {
    sectorsize = 16 * 1024;
  }
  else if(Sector == FLASH_SECTOR_4)
  {
    sectorsize = 64 * 1024;
  }
  else
  {
    sectorsize = 128 * 1024;
  }  
  return sectorsize;
}

void UserFlashWrite(uint32_t address, uint32_t *data, uint32_t len)
{
	STMFLASH_Write(address, data, len);
}

static void UserFlashRead(uint32_t address, uint32_t *data, uint32_t len)
{
	STMFLASH_Read(address, data, len);
}

void TouchReadFlash(TouchSet *rst)
{
	UserFlashRead(USER_TOUCH_SET_ADDRESS, (uint32_t *)rst, sizeof(TouchSet)/4);
}

void TouchWriteFlash(TouchSet *rst)
{
	UserFlashWrite(USER_TOUCH_SET_ADDRESS, (uint32_t *)rst, sizeof(TouchSet)/4);
}

static void Error_Handler(void)
{
	printf("Flash error\r\n");
}