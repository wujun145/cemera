#ifndef __SRAM_H

#define BANK1_SRAM4_ADDR	(uint32_t)0x6C000000
#define IS62WV51216_SIZE 0x100000  //512*16/2bits = 0x100000  £¬1M×Ö½Ú
extern SRAM_HandleTypeDef hsramMy;
void FSMC_SRAM_Init(void);
uint8_t SRAM_Test(void);
void MySramInit(void);
void MySramWriteBuffer(uint8_t *pBuffer, uint32_t WriteAddr, uint32_t n);
void MySramReadBuffer(uint8_t *pBuffer, uint32_t ReadAddr, uint32_t n);
#endif