#ifndef __SRAM_H

#define BANK1_SRAM3_ADDR	(uint32_t)0x68000000

void MySramInit(void);
void MySramWriteBuffer(uint8_t *pBuffer, uint32_t WriteAddr, uint32_t n);
void MySramReadBuffer(uint8_t *pBuffer, uint32_t ReadAddr, uint32_t n);
#endif