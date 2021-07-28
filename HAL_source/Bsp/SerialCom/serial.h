#ifndef __SERIAL_H

#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef husart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_memtomem_dma2;

void MX_Usart1Init(void);
void MX_DMA_Init(void);

#endif
