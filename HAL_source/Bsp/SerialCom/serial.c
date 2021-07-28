#include "serial.h"

#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
  
UART_HandleTypeDef husart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_memtomem_dma2;

void HAL_UART_MspInit(UART_HandleTypeDef* huart);
static void TranslateOK(DMA_HandleTypeDef *dma);
static void TranslateErr(DMA_HandleTypeDef *dma);

void MX_Usart1Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_USART1_CLK_ENABLE();

	GPIO_InitStructure.Pin = GPIO_PIN_9| GPIO_PIN_10;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStructure.Alternate = GPIO_AF7_USART1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	husart1.Instance = USART1;
	husart1.Init.BaudRate = 115200;
	husart1.Init.WordLength = UART_WORDLENGTH_8B;
	husart1.Init.StopBits = UART_STOPBITS_1;
	husart1.Init.Parity = UART_PARITY_NONE;
	husart1.Init.Mode = UART_MODE_TX_RX;
	husart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	husart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if(HAL_UART_Init(&husart1) != HAL_OK)
	{
		Error_Handler();
	}
}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
	if(huart->Instance == USART1)
	{
		hdma_usart1_tx.Instance = DMA2_Stream7;
		hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
		hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_usart1_tx.Init.Mode = DMA_NORMAL;
		hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
		hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
		hdma_usart1_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
		hdma_usart1_tx.Init.MemBurst = DMA_MBURST_SINGLE;
		hdma_usart1_tx.Init.PeriphBurst = DMA_PBURST_SINGLE;
		if(HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
		{
			Error_Handler();
		}

		__HAL_LINKDMA(huart, hdmatx, hdma_usart1_tx);

		HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
	}
}

void MX_DMA_Init(void)
{
	/* DMA controller clock enable */
  	__HAL_RCC_DMA2_CLK_ENABLE();

	hdma_memtomem_dma2.Instance = DMA2_Stream0;
	hdma_memtomem_dma2.Init.Channel = DMA_CHANNEL_0;
	hdma_memtomem_dma2.Init.PeriphInc = DMA_PINC_ENABLE;
	hdma_memtomem_dma2.Init.Direction = DMA_MEMORY_TO_MEMORY;
	hdma_memtomem_dma2.Init.MemInc = DMA_MINC_ENABLE;
	hdma_memtomem_dma2.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	hdma_memtomem_dma2.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
	hdma_memtomem_dma2.Init.Mode = DMA_NORMAL;
	hdma_memtomem_dma2.Init.Priority = DMA_PRIORITY_HIGH;
	hdma_memtomem_dma2.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
	hdma_memtomem_dma2.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	hdma_memtomem_dma2.Init.MemBurst = DMA_MBURST_SINGLE;
	hdma_memtomem_dma2.Init.PeriphBurst = DMA_PBURST_SINGLE;
//	hdma_memtomem_dma2.XferCpltCallback = TranslateOK;
//	hdma_memtomem_dma2.XferErrorCallback = 	TranslateErr;
	if(HAL_DMA_Init(&hdma_memtomem_dma2) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_DMA_RegisterCallback(&hdma_memtomem_dma2, HAL_DMA_XFER_CPLT_CB_ID, TranslateOK);
	HAL_DMA_RegisterCallback(&hdma_memtomem_dma2, HAL_DMA_XFER_ERROR_CB_ID, TranslateErr);
	
  	/* DMA interrupt init */
 	/* DMA2_Stream7_IRQn interrupt configuration */
  	HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  	HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
}

static void TranslateOK(DMA_HandleTypeDef *dma)
{
	HAL_UART_Transmit(&husart1, "dma translate OK\r\n", 18, 0xff);
}

static void TranslateErr(DMA_HandleTypeDef *dma)
{
	HAL_UART_Transmit(&husart1, "dma translate NONE\r\n", 20, 0xff);
}


/*remap printf() to usart1*/
//int fputc(int c, FILE *f)
//{
//	HAL_UART_Transmit(&husart1, (uint8_t*)&c, 1, 0xffff);
//	return c;
//}

//int fgetc(FILE * F)
//{
//	uint8_t ch_r;
//	HAL_UART_Receive(&husart1, &ch_r, 1, 0xffff);
//	return ch_r;
//}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&husart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}
