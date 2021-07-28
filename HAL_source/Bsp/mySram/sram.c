#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_fsmc.h"

#include "sram.h"

SRAM_HandleTypeDef hsramMy;
static SRAM_HandleTypeDef  SRAM_Handler;
static FMC_NORSRAM_TimingTypeDef Timing;

void MySramInit(void)
{
//  static DMA_HandleTypeDef dmaHandle;
  GPIO_InitTypeDef GPIO_Init_Structure;
//  SRAM_HandleTypeDef *hsram = & SRAM_Handler;
    
  /* Enable FMC clock */
  __HAL_RCC_FSMC_CLK_ENABLE();
  
//  /* Enable chosen DMAx clock */
//  __SRAM_DMAx_CLK_ENABLE();

  /* Enable GPIOs clock */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  
  /* Common GPIO configuration */
  GPIO_Init_Structure.Mode      = GPIO_MODE_AF_PP;
  GPIO_Init_Structure.Pull      = GPIO_PULLUP;
  GPIO_Init_Structure.Speed     = GPIO_SPEED_HIGH;
  GPIO_Init_Structure.Alternate = GPIO_AF12_FSMC;
  
  /* GPIOD configuration */
  GPIO_Init_Structure.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8     |\
                              GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 |\
                              GPIO_PIN_14 | GPIO_PIN_15;
   
  HAL_GPIO_Init(GPIOD, &GPIO_Init_Structure);

  /* GPIOE configuration */  
  GPIO_Init_Structure.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_7     |\
                              GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 |\
                              GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOE, &GPIO_Init_Structure);
  
  /* GPIOF configuration */  
  GPIO_Init_Structure.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2| GPIO_PIN_3 | GPIO_PIN_4     |\
                              GPIO_PIN_5 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOF, &GPIO_Init_Structure);
  
  /* GPIOG configuration */  
  GPIO_Init_Structure.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2| GPIO_PIN_3 | GPIO_PIN_4     |\
                              GPIO_PIN_5 | GPIO_PIN_12;
  
  HAL_GPIO_Init(GPIOG, &GPIO_Init_Structure);  

	
}
/**
  * @brief  Initializes the SRAM device.
  * @retval SRAM status
  */
void FSMC_SRAM_Init(void)
{ 
   SRAM_Handler.Instance = FMC_NORSRAM_DEVICE;
   SRAM_Handler.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
  
  /* SRAM device configuration */  
  Timing.AddressSetupTime      = 0x00;
  Timing.AddressHoldTime       = 0x00;
  Timing.DataSetupTime         = 0x08;
  Timing.BusTurnAroundDuration = 0x00;
  Timing.CLKDivision           = 0x00;
  Timing.DataLatency           = 0x00;
  Timing.AccessMode            = FSMC_ACCESS_MODE_A;
  //ʹ��NE4
  SRAM_Handler.Init.NSBank=FSMC_NORSRAM_BANK4;     					        
	//��ַ/�����߲�����
	SRAM_Handler.Init.DataAddressMux=FSMC_DATA_ADDRESS_MUX_DISABLE; 	
	//SRAM
	SRAM_Handler.Init.MemoryType=FSMC_MEMORY_TYPE_SRAM;   				    
	//16λ���ݿ��
	SRAM_Handler.Init.MemoryDataWidth=FSMC_NORSRAM_MEM_BUS_WIDTH_16; 
	//�Ƿ�ʹ��ͻ������,����ͬ��ͻ���洢����Ч,�˴�δ�õ�
	SRAM_Handler.Init.BurstAccessMode=FSMC_BURST_ACCESS_MODE_DISABLE; 
	//�ȴ��źŵļ���,����ͻ��ģʽ����������
	SRAM_Handler.Init.WaitSignalPolarity=FSMC_WAIT_SIGNAL_POLARITY_LOW;
	//�洢�����ڵȴ�����֮ǰ��һ��ʱ�����ڻ��ǵȴ������ڼ�ʹ��NWAIT
	SRAM_Handler.Init.WaitSignalActive=FSMC_WAIT_TIMING_BEFORE_WS;   
	//�洢��дʹ��
	SRAM_Handler.Init.WriteOperation=FSMC_WRITE_OPERATION_ENABLE;    
	//�ȴ�ʹ��λ,�˴�δ�õ�
	SRAM_Handler.Init.WaitSignal=FSMC_WAIT_SIGNAL_DISABLE;           	
	//��дʹ����ͬ��ʱ��
	SRAM_Handler.Init.ExtendedMode=FSMC_EXTENDED_MODE_DISABLE;        	
	//�Ƿ�ʹ��ͬ������ģʽ�µĵȴ��ź�,�˴�δ�õ�
	SRAM_Handler.Init.AsynchronousWait=FSMC_ASYNCHRONOUS_WAIT_DISABLE;	
	//��ֹͻ��д
	SRAM_Handler.Init.WriteBurst=FSMC_WRITE_BURST_DISABLE;           	  
	SRAM_Handler.Init.ContinuousClock=FSMC_CONTINUOUS_CLOCK_SYNC_ASYNC;
  
  /* SRAM controller initialization */
  MySramInit();
  HAL_SRAM_Init(& SRAM_Handler, &Timing, &Timing);
}

void MySramWriteBuffer(uint8_t *pBuffer, uint32_t WriteAddr, uint32_t n)
{
	for(; n!=0; n--)
	{
		*(__IO uint8_t*)(BANK1_SRAM4_ADDR + WriteAddr) = *pBuffer;
		WriteAddr++;
		pBuffer++;
	}
}

void MySramReadBuffer(uint8_t *pBuffer, uint32_t ReadAddr, uint32_t n)
{
	for(; n!=0; n--)
	{
		*pBuffer++ = *(__IO uint8_t*)(BANK1_SRAM4_ADDR + ReadAddr);
		ReadAddr++;
	}
}
uint8_t SRAM_Test(void)
{
  /*д�����ݼ�����*/
  uint32_t counter=0;
  
  /* 8λ������ */
  uint8_t ubWritedata_8b = 0, ubReaddata_8b = 0;  
  
  /* 16λ������ */
  uint16_t uhWritedata_16b = 0, uhReaddata_16b = 0; 
  
//  SRAM_INFO("���ڼ��SRAM����8λ��16λ�ķ�ʽ��дsram...");


  /*��8λ��ʽ��д���ݣ���У��*/
  
  /* ��SRAM����ȫ������Ϊ0 ��IS62WV51216_SIZE����8λΪ��λ�� */
  for (counter = 0x00; counter < IS62WV51216_SIZE; counter++)
  {
    *(__IO uint8_t*) (BANK1_SRAM4_ADDR + counter) = (uint8_t)0x0;
  }
  
  /* ������SRAMд������  8λ */
  for (counter = 0; counter < IS62WV51216_SIZE; counter++)
  {
    *(__IO uint8_t*) (BANK1_SRAM4_ADDR + counter) = (uint8_t)(ubWritedata_8b + counter);
  }
  
  /* ��ȡ SRAM ���ݲ����*/
  for(counter = 0; counter<IS62WV51216_SIZE;counter++ )
  {
    ubReaddata_8b = *(__IO uint8_t*)(BANK1_SRAM4_ADDR + counter);  //�Ӹõ�ַ��������
    
    if(ubReaddata_8b != (uint8_t)(ubWritedata_8b + counter))      //������ݣ�������ȣ���������,���ؼ��ʧ�ܽ����
    {
//      SRAM_ERROR("8λ���ݶ�д����");
      return 0;
    }
  }
	
  
  /*��16λ��ʽ��д���ݣ������*/
  
  /* ��SRAM����ȫ������Ϊ0 */
  for (counter = 0x00; counter < IS62WV51216_SIZE/2; counter++)
  {
    *(__IO uint16_t*) (BANK1_SRAM4_ADDR + 2*counter) = (uint16_t)0x00;
  }
  
  /* ������SRAMд������  16λ */
  for (counter = 0; counter < IS62WV51216_SIZE/2; counter++)
  {
    *(__IO uint16_t*) (BANK1_SRAM4_ADDR + 2*counter) = (uint16_t)(uhWritedata_16b + counter);
  }
  
    /* ��ȡ SRAM ���ݲ����*/
  for(counter = 0; counter<IS62WV51216_SIZE/2;counter++ )
  {
    uhReaddata_16b = *(__IO uint16_t*)(BANK1_SRAM4_ADDR + 2*counter);  //�Ӹõ�ַ��������
    
    if(uhReaddata_16b != (uint16_t)(uhWritedata_16b + counter))      //������ݣ�������ȣ���������,���ؼ��ʧ�ܽ����
    {
//     SRAM_ERROR("16λ���ݶ�д����");

      return 0;
    }
  }
  
 // SRAM_INFO("SRAM��д����������"); 
  /*���������return 1 */
  return 1;

}

