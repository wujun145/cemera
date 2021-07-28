#include "stm32f4xx_hal.h"
#include "delay.h"

static __IO uint32_t TimingDelay = 0;

__IO uint32_t fac_us = 0;
__IO uint32_t fac_ms = 0;

void delay_init(void)
{
	SystemCoreClockUpdate();
}

void delay_us(uint32_t nus)
{
	SysTick_Config(SystemCoreClock / 1000000);
	fac_us = nus;
	while(fac_us != 0);
}

void delay_ms(uint32_t nms)
{
	SysTick_Config(SystemCoreClock / 1000);
	fac_ms = nms;
	while(fac_ms != 0);
}


