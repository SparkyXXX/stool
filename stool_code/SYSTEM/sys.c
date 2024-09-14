#include "sys.h"

// THUMB指令不支持汇编内联
// 采用如下方法实现执行汇编指令WFI
__asm void WFI_SET(void)
{
	WFI;
}
// 关闭所有中断
__asm void INTX_DISABLE(void)
{
	CPSID I;
}
// 开启所有中断
__asm void INTX_ENABLE(void)
{
	CPSIE I;
}
// 设置栈顶地址
// addr:栈顶地址
__asm void MSR_MSP(uint32_t addr)
{
	MSR MSP, r0 // set Main Stack value
				 BX r14
}

void JTAG_Set(uint8_t mode)
{
	uint32_t temp;
	temp = mode;
	temp <<= 25;
	RCC->APB2ENR |= 1 << 0;	  // 开启辅助时钟
	AFIO->MAPR &= 0XF8FFFFFF; // 清除MAPR的[26:24]
	AFIO->MAPR |= temp;		  // 设置jtag模式
}
