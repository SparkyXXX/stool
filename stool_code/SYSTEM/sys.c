#include "sys.h"

// THUMBָ�֧�ֻ������
// �������·���ʵ��ִ�л��ָ��WFI
__asm void WFI_SET(void)
{
	WFI;
}
// �ر������ж�
__asm void INTX_DISABLE(void)
{
	CPSID I;
}
// ���������ж�
__asm void INTX_ENABLE(void)
{
	CPSIE I;
}
// ����ջ����ַ
// addr:ջ����ַ
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
	RCC->APB2ENR |= 1 << 0;	  // ��������ʱ��
	AFIO->MAPR &= 0XF8FFFFFF; // ���MAPR��[26:24]
	AFIO->MAPR |= temp;		  // ����jtagģʽ
}
