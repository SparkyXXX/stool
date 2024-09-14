/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

ADC_HandleTypeDef hadc2;

/* ADC2 init function */
void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */
	HAL_ADCEx_Calibration_Start(&hadc2);//У׼ADC����У׼�ᵼ��ADC������׼ȷ
  /* USER CODE END ADC2_Init 2 */

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspInit 0 */

  /* USER CODE END ADC2_MspInit 0 */
    /* ADC2 clock enable */
    __HAL_RCC_ADC2_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**ADC2 GPIO Configuration
    PC1     ------> ADC2_IN11
    PC5     ------> ADC2_IN15
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC2_MspInit 1 */

  /* USER CODE END ADC2_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspDeInit 0 */

  /* USER CODE END ADC2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC2_CLK_DISABLE();

    /**ADC2 GPIO Configuration
    PC1     ------> ADC2_IN11
    PC5     ------> ADC2_IN15
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_1|GPIO_PIN_5);

  /* USER CODE BEGIN ADC2_MspDeInit 1 */

  /* USER CODE END ADC2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/**************************************************************************
Function: AD sampling
Input   : ch��Channel of ADC1
Output  : AD conversion result
�������ܣ�AD����
��ڲ���: ch��ADC2 ��ͨ��
����  ֵ��ADת�����
**************************************************************************/
uint16_t Get_Adc(uint8_t ch)
{
	ADC_ChannelConfTypeDef sconfig;
	switch(ch){
		case 11:sconfig.Channel=ADC_CHANNEL_11; break;
		case 15:sconfig.Channel=ADC_CHANNEL_15; break;
	}
	sconfig.Rank=ADC_REGULAR_RANK_1;
	sconfig.SamplingTime=ADC_SAMPLETIME_239CYCLES_5;
	HAL_ADC_ConfigChannel(&hadc2,&sconfig);
	HAL_ADC_Start(&hadc2);                                //����ADC
  HAL_ADC_PollForConversion(&hadc2,10);                 //��ѯת��
	return (uint16_t)HAL_ADC_GetValue(&hadc2);	//�����ѹ���������ԭ��ͼ�򵥷������Եõ�
}

//��ȡͨ��ch��ת��ֵ��ȡtimes��,Ȼ��ƽ��
//ch:ͨ�����
//times:��ȡ����
//����ֵ:ͨ��ch��times��ת�����ƽ��ֵ
uint16_t Get_Adc_Average(uint8_t ch,uint8_t times)
{
	uint32_t temp_val=0;
	uint8_t t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch);
		delay_us(1);
	}
	return temp_val/times;
}
/**************************************************************************
Function: Read battery voltage
Input   : none
Output  : Battery voltage��MV��
�������ܣ���ȡ��ص�ѹ
��ڲ���: ��
����  ֵ����ص�ѹ ��λMV
**************************************************************************/
int Get_battery_volt(void)
{
   	int Volt;
	  Volt =Get_Adc(Battery_Ch)*3.3*11*100/4096;	//�����ѹ���������ԭ��ͼ�򵥷������Եõ�
	  return Volt;	                                      	//�������һ��ADC1�������ת�����
}
/* USER CODE END 1 */
