/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sys.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t Pick_up_stop = 0;                                                                        // ����Ƿ������־λ
int Middle_angle = 0;                                                                            // ��е��ֵĬ��Ϊ0
uint8_t Way_Angle = 2;                                                                           // ��ȡ�Ƕȵ��㷨��1����Ԫ��  2��������  3�������˲�
uint16_t Flag_front, Flag_back, Flag_Left, Flag_Right, Flag_velocity = 2, Target_Velocity = 300; // ����ң����صı���?
float RC_Velocity, RC_Turn_Velocity;                                                             // ң�ؿ��Ƶ��ٶ�
uint8_t Flag_Stop = 1, Flag_Show = 0;                                                            // ���ֹͣ��־λ����ʾ��־�?  Ĭ��ֹͣ ��ʾ��
uint8_t PS2_ON_Flag = 0;                                                                         // Ĭ�����з�ʽ������
uint8_t Mode = 0;                                                                                // ģʽѡ��Ĭ������ͨ�Ŀ���ģʽ
float Move_X, Move_Z;                                                                            // ����С�����ϡ�����ʱǰ���ı�����ת��ı���?
uint16_t determine;                                                                              // �״����ģʽ��һ����־�?
int Encoder_Left, Encoder_Right;                                                                 // ���ұ��������������?
int Motor_Left, Motor_Right;                                                                     // ���PWM���� Ӧ��Motor�� ��Moto�¾�
int Temperature;                                                                                 // �¶ȱ���
int Voltage;                                                                                     // ��ص�ѹ������صı���
float Angle_Balance, Gyro_Balance, Gyro_Turn;                                                    // ƽ�����? ƽ�������� ת��������
uint32_t Distance;                                                                               // �״���
uint8_t delay_50, PID_Send;                                                                      // ��ʱ�͵�����ر���?
volatile uint8_t delay_flag;
uint8_t Flag_follow = 0, Flag_avoid = 0, Flag_straight = 0;                                                    // �״����?�״���ϱ�־�?
uint8_t Lidar_Detect = Lidar_Detect_ON;                                                                        // ���Ѳ��ģʽ�״����ϰ���?Ĭ�Ͽ���
float Acceleration_Z;                                                                                          // Z����ٶȼ�?
uint8_t CCD_Zhongzhi, CCD_Yuzhi;                                                                               // ����CCD���?
float Balance_Kp = 27000, Balance_Kd = 110, Velocity_Kp = 400, Velocity_Ki = 2, Turn_Kp = 4200, Turn_Kd = 100; // PID�������Ŵ�100����
uint16_t Angle_ADC = 0;
extern float p;
extern int p1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM8_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  JTAG_Set(JTAG_SWD_DISABLE);
  JTAG_Set(SWD_ENABLE);
  delay_init();
  BEEP_GPIO_Config();
  OLED_Init();
  MPU6050_initialize();
  DMP_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    if (Flag_Show == 0)
    {
      APP_Show();
      oled_show();
      //				PS2_Read();
    }
    else
    {
      DataScope();
    }
    delay_flag = 1;
    delay_50 = 0;
    while (delay_flag)
      ;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
