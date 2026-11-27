/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "global.h"
#include "stdio.h"
#include "stdlib.h"
#include "led.h"
#include "buzzer.h"
#include "adc_sample.h"
#include "app_porting.h"
#include "ps2_porting.h"
#include "adc_sample.h"
#include "oled.h"
#include "serial_servo.h"
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
float voltage;
uint8_t arm_select;
uint8_t func_state;
uint8_t button_state;
uint8_t last_button_state;
uint8_t read_servos_id = 0;
uint8_t action_group_state1 = 0;
uint8_t action_group_state2 = 0;
uint32_t servo_init_tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch,FILE *f)
{
	/* 采用轮询方式发鿿1字节数据，超时时间设置为无限等待 */
	HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,HAL_MAX_DELAY);
	return ch;
}

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
  __HAL_RCC_I2C1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();  
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  led_init();
  buzzer_init();
  adc_sample_init();
  
  serial_servo_init();
  HAL_Delay(500);
  servo_init_tick = HAL_GetTick();
  while(!serial_servo_read_id(&serial_servo_controller, 1, &read_servos_id))
  {
	  if(HAL_GetTick() - servo_init_tick >= 200)
	  {
		  break;
	  }		  
  }
  
  if(read_servos_id == 1)
  {
	  buzzer_on();
	  HAL_Delay(400);
	  buzzer_off();
	  arm_select = 2;
  }
  else
  {
	  arm_select = 1;
  }

  if(robot_arm_init() == false)
  {
	 led_flash(2, 50, 50, 0);
	 while(1);
  }
  
  app_init();
  ps2_init();
  led_flash(1, 2000, 2000, 0);
  bluetooth_switch_write_pin(1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {	
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */  
	  button_state = get_button_state();
	  if(button_state != 0)
	  {
		  HAL_Delay(20);
		  button_state = get_button_state();
	      if(button_state == 1 && last_button_state == 0)
		  {
			  buzzer_toggle(50, 50, 2);
			  func_state++;
			  func_state = func_state == 4 ? 0 : func_state;
			  switch(func_state)
			  {	  
				  /* APP */
				  case 0:
					  start_app_receive();
					  bluetooth_switch_write_pin(1);
					  action_group_state1 = 0;
					  buzzer_toggle(100, 100, 1);
					  led_flash(1, 2000, 2000, 0);
					  break;
				  
				  /* PC */
				  case 1:
					  bluetooth_switch_write_pin(0);
					  buzzer_toggle(100, 100, 2);
					  led_flash(1, 1000, 1000, 0);
					  break;	
				  /* PS2 */
				  case 2:
					  start_ps2_receive();
					  buzzer_toggle(100, 100, 3);
					  led_flash(1, 200, 200, 0);
					  break;
				  
				  /* 脱机 */
				  case 3:
					  stop_ps2_receive();
					  action_group_reset();
					  action_group_state1 = 1;
					  buzzer_toggle(100, 100, 4);
					  led_flash(1, 50, 50, 0);
					  break;				  
			  }
		  }
		  
		  if(button_state == 2 && last_button_state == 0 && func_state == 3)
		  {
			  buzzer_toggle(50, 50, 2);
			  action_group_reset();
			  action_group_state2 = 1;
		  }
	  }
	  last_button_state = button_state;
	  
	  if(action_group_state1 == 1 && action_group_state2== 1)
	  {
		  while(!action_group_run(17, 1));
		  action_group_state2 = 0;
	  }
	  
 	  switch(func_state)
	  {
	  case 0:
	  case 1:
		  app_handler();
		  break;
	  
	  case 2:
		  ps2_handler();
		  break;
	  }
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
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
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

#ifdef  USE_FULL_ASSERT
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
