/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fonts.h"
#include "ssd1306.h"
#include "si5351.h"
#include "stdlib.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define INTERMEDIATE_FREQUENCY	10700000
#define START_FREQUENCY			7000000

#define STEP_BUF_SIZE	5
uint32_t stepBuf[STEP_BUF_SIZE] = { 100, 500, 1000, 10000, 100000 };	// Hz
uint8_t stepBufIdx = 0;

#define ENCODER_TIM    htim2

#define XTAL_FREQ	25000000

#define SI_CLK0_CONTROL		16
#define SI_PLL_RESET		177
#define SI_CLK_SRC_PLL_A	0b00000000
#define SI_CLK_SRC_PLL_B	0b00100000

typedef enum _ENCODER_DIRECTION_TYPE {
	DIRECTION_NONE = 0,
	DIRECTION_UP = 1,
	DIRECTION_DOWN
} ENCODER_DIRECTION_TYPE;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
char buff[20];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
uint32_t encoderValue = 0;
ENCODER_DIRECTION_TYPE getEncoderDir(void)
{
	ENCODER_DIRECTION_TYPE tmpDir = DIRECTION_NONE;
	static uint8_t cntTick = 0;

	if ( encoderValue != ENCODER_TIM.Instance->CNT && ( cntTick < 4 )) {
		tmpDir = (encoderValue < ENCODER_TIM.Instance->CNT) ? DIRECTION_UP : DIRECTION_DOWN;
		cntTick++;
		encoderValue = ENCODER_TIM.Instance->CNT;

		if ( cntTick == 4 ) {
			cntTick = 0;
			return tmpDir;
		}
	}
	return DIRECTION_NONE;
}

void printRfFrequency(uint32_t frequency) {

	itoa(frequency/100, buff, 10);
	ssd1306_SetCursor(1, 0);

	if ( frequency >= 10000000 ) {
		buff[6] = buff[5];
		buff[5] = '.';
	}
	else if (frequency < 1000000) {
		buff[4] = buff[3];
		buff[3] = '.';
		buff[5] = ' ';
		buff[6] = ' ';
	}
	else {
		buff[5] = buff[4];
		buff[4] = '.';
		buff[6] = ' ';
	}

	buff[7] = 0;

	/*buff[7] = ' ';
	buff[8] = 'k';
	buff[9] = 'H';
	buff[10] = 'z';
	buff[11] = ' ';
	buff[12] = 0;*/

	/*itoa(frequency, buff, 10);
	ssd1306_SetCursor(1, 0);*/

	ssd1306_WriteString(buff, Font_16x26, White);
	ssd1306_UpdateScreen();
}

void showTuningStep(void) {
	ssd1306_SetCursor(75, 45);
	/*itoa(stepBuf[stepBufIdx] >= 1000 ? stepBuf[stepBufIdx]/1000 : stepBuf[stepBufIdx], buff, 10);
	ssd1306_WriteString("Step:        ", Font_11x18, White);
	ssd1306_SetCursor(5+(6*10), 0);
	if (stepBuf[stepBufIdx] == 1000 ) { // '1k'
		buff[1] = 'k';
		buff[2] = 0;
	}
	if (stepBuf[stepBufIdx] == 10000 ) { // '10k'
		buff[2] = 'k';
		buff[3] = 0;
	}
	if (stepBuf[stepBufIdx] == 100000 ) { // '100k'
		buff[3] = 'k';
		buff[4] = 0;
	}

	ssd1306_WriteString(buff, Font_11x18, White);*/

	if ( stepBuf[stepBufIdx] == 100 ) {
		ssd1306_WriteString("100 ", Font_11x18, White);
	}
	else if ( stepBuf[stepBufIdx] == 500 ) {
		ssd1306_WriteString("500 ", Font_11x18, White);
	}
	else if ( stepBuf[stepBufIdx] == 1000 ) {
		ssd1306_WriteString("1k  ", Font_11x18, White);
	}
	else if ( stepBuf[stepBufIdx] == 10000 ) {
		ssd1306_WriteString("10k ", Font_11x18, White);
	}
	else if ( stepBuf[stepBufIdx] == 100000 ) {
		ssd1306_WriteString("100k", Font_11x18, White);
	}
	ssd1306_UpdateScreen();
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define NUMBER_TICK_KEY_WAIT	20000

unsigned int countDelayKey = 0;
uint8_t keyPressed = 0;

bool isEncoderKeyPressed() {

		if ( !keyPressed ) {
			if ( (Enkoder_Key_GPIO_Port->IDR & Enkoder_Key_Pin) == 0 ) {
				keyPressed = 1;
			}
		}
		else if ( countDelayKey < NUMBER_TICK_KEY_WAIT ) {
			countDelayKey++;
			if( countDelayKey == NUMBER_TICK_KEY_WAIT ) {
				if ( (Enkoder_Key_GPIO_Port->IDR & Enkoder_Key_Pin) == 0 ) {
					//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
					keyPressed = 0;
					countDelayKey = 0;
					return true;
				}
			}

			if( countDelayKey >= NUMBER_TICK_KEY_WAIT ) {
				keyPressed = 0;
				countDelayKey = 0;
			}
		}
		return false;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	  ENCODER_DIRECTION_TYPE encoderDir;
	  uint32_t pll_frequency = START_FREQUENCY;	//17700000;
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
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init();
    /*ssd1306_SetCursor(5, 0);
    ssd1306_WriteString("Step: 100", Font_11x18, White);
    ssd1306_UpdateScreen();*/
  showTuningStep();

    //uint32_t xt_freq = SI5351_XTAL_FREQ;
    //Si5351RegSet pll_reg;
    Si5351_init(SI5351_CRYSTAL_LOAD_8PF, SI5351_XTAL_FREQ);
    Si5351_set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
    Si5351_set_freq(((pll_frequency + INTERMEDIATE_FREQUENCY)* SI5351_FREQ_MULT), SI5351_PLL_FIXED, SI5351_CLK0);
    Si5351_drive_strength(SI5351_CLK0, SI5351_DRIVE_2MA);

  //  Si5351_set_pll(SI5351_PLL_FIXED, SI5351_PLLB);
  //  Si5351_set_freq(((pll_frequency)* SI5351_FREQ_MULT), SI5351_PLL_FIXED, SI5351_CLK1);
  //  Si5351_drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA);

    Si5351_set_state_out(SI5351_CLK1, SI5351_OUT_DISABLE);
    Si5351_set_state_out(SI5351_CLK2, SI5351_OUT_DISABLE);

    printRfFrequency(pll_frequency);

    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  encoderDir = getEncoderDir();
	  		if ( encoderDir == DIRECTION_UP ) {
	  			pll_frequency = pll_frequency + stepBuf[stepBufIdx];
	  			//si5351aSetFrequency(pll_frequency);
	  			Si5351_set_freq(((pll_frequency + INTERMEDIATE_FREQUENCY)* SI5351_FREQ_MULT), SI5351_PLL_FIXED, SI5351_CLK0);
	  			printRfFrequency(pll_frequency);
	  		}
	  		else if ( encoderDir == DIRECTION_DOWN ) {
	  			if (pll_frequency > 0 && (pll_frequency - stepBuf[stepBufIdx]) >= 100000) {
					pll_frequency = pll_frequency - stepBuf[stepBufIdx];
					//si5351aSetFrequency(pll_frequency);
					Si5351_set_freq(((pll_frequency + INTERMEDIATE_FREQUENCY)* SI5351_FREQ_MULT), SI5351_PLL_FIXED, SI5351_CLK0);
					printRfFrequency(pll_frequency);
	  			}
	  		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  		if ( isEncoderKeyPressed() ) {
	  			//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	  			if ( stepBufIdx < STEP_BUF_SIZE ) {
	  				stepBufIdx++;
	  				if ( stepBufIdx == STEP_BUF_SIZE )
	  				{
	  					stepBufIdx = 0;
	  				}
	  			}
	  			showTuningStep();
	  		}
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : Enkoder_Key_Pin */
  GPIO_InitStruct.Pin = Enkoder_Key_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Enkoder_Key_GPIO_Port, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
