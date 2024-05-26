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
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile int x = 0;
volatile int y = 0;
volatile unsigned char map[256];
volatile unsigned char cmap[32];

volatile unsigned char customs[7][8];

void create_customs() {
	//znak1
	customs[0][0] = 0b01010;
	customs[0][1] = 0b01010;
	customs[0][2] = 0b01010;
	customs[0][4] = 0b00000;
	customs[0][3] = 0b00000;
	customs[0][5] = 0b01010;
	customs[0][6] = 0b01010;
	customs[0][7] = 0b01010;

	//znak2
	customs[1][0] = 0b11011;
	customs[1][1] = 0b00000;
	customs[1][2] = 0b11011;
	customs[1][4] = 0b11011;
	customs[1][3] = 0b11011;
	customs[1][5] = 0b11011;
	customs[1][6] = 0b00000;
	customs[1][7] = 0b11011;

	//znak3
	customs[2][0] = 0b00000;
	customs[2][1] = 0b00100;
	customs[2][2] = 0b00100;
	customs[2][4] = 0b00101;
	customs[2][3] = 0b10100;
	customs[2][5] = 0b00100;
	customs[2][6] = 0b00100;
	customs[2][7] = 0b00000;


	//znak4
	customs[3][0] = 0b00100;
	customs[3][1] = 0b01110;
	customs[3][2] = 0b00000;
	customs[3][4] = 0b11011;
	customs[3][3] = 0b11011;
	customs[3][5] = 0b00000;
	customs[3][6] = 0b01110;
	customs[3][7] = 0b00100;

	//znak5
	customs[4][0] = 0b10001;
	customs[4][1] = 0b10001;
	customs[4][2] = 0b11011;
	customs[4][4] = 0b00000;
	customs[4][3] = 0b00000;
	customs[4][5] = 0b11011;
	customs[4][6] = 0b10001;
	customs[4][7] = 0b10001;

	//znak6
	customs[5][0] = 0b11011;
	customs[5][1] = 0b00000;
	customs[5][2] = 0b10101;
	customs[5][4] = 0b00100;
	customs[5][3] = 0b00100;
	customs[5][5] = 0b10101;
	customs[5][6] = 0b00000;
	customs[5][7] = 0b11011;

	//znak7
	customs[6][0] = 0b00000;
	customs[6][1] = 0b01010;
	customs[6][2] = 0b11011;
	customs[6][4] = 0b01000;
	customs[6][3] = 0b00010;
	customs[6][5] = 0b11011;
	customs[6][6] = 0b01010;
	customs[6][7] = 0b00000;

	for(int k = 0; k<7; k++) {
		lcd_cmd(0x40 + (k + 1) * 8);
		for (int i = 0; i < 8; i++) {
			lcd_char_cp(customs[k][i]);
		}
	}

	//template
	/*
	 * customs[0][0] = 0b00000;
	customs[0][1] = 0b00000;
	customs[0][2] = 0b00000;
	customs[0][4] = 0b00000;
	customs[0][3] = 0b00000;
	customs[0][5] = 0b00000;
	customs[0][6] = 0b00000;
	customs[0][7] = 0b00000;
	 */

}

void create_map() {

	for(int i = 0; i < 32; i++) {
		cmap[i] = ' ';
	}

	for(int i = 1; i<16; i++) {
		int prz = rand() % 7;
		cmap[i] = 1 + prz;
		for (int j = 0; j < 8; j++) {
			map[i * 8 + j] = customs[prz][j];
		}
	}
	for(int i = 0; i<15; i++) {
		int prz = rand() % 7;
		cmap[16 + i] = 1 + prz;
		for (int j = 0; j < 8; j++) {
			map[128 + i * 8 + j] = customs[prz][j];
		}
	}

	//int row
//	int k = 16;
//	cmap[k/8] = 1;
//	for (int i = 0; i < 8; i++) {
//		map[k + i] = customs[0][i];
//	}
//
//	k = 80;
//	cmap[k/8] = 2;
//	for (int i = 0; i < 8; i++) {
//		map[k + i] = customs[1][i];
//	}

	for (int i = 0; i < 16; i++) {
		lcd_char(1, i + 1, cmap[i]);
	}
	for (int i = 0; i < 16; i++) {
		lcd_char(2, i + 1, cmap[16 + i]);
	}
}

void create_char(int addr, int x, int y, char p) {

	int mx = x % 5;
	int my = y % 8;

	lcd_cmd(addr);
	for(int i = 0; i < 8; i++) {
		int col = x / 5;
		int row = y / 8;

		int player = (i == my) ? (1 << (4 - mx)) : 0;
		int m = map[(col * 8) + i % 8 + (row * 128)];

		lcd_char_cp(m | (p ? player : 0));
	}
}

void move(int d) {

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  lcd_init(_LCD_4BIT, _LCD_FONT_5x8, _LCD_2LINE);

  lcd_clear();

  create_customs();
  //create(0x40, 3, 5);

  //lcd_char(1, 1, 0);
  //lcd_char(1, 1, 2);
  //lcd_char(1, 2, 0);

  //create(0x40, 2, 3);

  int x = 1;
  int y = 1;

  int move_t = 0;
  int move_cd = 5;

  int start = 0;
  int frame = 0;

  int state = 0; // PRE = 0, INGAME = 1, SCORE = 2

  int score = 0;

  lcd_clear();
  lcd_print(1, 1, "  Press SELECT");
  lcd_print(2, 1, "    to play");

  //lcd_print(1, 1, "Hello World");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //continue;

	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 2500);
	  int k = (300 + HAL_ADC_GetValue(&hadc1)) / 700;

	  if(state == 0) {
		  if(k != 4) {
			  continue;
		  }
		  start = frame;
		  score = 0;
		  create_map();

		  x = 1;
		  y = 1;

		  state = 1;
		  continue;
	  }

	  if(state == 2) {
		  if(k != 4) {
			  continue;
		  }
		  start = frame;
		  score = 0;
		  create_map();

		  x = 1;
		  y = 1;

		  state = 1;
		  continue;
	  }

	  if((frame - start)/2 >= 1000) {
		  score = 1000 * (x - 79 + y - 15) / 94;

		  lcd_clear();
		  lcd_line1();
		  lcd_print(1, 1, "     SCORE:");
		  char buffer[16] = {' '};
		  sprintf(buffer, "       %d", score);
		  lcd_line2();
		  lcd_print(2, 1, buffer);

		  state = 2;
		  continue;
	  }

	  if(frame > move_t) {
		  int px = x;
		  int py = y;
		  switch(k) {
		  case 0:
			  x += 1;
		  break;

		  case 1:
			  y -= 1;
		  break;

		  case 2:
			  y += 1;
		  break;

		  case 3:
			  x -= 1;
		  break;
		  }

		  if(x == 80 - 1 && y == 16 - 1) {
			  score = 1000 - (frame - start)/2;

			  lcd_clear();
			  lcd_print(1, 1, "     SCORE:");
			  char buffer[16] = {' '};
			  sprintf(buffer, "      %d", score);
			  lcd_print(2, 1, buffer);

			  state = 2;
			  continue;
		  }

		  int col = x / 5;
		  int row = y / 8;

		  char cant = (map[(col * 8) + y % 8 + (row * 128)] >> (4 - x % 5)) & 1;

		  if(cant || x < 0 || x >= 80 || y < 0 || y >= 16) {
			  x = px;
			  y = py;
		  } else {
			  if (x / 5 != px / 5) {
				  int row = y / 8;
				  int col = px / 5;

				  lcd_char(1 + row, 1 + col, cmap[row * 16 + col]);
			  }
			  if (y / 8 != py / 8) {
				  int row = py / 8;
				  int col = x / 5;

				  lcd_char(1 + row, 1 + col, cmap[row * 16 + col]);
			  }

			  move_t = frame + move_cd;
		  }

	  }



	  create_char(0x40, x, y, (frame / 10) % 2);
	  lcd_char(1 + y / 8, 1 + x / 5, 0);

	  //lcd_print(1, 1, "Hello World");

	  frame += 1;
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.Timing = 0x00000E14;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_D6_Pin|LCD_D5_Pin|LCD_D4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_D7_Pin|LCD_RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_D6_Pin LCD_D5_Pin LCD_D4_Pin */
  GPIO_InitStruct.Pin = LCD_D6_Pin|LCD_D5_Pin|LCD_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_EN_Pin */
  GPIO_InitStruct.Pin = LCD_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_D7_Pin LCD_RS_Pin */
  GPIO_InitStruct.Pin = LCD_D7_Pin|LCD_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
