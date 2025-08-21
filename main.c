/* USER CODE BEGIN Header */
	/**
	  ******************************************************************************
	  * @file           : main.c
	  * @brief          : Main program body
	  ******************************************************************************
	  * @attention
	  *
	  * Copyright (c) 2025 STMicroelectronics.
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
#include "pdm2pcm.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
	#include <string.h>
	#include "stm32f4_discovery_audio.h"
	#include <math.h>
	#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
	#define AUDIO_BUFFER_SIZE_PDM 128
	#define AUDIO_BUFFER_SIZE_PCM 32
	#define AUDIO_CHIP_ADDRESS 0x4A
	#define SAMPLE_RATE 16000
	#define PI 3.14159265
	#define FREQ 2000.0  // Frequência da senoide (440Hz = Lá)
	#define AUDIO_TEST_BUFFER_SIZE 16000
	#define UART_TX_BUFFER_SIZE 2048




/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi3_tx;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
	volatile uint8_t isAudioBufferPDMHalf = 0u;
	volatile uint8_t isAudioBufferPDMFull = 0u;
	volatile uint8_t pcm_buffer_half_free =0u;
	volatile uint8_t isI2SReadingError = 0u;
	volatile uint8_t uart_busy = 0 ;
	volatile uint8_t pcm_counter = 0;
	volatile uint8_t pcm_ready = 0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_I2S3_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	uint16_t audioBufferPDM[AUDIO_BUFFER_SIZE_PDM];
	int16_t audioBufferPCM[AUDIO_BUFFER_SIZE_PCM];
	int16_t pcm_tx_buffer[AUDIO_BUFFER_SIZE_PCM];
	int16_t uartBufferPCM[AUDIO_BUFFER_SIZE_PCM];
	char uart_tx_buffer[UART_TX_BUFFER_SIZE];


	//esse size é a quantidade de amostras do buffer : pcmBuffer[size]
	void Format_PCM_For_Plotter(int16_t* pcmBuffer, uint16_t size)
	{
	    uint16_t len = 0;
	    for (uint16_t i = 0; i < size; i++)
	    {
	        len += snprintf(&uart_tx_buffer[len], UART_TX_BUFFER_SIZE - len, "%d\n", pcmBuffer[i]);
	        if (len >= UART_TX_BUFFER_SIZE - 8) break;
	    }
	    //HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12, GPIO_PIN_SET);
	}
	void Transmit_PCM_Plotter_DMA(int16_t* pcmBuffer, uint16_t size)
		{
			Format_PCM_For_Plotter(pcmBuffer, size);
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer));
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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2S2_Init();
  MX_USART2_UART_Init();
  MX_CRC_Init();
  MX_PDM2PCM_Init();
  MX_I2S3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

	  //resetando o codec

	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
	  HAL_Delay(5);
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
	  HAL_Delay(5);
	  BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_HEADPHONE, 90, I2S_AUDIOFREQ_16K);
	  BSP_AUDIO_OUT_SetVolume(90);
	  memset(audioBufferPCM, 0, sizeof(audioBufferPCM)); //evita lixo eletronico zerando os bytes do buffer pcm
	  BSP_AUDIO_OUT_Play((uint16_t*)audioBufferPCM, AUDIO_BUFFER_SIZE_PCM * 2);
	  HAL_I2S_Receive_DMA(&hi2s2,(uint16_t*)audioBufferPDM, AUDIO_BUFFER_SIZE_PDM);
	  //BSP_AUDIO_OUT_Play((uint16_t*)audioTestBuffer, AUDIO_TEST_BUFFER_SIZE * 2);
	  //HAL_Delay(5000);
	  //BSP_AUDIO_OUT_SetVolume(0);

	  pcm_buffer_half_free = 1;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	  while (1)
	   {
			  if (pcm_buffer_half_free == 1)
				{

				  if (isAudioBufferPDMHalf == 1)
				  {

					PDM_Filter((uint8_t*)&audioBufferPDM[0], &audioBufferPCM[0], &PDM1_filter_handler);
					//PDM_Filter((uint8_t*)&audioBufferPDM[0], &uartBufferPCM[0], &PDM1_filter_handler);
					isAudioBufferPDMHalf = 0;
					pcm_buffer_half_free = 0;
					//pcm_counter++;


				  }
				  else if (isAudioBufferPDMFull == 1)
					 {
					   PDM_Filter((uint8_t*)&audioBufferPDM[AUDIO_BUFFER_SIZE_PDM/2], &audioBufferPCM[0], &PDM1_filter_handler);
					   //PDM_Filter((uint8_t*)&audioBufferPDM[0], &uartBufferPCM[0], &PDM1_filter_handler);
					   isAudioBufferPDMFull = 0;
					   pcm_buffer_half_free = 0;
					   //pcm_counter++;

					 }
				   }


			  if (pcm_buffer_half_free == 2)
				{

				  if (isAudioBufferPDMHalf == 1)
				  {

					PDM_Filter((uint8_t*)&audioBufferPDM[0], &audioBufferPCM[16], &PDM1_filter_handler);
					//PDM_Filter((uint8_t*)&audioBufferPDM[0], &uartBufferPCM[0], &PDM1_filter_handler);
					isAudioBufferPDMHalf = 0;
					pcm_buffer_half_free = 0;

				  }
				  else if (isAudioBufferPDMFull == 1)
				  {

					PDM_Filter((uint8_t*)&audioBufferPDM[AUDIO_BUFFER_SIZE_PDM/2], &audioBufferPCM[16], &PDM1_filter_handler);
					//PDM_Filter((uint8_t*)&audioBufferPDM[0], &uartBufferPCM[0], &PDM1_filter_handler);
					isAudioBufferPDMFull = 0;
					pcm_buffer_half_free = 0;
					pcm_counter++;
				  }
				}


		 if (isI2SReadingError)
		 {
		   isI2SReadingError = 0u;
		   HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
		 }
		 if(pcm_counter == 2)
		 {
			 //memcpy(pcm_tx_buffer, audioBufferPCM, sizeof(pcm_tx_buffer));//faz uma cópia do buffer pcm pro buffer tx
			 for (int i = 0; i < AUDIO_BUFFER_SIZE_PCM; i++) {
			     pcm_tx_buffer[i] = audioBufferPCM[i];
			 }
			 pcm_counter = 0;
			 pcm_ready = 1;
		 }
		 if (!uart_busy) {
			 if (pcm_ready)
			 {
				uart_busy = 1;
				pcm_ready = 0;
				//sprintf(uart_tx_buffer, "%d\n", 500);
				Format_PCM_For_Plotter(pcm_tx_buffer, AUDIO_BUFFER_SIZE_PCM);
				//HAL_UART_Transmit_DMA(&huart2, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer));
				HAL_UART_Transmit_DMA(&huart2, (uint8_t*)audioBufferPCM,AUDIO_BUFFER_SIZE_PCM * sizeof(int16_t));
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12,GPIO_PIN_SET);
			 }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_CRC_DR_RESET(&hcrc);
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_MSB;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_16K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_16K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
	void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
	{
		isAudioBufferPDMHalf = 1;

	}

	void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
	{

		isAudioBufferPDMFull = 1;


	}

	void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s)
	{
		isI2SReadingError = 1u;
	}
	void BSP_AUDIO_OUT_HalfTransfer_CallBack(void)
	{
		pcm_buffer_half_free = 1; // Sinaliza que a PRIMEIRA metade está livre
	}
	void BSP_AUDIO_OUT_TransferComplete_CallBack(void)
	{
		pcm_buffer_half_free = 2; // Sinaliza que a SEGUNDA metade está livre
	}
	void BSP_AUDIO_OUT_Error_CallBack(void)
	{

	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
	  Error_Handler();
	}
	void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
	{
		if (huart->Instance == USART2)
		{
			uart_busy = 0;
			//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12,GPIO_PIN_SET);
		}
	}



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
