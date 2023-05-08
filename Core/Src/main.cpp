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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lidar_driver.h"
#include "lidar.h"
#include "dbscan.h"
#include <stdio.h>
#include <string.h>
#include <vector>
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
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

// Variáveis de recepção de dados do lidar
volatile bool received_flag, t_flag = false;
volatile bool parseData = false;
uint8_t buff[3*BLOCK_SIZE] = {0};

// informações do mapa scaneado pelo lidar
lidar_map map_li;

// aux
uint8_t i = 0;
int count = 0;

//Quantidade de pacotes recebidos -> utilizado no dbscan
int n_blocks = 0;

//Variáveis dbscan
float eps = 0.05; // vizinhaça
int minPts = 4; // minimo de pontos para formar uma região densa

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM12_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

extern "C"{
int _write(int fd, char* ptr, int len) {
	//HAL_UART_Transmit_IT(&huart2, (uint8_t *) ptr, strlen(ptr));
//	HAL_UART_Transmit_DMA(&huart2, (uint8_t*) ptr, strlen(ptr));
	HAL_UART_Transmit(&huart2, (uint8_t *) ptr, len, strlen(ptr));
	return len;
}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
		static uint8_t k = 0;
		static uint8_t previous_k = 2;
		static uint8_t next_k = 1;

		printf("enviou\r\n");

		if(parseData)
		{
			//Set up DMA to receive next packet.
			HAL_UART_Receive_DMA(&huart1, buff + next_k * BLOCK_SIZE, BLOCK_SIZE);
			n_blocks++;

			//Calculate start angles of current and last packet, compare them -> calculate step, convert to Q16
			int current_start_angle_q8 = (((uint16_t)(*(buff + k * BLOCK_SIZE + ANGLE_OFFSET_14_8)) & 0x7f) << 10);
			current_start_angle_q8 |= (*(buff + k * BLOCK_SIZE + ANGLE_OFFSET_07_0)) << 2;
			int previous_start_angle_q8 = (((uint16_t)(*(buff + previous_k * BLOCK_SIZE + ANGLE_OFFSET_14_8)) & 0x7f) << 10);
			previous_start_angle_q8 |= (*(buff + previous_k * BLOCK_SIZE + ANGLE_OFFSET_07_0)) << 2;

			int start_angle_diff_q8 = current_start_angle_q8 - previous_start_angle_q8;
			if(previous_start_angle_q8 > current_start_angle_q8) start_angle_diff_q8 += (360 << 8);
			int angle_step_q16 = (start_angle_diff_q8 << 3) / 3;
			int current_raw_angle_q16 = (previous_start_angle_q8 << 8);


			if(previous_start_angle_q8 > 0 &&  map_li.recv_status != RECV_STATUS_SYNCED_TWICE_MAP_READY)
			{
				for(uint8_t i = 0; i < ULTRA_CABINS_IN_RESPONSE ; i++)
				{
					//Each ultra_cabin consists of data for three points.
					int dist_q2[3];
					int angle_q6[3];
					int syncBit[3];

					//Get one ultra-cabin FROM PREVIOUS BUFFER and save it as uint32_t
					uint32_t combined = (*((uint32_t*)(buff + previous_k * BLOCK_SIZE + ULTRA_CABIN_0_OFFSET + CABIN_SIZE * i)));

					//"Magic shift", 'DO NOT TOUCH', from SDK.
					int dist_major = (combined & 0xfff);
					int dist_predict1 = (((int)(combined << 10)) >> 22);
					int dist_predict2 = (((int)combined) >> 22);

					int dist_major2;
					uint32_t scale1, scale2;

					//Fetch data from next ultra_cabin, from the right packet.
					if(i == ULTRA_CABINS_IN_RESPONSE - 1){
						//last ultra cabin in packet, get data from first ultra_cabin in "current" buffer
						dist_major2 = (*((uint32_t*)(buff + k * BLOCK_SIZE + ULTRA_CABIN_0_OFFSET + CABIN_SIZE * 0))) & 0xfff;
					}else{
						dist_major2 = (*((uint32_t*)(buff + previous_k * BLOCK_SIZE + ULTRA_CABIN_0_OFFSET + CABIN_SIZE * (i + 1)))) & 0xfff;
					}


					//### SDK MATH ###
					dist_major = _varbitscale_decode(dist_major, scale1);
					dist_major2 = _varbitscale_decode(dist_major2, scale2);

					int dist_base1 = dist_major;
					int dist_base2 = dist_major2;

					if((!dist_major) && dist_major2){
						dist_base1 = dist_major2;
						scale1 = scale2;
					}


					//Fetch data for THREE points
					dist_q2[0] = (dist_major << 2);

					//For -Wsign-compare warnings: The following line was copied from SDK and probably should not be changed
					if((dist_predict1 == 0xFFFFFE00) || (dist_predict1 == 0x1FF)){
						dist_q2[1] = 0;
					} else {
						dist_predict1 = (dist_predict1 << scale1);
						dist_q2[1] = (dist_predict1 + dist_base1) << 2;
					}

					//For -Wsign-compare warnings: The following line was copied from SDK and probably should not be changed
					if((dist_predict1 == 0xFFFFFE00) || (dist_predict2 == 0x1FF)){
						dist_q2[2] = 0;
					} else {
						dist_predict2 =  (dist_predict2 << scale2);
						dist_q2[2] = (dist_predict2 + dist_base2) << 2;
					}

					//## ENDOF SDK MATH ###
					for(int j = 0; j < 3; j++)
					{

						syncBit[j] = (((current_raw_angle_q16 + angle_step_q16) % (360 << 16)) < angle_step_q16) ? 1 : 0;

						//Check if a full 360 map_li is already saved (two syncs <=> twice passed 0 angle)
						if(syncBit[j]){
							if(map_li.recv_status == RECV_STATUS_NOT_SYNCED){
								map_li.recv_status = RECV_STATUS_SYNCED_ONCE;

							}else if(map_li.recv_status == RECV_STATUS_SYNCED_ONCE){
								map_li.recv_status = RECV_STATUS_SYNCED_TWICE_MAP_READY;
								break;
							}
						}

						//### SDK MATH ###
						int offset_angle_mean_q16 = (int)(7.5 * 3.1415926535 * (1<<16) / 180.0);

						if(dist_q2[j] >= (50*4)){
							const int k1 = 98361;
							const int k2 = (int)(k1/dist_q2[j]);
							offset_angle_mean_q16 = (int)((8 * 3.1415926535 * (1<<16) / 180) - (k2 << 6) - (k2 * k2 * k2) / 98304);
						}

						angle_q6[j] = ((current_raw_angle_q16 - int(offset_angle_mean_q16 * 180 / 3.14159265)) >> 10);
						current_raw_angle_q16 += angle_step_q16;

						if(angle_q6[j] < 0) angle_q6[j] += (360 << 6);
						else if(angle_q6[j] >= (360 << 6)) angle_q6[j] -= (360 << 6);

						//### ENDOF SDK MATH ###

						float f_angle = ((float) angle_q6[j])/ 64.0f;
						float f_distance = ((float) dist_q2[j]) / 4.0f;

						uint16_t quality = dist_q2[j] ? (0x2f << RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) : 0;

						if(f_angle >1.0 && f_angle < 360.0 && quality != 0  && map_li.cnt < POINTS  && map_li.recv_status == RECV_STATUS_SYNCED_ONCE)
						{

						  f_angle = 180.0 - f_angle;
						  if(f_angle < 0) f_angle += 360.0;

						  //Add received data to the arrays
						  map_li.angles[map_li.cnt] = f_angle;
						  map_li.distances[map_li.cnt] = f_distance;

						  //If this is the greatest distance, save it
						  if(f_distance > map_li.dmax){
							  map_li.dmax = f_distance;
							  map_li.amax = f_angle;
						  }
						  else if(f_distance < map_li.dmin && f_distance > 0.0){
							  map_li.dmin = f_distance;
							  map_li.amin = f_angle;
						  }
						  map_li.cnt++;
						}
					}
					if(map_li.recv_status == RECV_STATUS_SYNCED_TWICE_MAP_READY) break;
				}
			}
//			DBSCAN();

//			while(i < next_k * BLOCK_SIZE){
//			while(i < 132){
//				if(count == 0){
//					printf("começou novo pacote!!!!\r\n");
//					count = 1;
//				}
//				printf("%d -- ", buff[i+ next_k * BLOCK_SIZE]);
//				i++;
//			}

			//printf("distance: %d\r\n", distance);
			//printf("angle: %d\r\n", angle);
			k++;
			previous_k++;
			next_k++;
			if(k >= 3) k = 0;
			if(previous_k >= 3) previous_k = 0;
			if(next_k >= 3) next_k = 0;
		}
		//In main, this flag is used only once.
		//Can be used to indicate that DMA has finished receiving data.
		received_flag = true;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	t_flag = true;
}

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
  MX_DMA_Init();
  MX_TIM12_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("Some String\r\n");
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1 );

  HAL_UART_Transmit_DMA(&huart1, (uint8_t*) rplidar_reset, 2);
  HAL_Delay(3000);

  //Send the ultra capsuled mode scan request over DMA
  HAL_UART_Transmit_DMA(&huart1, (uint8_t*) rplidar_scan_ultra, 9);


//  //Clear UART receiving flags
//  __HAL_UART_CLEAR_NEFLAG(&huart2);
//  __HAL_UART_CLEAR_OREFLAG(&huart2);

  //Set DMA to receive 8-byte response descriptor, do parse it(parseData==false)
  HAL_UART_Receive_DMA(&huart1, buff, 7);

  //Wait until the descriptor is received.
  while(!received_flag);
//  printf("depois do while received flag\r\n");

  //With parseData flag on, the DMA "received" interrupt from now will now parse the data packets
  //received. The interrupt will also set DMA to pend for another 132b block, continuously.
  parseData = true;

  //Let the DMA receive blocks continuously
  HAL_UART_Receive_DMA(&huart1, buff, BLOCK_SIZE);
  printf("after dma receive\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  printf("detro do while\r\n");
	  while(map_li.recv_status != RECV_STATUS_SYNCED_TWICE_MAP_READY);
	  //Clear received data.
	  map_li = lidar_map();
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 3;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 1000;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 256000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
