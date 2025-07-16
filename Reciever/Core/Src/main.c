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
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "nrf24l01p.h"
#include "uart.h"
#include "u8g2.h"

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

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t nrf24l01p_rx_flag = 0;
extern ring_buffer_t uart_rx;

bool sen_L = 0;
bool sen_S = 0;
bool sen_B = 0;
bool sen_R = 0;

char control='n';
char speed  ='0';
char buttons='0';
char EN		='0';
bool   in1,in2,in3,in4;
int	  ADC_distance;
//uint8_t address[5]={0xE7,0xE7,0xE7,0xE7,0xE7};
char  data[10];
char  test[10]="STOP";
bool  autoMode = 0;
uint8_t P0_address[6] = "node0";
uint8_t P1_address[6] = "node1";

uint8_t text_trans[2];
uint8_t rx_data[NRF24L01P_PAYLOAD_LENGTH] = { 0 };
uint8_t tx_data[NRF24L01P_PAYLOAD_LENGTH] = { 0 };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern uint8_t u8x8_stm32_gpio_and_delay(u8x8_t *u8x8, uint8_t msg,
		uint8_t arg_int, void *arg_ptr);
extern uint8_t u8x8_byte_stm32_hw_spi(u8x8_t *u8x8, uint8_t msg,
		uint8_t arg_int, void *arg_ptr);

static u8g2_t u8g2;

// Hàm map() ánh xạ giá trị từ một phạm vi này sang một phạm vi khác
int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void int_to_string(int number, char* buffer) {
    sprintf(buffer, "%d", number);   // Ham chuyen int sang string
}
void  sensor()
{
	sen_R = HAL_GPIO_ReadPin(sensor_Right_GPIO_Port, sensor_Right_Pin);
	sen_L = HAL_GPIO_ReadPin(sensor_Left_GPIO_Port, sensor_Left_Pin);
	sen_S = HAL_GPIO_ReadPin(sensor_Straight_GPIO_Port, sensor_Straight_Pin);
	sen_B = HAL_GPIO_ReadPin(sensor_Back_GPIO_Port, sensor_Back_Pin);
	if( sen_R ==0 || sen_L ==0 || sen_B ==0 || sen_S ==0  )
		{
			if( sen_R == 0)
				{text_trans[0] = 'R';
				TIM3->CCR1 = 0;	//  left
				TIM3->CCR2 = 0;	//	right
				TIM3->CCR3 = 0;	//	Left Mecanum
				TIM3->CCR4 = 0;	//	Right Mecanum
				}

			if( sen_L  == 0 )
				{text_trans[0] = 'L';
				TIM3->CCR1 = 0;	//  left
				TIM3->CCR2 = 0;	//	right
				TIM3->CCR3 = 0;	//	Left Mecanum
				TIM3->CCR4 = 0;	//	Right Mecanum
				}

			if( sen_B == 0)
				{text_trans[0] = 'B';
				TIM3->CCR1 = 0;	//  left
				TIM3->CCR2 = 0;	//	right
				TIM3->CCR3 = 0;	//	Left Mecanum
				TIM3->CCR4 = 0;	//	Right Mecanum
				}
			if( sen_S == 0)
				{text_trans[0] = 'S';
				TIM3->CCR1 = 0;	//  left
				TIM3->CCR2 = 0;	//	right
				TIM3->CCR3 = 0;	//	Left Mecanum
				TIM3->CCR4 = 0;	//	Right Mecanum
				}

			while(sen_R ==0 || sen_L ==0 || sen_B ==0 || sen_S ==0  ){
				sen_R = HAL_GPIO_ReadPin(sensor_Right_GPIO_Port, sensor_Right_Pin);
				sen_L = HAL_GPIO_ReadPin(sensor_Left_GPIO_Port, sensor_Left_Pin);
				sen_S = HAL_GPIO_ReadPin(sensor_Straight_GPIO_Port, sensor_Straight_Pin);
				sen_B = HAL_GPIO_ReadPin(sensor_Back_GPIO_Port, sensor_Back_Pin);
				trans_nRF();
			}
		}
	else
		text_trans[0] = 'N';

}
float distance(){
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion (&hadc1, 100);
	ADC_distance = HAL_ADC_GetValue(&hadc1);
	float dis= 29.988 * pow(map(ADC_distance, 0, 4095, 0, 5000)/1000.0, -1.173);
}
void read_nRF(){
		if (nrf24l01p_rx_flag == 1) {
			nrf24l01p_rx_flag =0;
			uint8_t stat = read_register(NRF24L01P_REG_CONFIG);
			if (!(stat & (1 << 0))) {

				nrf24l01p_prx_mode();

			}

			//nrf24l01p_flush_rx_fifo();
			nrf24l01p_rx_receive(rx_data);
			for(int i = 0; i<4; i++)
			 data[i] = rx_data[i];

			HAL_UART_Transmit(&huart1, rx_data, sizeof(rx_data), 500);
			HAL_GPIO_TogglePin(check_led_GPIO_Port, check_led_Pin);

		}

}

void   trans_nRF(){
		memset(tx_data, 0, sizeof(tx_data));
		int num = 0;
//		if (num = uart_available(&uart_rx)) {
		if (EN = '1') {
			nrf24l01p_ptx_mode();
//			for (int i = 0; i < num; i++) {
//
//				int ch = pop(&uart_rx);
//				if (ch != -1) {
//					tx_data[i] = ch;
//				}
//
//			}

			nrf24l01p_flush_tx_fifo();
			nrf24l01p_tx_transmit(text_trans);
			HAL_Delay(50);
			//HAL_UART_Transmit(&huart1, tx_data, sizeof(tx_data), 500);
			nrf24l01p_prx_mode();
			nrf24l01p_flush_tx_fifo();
		}
}

void controler(char temp)
{
	switch ( temp ){
	case 'W':
		in1 = 1;
		in2 = 0;
		in3 = 1;
		in4 = 0;
			break;
	case 'S':
		in1 = 0;
		in2 = 1;
		in3 = 0;
		in4 = 1;

			break;
	case 'A': // ngang trái
			break;
	case 'D': // ngang phải
			break;
	case 'R': // quẹo phải
		in1 = 0;
		in2 = 1;
		in3 = 1;
		in4 = 0;
		TIM3->CCR1 = 1700*(temp);	//  left
		TIM3->CCR2 = 1700*(temp);	//	right
		TIM3->CCR3 = 2800*(temp);	//	Left Mecanum
		TIM3->CCR4 = 2800*(temp);	//	Right Mecanum
			break;
	case 'L': // quẹo phải
		in1 = 1;
		in2 = 0;
		in3 = 0;
		in4 = 1;
		TIM3->CCR1 = 1700*(temp);	//  left
		TIM3->CCR2 = 1700*(temp);	//	right
		TIM3->CCR3 = 2800*(temp);	//	Left Mecanum
		TIM3->CCR4 = 2800*(temp);	//	Right Mecanum
			break;
	default :
		in1 = 0;
		in2 = 0;
		in3 = 0;
		in4 = 0;

			break;
	}
	HAL_GPIO_WritePin(L298_M_IN1_GPIO_Port,L298_M_IN1_Pin, in1);
	HAL_GPIO_WritePin(L298_M_IN2_GPIO_Port,L298_M_IN2_Pin, in2);
	HAL_GPIO_WritePin(L298_M_IN3_GPIO_Port,L298_M_IN3_Pin, in3);
	HAL_GPIO_WritePin(L298_M_IN4_GPIO_Port,L298_M_IN4_Pin, in4);
}
void readbuttons(char temp ){
	switch ( temp ){
		case '0' :
				break;
		case '1' :
			// Right , đã define trong controller
				break;

		case '2' :
			// Left , đã define trong controller
				break;
		case '3' :

				break;
		case '4' :

				break;
		case '5' :

				break;
		case '6' :

				break;
		case '7' :

				break;
		case '8' :
					autoMode = !autoMode;
						HAL_Delay(500);
				break;
		default:
				break;
		}
}
void readData()
{

	buttons = data[2];
	EN 		= data[3];
	readbuttons(buttons);

}
void remoteControl(){
	control = data[0];
	controler(control);
	speed 	= data[1];
	short temp = speed - 48;
	TIM3->CCR1 = 1700*(temp);	//  left
	TIM3->CCR2 = 1700*(temp);	//	right
	TIM3->CCR3 = 2800*(temp);	//	Left Mecanum
	TIM3->CCR4 = 2800*(temp);	//	Right Mecanum
}
void autoControl()
{
	if ( sen_S == 1)
	{
		in1 = 1;
		in2 = 0;
		in3 = 1;
		in4 = 0;
		TIM3->CCR1 = 5000;	//  left
		TIM3->CCR2 = 5000;	//	right
		TIM3->CCR3 = 10000;	//	Left Mecanum
		TIM3->CCR4 = 10000;	//	Right Mecanum
	}
	else
	{
		in1 = 0;
		in2 = 0;
		in3 = 0;
		in4 = 0;
		TIM3->CCR1 = 0;	//  left
		TIM3->CCR2 = 0;	//	right
		TIM3->CCR3 = 0;	//	Left Mecanum
		TIM3->CCR4 = 0;	//	Right Mecanum

	}
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

// EN LN298
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);	//  left
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);	//	right
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); //  Left Mecanum
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); //  Right Mecanum
  TIM3->CCR1 = 0;	//  left
  TIM3->CCR2 = 0;	//	right
  TIM3->CCR3 = 0;	//	Left Mecanum
  TIM3->CCR4 = 0;	//	Right Mecanum

  //
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  TIM4->CCR1 = 0;
  TIM4->CCR2 = 0;
  TIM4->CCR3 = 0;
  TIM4->CCR4 = 0;

	/*
	 u8g2_Setup_ssd1306_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_stm32_hw_spi,
	 u8x8_stm32_gpio_and_delay);

	 u8g2_InitDisplay(&u8g2);
	 u8g2_SetPowerSave(&u8g2, 0);

	 //u8g2_SetFont(&u8g2, u8g2_font_5x7_tr);
	 //u8g2_SetFont(&u8g2, u8g2_font_6x13_mr);
	 u8g2_SetFont(&u8g2, u8g2_font_inb16_mr);

	 //u8g2_SetDisplayRotation(&u8g2, U8G2_R2);
	 */

	nrf24l01p_rx_init(2500, _1Mbps, P0_address, P1_address);

	rxBufferInit(&uart_rx);

	/* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_ERR);

	/* Enable the UART Data Register not empty Interrupt */
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		/*
		 u8g2_FirstPage(&u8g2);
		 do {
		 u8g2_DrawStr(&u8g2, 5, 21, rx_data);
		 } while (u8g2_NextPage(&u8g2));
		 */
		// doc gia tri cam bien vat can
		sensor();
		// nRF24L01
		readData();
		read_nRF();
		trans_nRF();
		if(autoMode)
			autoControl();
		else
			remoteControl();
		HAL_Delay(30);
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 60000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  huart1.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(check_led_GPIO_Port, check_led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CE_Pin|L298_M_IN3_Pin|L298_M_IN4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OLED_DC_Pin|L298_M_IN2_Pin|L298_M_IN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OLED_RST_Pin|OLED_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : check_led_Pin */
  GPIO_InitStruct.Pin = check_led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(check_led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IRQ_Pin */
  GPIO_InitStruct.Pin = IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CSN_Pin */
  GPIO_InitStruct.Pin = CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CSN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CE_Pin */
  GPIO_InitStruct.Pin = CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_DC_Pin L298_M_IN2_Pin */
  GPIO_InitStruct.Pin = OLED_DC_Pin|L298_M_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_RST_Pin OLED_CS_Pin */
  GPIO_InitStruct.Pin = OLED_RST_Pin|OLED_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : L298_M_IN1_Pin */
  GPIO_InitStruct.Pin = L298_M_IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(L298_M_IN1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : L298_M_IN3_Pin L298_M_IN4_Pin */
  GPIO_InitStruct.Pin = L298_M_IN3_Pin|L298_M_IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : sensor_Left_Pin sensor_Straight_Pin */
  GPIO_InitStruct.Pin = sensor_Left_Pin|sensor_Straight_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : sensor_Right_Pin sensor_Back_Pin */
  GPIO_InitStruct.Pin = sensor_Right_Pin|sensor_Back_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == NRF24L01P_IRQ_PIN_NUMBER) {
		nrf24l01p_rx_flag=1;
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
	while (1) {
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
