/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 * author          : Nhan cho dien, dien vi yeu em
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
#include "nrf24l01p.h"
#include "uart.h"
#include "stdio.h"
#include  "i2c-lcd.h"
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
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t nrf24l01p_rx_flag = 0;
uint8_t adc_flag = 0;
double adcVal = 0.0;

extern ring_buffer_t uart_rx;

// Value Joystick
int	  ADC_X,ADC_Y;
short temp1,temp2;
char speed='z';
char control='z';
char EN    = '0';
short counter = 0;
short buttonStates[10] = {1,0,0,0,0,0,0,0,0,0};
short pressedButton = 0;
//uint8_t address[5]={0xE7,0xE7,0xE7,0xE7,0xE7};

uint8_t P0_address[6] = "node0";
uint8_t P1_address[6] = "node1";

uint8_t rx_data[NRF24L01P_PAYLOAD_LENGTH] = { 0 };
uint8_t tx_data[NRF24L01P_PAYLOAD_LENGTH] = { 0 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void int_to_tx_data(float value, uint8_t* tx_data, size_t max_length) {
	    char buffer[50];
	    int length = sprintf(buffer, "%d", value); // Chuyển đổi float sang chuỗi với 2 chữ số thập phân
	    //  ?ảm bảo độ dài chuỗi không vượt quá kích thước của tx_data
	       if (length > max_length - 1) {
	           length = max_length - 1;
	       }
	       // Sao chép chuỗi sang tx_data
	       memcpy(tx_data, buffer, length);
	       tx_data[length] = '  ';  // Thêm ký tự xuống hàng
	       tx_data[length + 1] = '\0'; //  ?ảm bảo kết thúc bằng ký tự null
	}

// Hàm doc nut nhan
char butt = 48;
void int_to_string(int number, char* buffer) {
    sprintf(buffer, "%d", number);   // Ham chuyen int sang string
}
void uint8_to_string(uint8_t number, char* str) {
    // Chuyển đổi uint8_t thành chuỗi và lưu vào str
    snprintf(str, "%u", number);
}
void readButtons()
{
	buttonStates[1]= HAL_GPIO_ReadPin(B_1_GPIO_Port, B_1_Pin);
	buttonStates[2]= HAL_GPIO_ReadPin(B_2_GPIO_Port, B_2_Pin);
	buttonStates[3]= HAL_GPIO_ReadPin(B_3_GPIO_Port, B_3_Pin);
	buttonStates[4]= HAL_GPIO_ReadPin(B_4_GPIO_Port, B_4_Pin);
	buttonStates[5]= HAL_GPIO_ReadPin(B_5_GPIO_Port, B_5_Pin);
	buttonStates[6]= HAL_GPIO_ReadPin(B_6_GPIO_Port, B_6_Pin);
	buttonStates[7]= HAL_GPIO_ReadPin(B_7_GPIO_Port, B_7_Pin);
	buttonStates[8]= HAL_GPIO_ReadPin(B_8_GPIO_Port, B_8_Pin);
//	buttonStates[9]= !HAL_GPIO_ReadPin(SW_GPIO_Port, SW_Pin);
    for (int i = 8; i >= 0; i--) {
        if (buttonStates[i] == 0) {
        	pressedButton = i;
            break;
        }
        else
        	pressedButton = 0;
    }

    switch (pressedButton)
		{
    case 1:
    		butt = 49;
    		break;
    case 2:
    		butt = 50;
    		break;
    case 3:
    		butt = 51;
    		break;
    case 4:
    		butt = 52;
    		break;
    case 5:
    		butt = 53;
    		break;
    case 6:
    		butt = 54;
    		break;
    case 7:
    		butt = 55;
    		break;
    case 8:
    		butt = 56;
    		break;
    case 9:
    		butt = 57;
    		break;
    default:
    	    butt = 48;
    	    break;
		}
	lcd_put_cur(1,6);
	lcd_send_data (butt);
}
// Hàm đoc Joystick
void ReadJoystic(){
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion (&hadc1, 10);
			ADC_X = HAL_ADC_GetValue(&hadc1) ;
			HAL_ADC_Start(&hadc2);
			HAL_ADC_PollForConversion (&hadc2, 10);
			ADC_Y = HAL_ADC_GetValue(&hadc2);
}
void  find_speed(short temp)
{
	switch ( temp ){
	case 0 :
			speed = '0';
			break;
	case 1 :
			speed = '1';
			break;
	case 2 :
			speed = '2';
			break;
	case 3 :
			speed = '3';
			break;
	case 4 :
			speed = '4';
			break;
	case 5 :
			speed = '5';
			break;
	case 6 :
			speed = '6';
			break;
	case 7 :
			speed = '7';
			break;
	case 8 :
			speed = '8';
			break;
	default:
			break;
	}
}
void  controler(short x,short y,char butt)
{
	if ( (x <= 1 && x >= -1 ) & ( y >= 2 || y <= -2) ){
		if( y >= 2 )

			control = 'D';
		if( y <= -2)
			control = 'A';
	}
	else if ( (y <= 1 && y >= -1 ) & ( x >= 2 || x <= -2) ){
		if( x >= 2 )
		control = 'W';
		if( x <= -2)
			control = 'S';

	}
	else if ( butt == 49 )
		control = 'R';
	else if ( butt == 50 )
		control = 'L';
	else
		control = 'n';
}
void  driver(int temp1,int temp2){
	char text[1];
	short temp;
	temp1=(temp1/ 4096.0)*24 - 7; // VarX
	temp2=(temp2/ 4096.0)*24 - 7; // VarY
	controler(temp1,temp2,butt);
	temp1=abs(temp1);
	temp2=abs(temp2);
	temp = temp1 ;
	if ( temp1 < temp2)
		temp = temp2;  // chuan hóa tôc đô
	find_speed(temp);  //
	int_to_string(temp,text);
	lcd_put_cur(0,0);
	lcd_send_string("Speed:");
	lcd_send_string(text);
	lcd_put_cur(0,8);
	lcd_send_string("Move:");
	lcd_send_data(control);
}

void led(short R,short G,short B)
{
	HAL_GPIO_WritePin(RGB_1_GPIO_Port, RGB_1_Pin, R);
	HAL_GPIO_WritePin(RGB_3_GPIO_Port, RGB_3_Pin, G);
}

// Hàm nhan tín hieu RF
void read_nRF(){// Hàm Nhận RF
 		if (nrf24l01p_rx_flag == 1) {
			nrf24l01p_rx_flag =0;
			uint8_t stat = read_register(NRF24L01P_REG_CONFIG);
			if (!(stat & (1 << 0))) {
				nrf24l01p_prx_mode();
			}
			//nrf24l01p_flush_rx_fifo();
			nrf24l01p_rx_receive(rx_data);
			HAL_UART_Transmit(&huart1, rx_data, sizeof(rx_data), 500);
			lcd_put_cur(1,11);
			lcd_send_data (rx_data[0]);

	   	    if(rx_data[0] == 'N') // Nếu có vật cản thì sáng đèn
	   	    	led(0,0,0);
	   	    else
	   	    	led(1,1,1);
		}
}

// Hàm truyen tin hieu RF
void trans(){
	memset(tx_data, 0, sizeof(tx_data));
	int num = 0;
// Ham kiem tra bang UART
	if (num = uart_available(&uart_rx)) {
		nrf24l01p_ptx_mode();

		for (int i = 0; i < num; i++) {

			int ch = pop(&uart_rx);
			if (ch != -1) {
				tx_data[i] = ch;
			}
		}
	nrf24l01p_flush_tx_fifo();
   	nrf24l01p_tx_transmit(tx_data);
		HAL_Delay(50);
		nrf24l01p_prx_mode();
		//nrf24l01p_rx_init(2500, _1Mbps, P0_address);
	}

}
void trans_nRF(){ // Hàm truyền RF
	memset(tx_data, 0, sizeof(tx_data));
	int num = 0;
// Sent Func
//	if (num = uart_available(&uart_rx)) {
	if (1) {
		nrf24l01p_ptx_mode();
		for (int i = 0; i < num; i++) {
			int ch = pop(&uart_rx);
			if (ch != -1) {
				tx_data[i] = ch;
			}
		}
		tx_data[0]=control;
		tx_data[1]=speed;
		tx_data[2]=butt;
		tx_data[3]=EN;
	    nrf24l01p_flush_tx_fifo();
   	    nrf24l01p_tx_transmit(tx_data);
		HAL_Delay(50);
		nrf24l01p_prx_mode();
		//nrf24l01p_rx_init(2500, _1Mbps, P0_address);
		nrf24l01p_flush_tx_fifo();
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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */


// LCD --------------------------------------------------------------------------
    lcd_init();
  	lcd_clear();
// --------------------------------------------------------------------------

	nrf24l01p_rx_init(2500, _1Mbps, P1_address, P0_address);  // Hàm gọi kênh
	rxBufferInit(&uart_rx);

	/* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_ERR);

	/* Enable the UART Data Register not empty Interrupt */
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

	HAL_ADC_Start(&hadc1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		ReadJoystic(); // read analog from joystic
		readButtons();
		driver(ADC_X,ADC_Y);

		if ( counter == 10)
				EN = '1';

		read_nRF();  	// Rx
		trans_nRF(); 	// Tx

		counter++;
		if ( counter == 11)
			{
			 EN = '0';
			 counter = 0;
			}


		/*

		 memset(tx_data, 0, sizeof(tx_data));

		 uint8_t stat = read_register(NRF24L01P_REG_CONFIG);
		 if ((stat & (1 << 0))) {

		 nrf24l01p_ptx_mode();
		 //nrf24l01p_tx_init(2500, _1Mbps, P0_address);
		 }

		 //nrf24l01p_tx_init(2500, _1Mbps, P0_address);
		 HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		 adcVal = HAL_ADC_GetValue(&hadc1);

		 sprintf(tx_data, "%.2f\n", adcVal);


		 nrf24l01p_flush_tx_fifo();

		 nrf24l01p_tx_transmit(tx_data);
		 HAL_Delay(10);
		 nrf24l01p_prx_mode();
		 */


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
  hadc1.Init.ContinuousConvMode = ENABLE;
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
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

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

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  HAL_GPIO_WritePin(check_GPIO_Port, check_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RGB_1_Pin|RGB_2_Pin|RGB_3_Pin|Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : check_Pin */
  GPIO_InitStruct.Pin = check_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(check_GPIO_Port, &GPIO_InitStruct);

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

  /*Configure GPIO pins : SW_Pin B_5_Pin B_4_Pin B_3_Pin
                           B_2_Pin B_1_Pin */
  GPIO_InitStruct.Pin = SW_Pin|B_5_Pin|B_4_Pin|B_3_Pin
                          |B_2_Pin|B_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RGB_1_Pin RGB_2_Pin RGB_3_Pin Led_Pin */
  GPIO_InitStruct.Pin = RGB_1_Pin|RGB_2_Pin|RGB_3_Pin|Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : B_8_Pin B_7_Pin B_6_Pin */
  GPIO_InitStruct.Pin = B_8_Pin|B_7_Pin|B_6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == NRF24L01P_IRQ_PIN_NUMBER) {
		nrf24l01p_rx_flag = 1;
	}

}
/*
 void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
 {
 adc_flag=1;
 HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY);
 adcVal = HAL_ADC_GetValue(&hadc1);
 sprintf(tx_data,"adc_val: %f\n",adcVal);

 //HAL_ADC_Stop(&hadc1);

 }
 */
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
