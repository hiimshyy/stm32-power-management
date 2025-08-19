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
#include "cmsis_os.h"
#include "usb_device.h"
#include "daly_bms.h"
#include "sk60x.h"
#include "debugger.h"
#include "ina219.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
osThreadId bmsTaskHandle;
osThreadId sk60xTaskHandle;
osThreadId ina219TaskHandle;
// Khai báo biến toàn cục cho 3 cảm biến INA219
INA219_t ina_12v, ina_5v, ina_3v3;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void const * argument);
void StartBMSTask(void const * argument);
void StartSK60xTask(void const * argument);
void StartTaskINA219(void const * argument);

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  DalyBMS_Set_Callback(DalyBMS_On_Request_Done);
  Debug_Init();
  Debug_SetMode(DEBUG_BOTH); // Set debug mode to both USB and UART
//  ModbusRTU_Init();
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 64);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of bmsTask */
  osThreadDef(bmsTask, StartBMSTask, osPriorityNormal, 0, 512);
  bmsTaskHandle = osThreadCreate(osThread(bmsTask), NULL);

  /* definition and creation of sk60xTask */
  osThreadDef(sk60xTask, StartSK60xTask, osPriorityNormal, 0, 256);
  sk60xTaskHandle = osThreadCreate(osThread(sk60xTask), NULL);

  /* definition and creation of ina219Task */
  osThreadDef(ina219Task, StartTaskINA219, osPriorityNormal, 0, 256);
  ina219TaskHandle = osThreadCreate(osThread(ina219Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  huart1.Init.BaudRate = 9600;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, LED_Pin|LED_FAULT_Pin|LED_UART_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RL_3V3_Pin|RL_5V_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RL_12V_Pin|RL_CHG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin LED_FAULT_Pin LED_UART_Pin */
  GPIO_InitStruct.Pin = LED_Pin|LED_FAULT_Pin|LED_UART_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RL_3V3_Pin RL_5V_Pin */
  GPIO_InitStruct.Pin = RL_3V3_Pin|RL_5V_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RL_12V_Pin RL_CHG_Pin */
  GPIO_InitStruct.Pin = RL_12V_Pin|RL_CHG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
    /* Infinite loop */
    for(;;)
    {
		HAL_GPIO_WritePin(GPIOA, RL_3V3_Pin|RL_5V_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, RL_12V_Pin|RL_CHG_Pin, GPIO_PIN_SET);
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		osDelay(500);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartBMSTask */
/**
* @brief Function implementing the bmsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBMSTask */
void StartBMSTask(void const * argument)
{
  /* USER CODE BEGIN StartBMSTask */
	memset(_tx_buffer, 0x00, FRAME_SIZE);
	DalyBMS_Clear_Get(&bms_data);
	/* Infinite loop */
	for(;;)
	{
		switch (_request_counter)
	    {
	  		case 0: // Request pack data & connectivity status
	  			if (DalyBMS_Get_Pack_Data())
	  			{
	  				HAL_GPIO_WritePin(LED_GPIO_Port, LED_UART_Pin, GPIO_PIN_SET);
	  				bms_data.connection_status = true;
	  				_error_counter = 0; // Reset error counter on successful data retrieval
	  				_request_counter++;
	  			}
	  			else
	  			{
	  				HAL_GPIO_WritePin(LED_GPIO_Port, LED_UART_Pin, GPIO_PIN_RESET);
	  				_request_counter = 0; // Reset request counter on failure
	  				if (_error_counter < MAX_ERROR) {
	  					_error_counter++;
	  				}
	  				else
	  				{
	  					bms_data.connection_status = false; // Set connection status to false after max errors
	  					_error_counter = 0; // Reset error counter after max errors
	  					if (_bms_request_callback) _bms_request_callback();
	  				}
	  			}
	  			//Debug_Printf("BMS request counter: %d\n", _request_counter);
	  			break;
	  		case 1: // Request min/max cell voltage
	  			_request_counter = DalyBMS_Get_Min_Max_Cell_Voltage() ? (_request_counter + 1) : 0;
//	  			Debug_Printf("BMS request counter: %d\n", _request_counter);
	  			break;
	  		case 2: // Request min/max temperature
	  			_request_counter = DalyBMS_Get_Pack_Temperature() ? (_request_counter + 1) : 0;
//	  			Debug_Printf("BMS request counter: %d\n", _request_counter);
	  			break;
	  		case 3: // Request charge/discharge MOS status
	  			_request_counter = DalyBMS_Get_Charge_Discharge_Status() ? (_request_counter + 1) : 0;
//	  			Debug_Printf("BMS request counter: %d\n", _request_counter);
	  			break;
	  		case 4: // Request status info
	  			//Debug_Printf("In case 4!\n");
	  			_request_counter = DalyBMS_Get_Status_Info() ? (_request_counter + 1) : 0;
//	  			Debug_Printf("BMS request counter: %d\n", _request_counter);
	  			break;
	  		case 5: // Request cell voltages
	  			_request_counter = DalyBMS_Get_Cell_Voltages() ? (_request_counter + 1) : 0;
//	  			Debug_Printf("BMS request counter: %d\n", _request_counter);
	  			break;
	  		case 6: // Request cell temperatures
	  			_request_counter = DalyBMS_Get_Cell_Temperatures() ? (_request_counter + 1) : 0;
//	  			Debug_Printf("BMS request counter: %d\n", _request_counter);
	  			break;
	  		case 7: // Request cell balance state
	  			_request_counter = DalyBMS_Get_Cell_Balance_State() ? (_request_counter + 1) : 0;
//	  			Debug_Printf("BMS request counter: %d\n", _request_counter);
	  			break;
	  		case 8: // Request failure codes
	  			_request_counter = DalyBMS_Get_Failure_Codes() ? (_request_counter + 1) : 0;
	  			if (_get_static_data) _request_counter = 0; // Reset request counter if static data is requested
	  			if (_bms_request_callback) _bms_request_callback();
	  			break;
	  		case 9: // Request Voltage Thresholds
	  			if (!_get_static_data) _request_counter = DalyBMS_Get_Voltage_Thresholds() ? (_request_counter + 1) : 0;
	  			if (_bms_request_callback) _bms_request_callback();
	  			break;
	  		case 10: // Request Pack Thresholds
	  			if (!_get_static_data) _request_counter = DalyBMS_Get_Pack_Thresholds() ? (_request_counter + 1) : 0;
	  			_request_counter = 0; // Reset request counter after pack thresholds
	  			if (_bms_request_callback) _bms_request_callback();
	  			_get_static_data = true;
	  			break;
	  		default:
	  			break;
	    }
    osDelay(150);
  }
  /* USER CODE END StartBMSTask */
}

/* USER CODE BEGIN Header_StartSK60xTask */
/**
* @brief Function implementing the sk60xTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSK60xTask */
void StartSK60xTask(void const * argument)
{
  /* USER CODE BEGIN StartSK60xTask */
  /* Infinite loop */
  for(;;)
  {
	  if (SK60X_Read_Data())
	  {
		  Debug_Printf("<SK60x> - Voltage set: %d V\n", sk60x_data.v_set);
		  Debug_Printf("<SK60x> - Current set: %d A\n", sk60x_data.i_set);
		  Debug_Printf("<SK60x> - Voltage: %d V\n", sk60x_data.v_out);
		  Debug_Printf("<SK60x> - Current: %d A\n", sk60x_data.i_out);
		  Debug_Printf("<SK60x> - Power: %d W\n", sk60x_data.p_out);
		  Debug_Printf("<SK60x> - Voltage in: %d V\n", sk60x_data.v_in);
		  Debug_Printf("<SK60x> - Current in: %d V\n", sk60x_data.i_in);
		  Debug_Printf("<SK60x> - Temperature: %d C\n", sk60x_data.temp);
		  Debug_Printf("<SK60x> - Volage lock: %d \n", sk60x_data.lock_v);
		  Debug_Printf("<SK60x> - Time used: %d h %d m %d s\n", sk60x_data.h_use, sk60x_data.m_use, sk60x_data.s_use);
		  Debug_Printf("<SK60x> - Is using: %d \n", sk60x_data.status);
		  Debug_Printf("<SK60x> - Status: %d\n", sk60x_data.on_off);
	  }
	  else
	  {
		  Debug_Printf("<SK60x> - Failed to read data!\n");
	}
    osDelay(500);
  }
  /* USER CODE END StartSK60xTask */
}

/* USER CODE BEGIN Header_StartTaskINA219 */
/**
* @brief Function implementing the ina219Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskINA219 */
void StartTaskINA219(void const * argument)
{
  /* USER CODE BEGIN StartTaskINA219 */
  INA219_Init(&ina_12v, &hi2c1, INA219_ADDR_12V, 0.1f, 3.0f);   // Rshunt = 0.1Ω, dòng tối đa 3A
  INA219_Init(&ina_5v,  &hi2c1, INA219_ADDR_5V,  0.1f, 3.0f);
  INA219_Init(&ina_3v3, &hi2c1, INA219_ADDR_3V3, 0.1f, 3.0f);

  float v, i, p;
  HAL_StatusTypeDef ret;
  /* Infinite loop */
  for(;;)
  {
    // Đọc và in giá trị INA219 12V
    ret = INA219_Read_Bus_Voltage(&ina_12v, &v);
    if(ret == HAL_OK) Debug_Printf("INA219-12V: V=%.3fV ", v);
    ret = INA219_Read_Current(&ina_12v, &i);
    if(ret == HAL_OK) Debug_Printf("I=%.3fA ", i);
    ret = INA219_Read_Power(&ina_12v, &p);
    if(ret == HAL_OK) Debug_Printf("P=%.3fW\n", p);

    // Đọc và in giá trị INA219 5V
    ret = INA219_Read_Bus_Voltage(&ina_5v, &v);
    if(ret == HAL_OK) Debug_Printf("INA219-5V: V=%.3fV ", v);
    ret = INA219_Read_Current(&ina_5v, &i);
    if(ret == HAL_OK) Debug_Printf("I=%.3fA ", i);
    ret = INA219_Read_Power(&ina_5v, &p);
    if(ret == HAL_OK) Debug_Printf("P=%.3fW\n", p);

    // Đọc và in giá trị INA219 3V3
    ret = INA219_Read_Bus_Voltage(&ina_3v3, &v);
    if(ret == HAL_OK) Debug_Printf("INA219-3V3: V=%.3fV ", v);
    ret = INA219_Read_Current(&ina_3v3, &i);
    if(ret == HAL_OK) Debug_Printf("I=%.3fA ", i);
    ret = INA219_Read_Power(&ina_3v3, &p);
    if(ret == HAL_OK) Debug_Printf("P=%.3fW\n", p);

    osDelay(1000);
  }
  /* USER CODE END StartTaskINA219 */
}

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
	  HAL_GPIO_TogglePin(GPIOC, LED_FAULT_Pin);
	  osDelay(100);
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
