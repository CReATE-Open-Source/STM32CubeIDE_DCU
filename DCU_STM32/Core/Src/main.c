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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RPM_ECO 	 2262;
#define RPM_NORMAL 	 3548;
#define RPM_SPORT	 4060;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

osThreadId canTransmitHandle;
osThreadId canRcvtaskHandle;
osThreadId parsePXNTaskHandle;
osThreadId parseVotolTaskHandle;
osThreadId ackermanHandle;
osThreadId throttleVotolHandle;
osThreadId can_rcvPXNTaskHandle;
/* USER CODE BEGIN PV */
uint8_t tes1 = 0;
uint8_t tes2 = 0;
uint16_t out_tes = 0;

uint8_t rxtxBuffer[100];

//int flag_trigPXN = 0;
//int flag_trigVotol = 0;
//int steering_angle=0, throttle=0, trig_cntr =0;
//float vehicle_speed = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM2_Init(void);
static void MX_CAN2_Init(void);
void can_transmit(void const * argument);
void can_rcv(void const * argument);
void parsePXN(void const * argument);
void parseSetVotol(void const * argument);
void ackerman_func(void const * argument);
void throttleVotolTask(void const * argument);
void canRcvPXN(void const * argument);

/* USER CODE BEGIN PFP */
//void getSpeed(void);
//void getWheelSpeed(void);
//void getAngularSpeed(void);
void MotorData(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ================= Variable for CAN ================= */
CAN_TxHeaderTypeDef   TxHeader1; //for LeftVotol1
CAN_TxHeaderTypeDef   TxHeader2; //for LeftVotol2
CAN_TxHeaderTypeDef   TxHeader3; //for LeftVotol1
CAN_TxHeaderTypeDef   TxHeader4; //for LeftVotol2
CAN_TxHeaderTypeDef   TxHeader5; //for PXN Low Voltage
CAN_TxHeaderTypeDef   TxHeader6; //for PXN Low Voltage 2
CAN_TxHeaderTypeDef   TxHeader7; //for PXN Low Voltage 3

CAN_RxHeaderTypeDef   RxHeader;    //for Receive CAN1
uint8_t               RxData[8] = {0};
CAN_RxHeaderTypeDef   RxHeaderPXN; //for Receive CAN2
uint8_t               RxDataPXN[8] = {0};

uint8_t               TxData1[8]; //for LeftVotol1
uint8_t               TxData2[8]; //for LeftVotol2
uint8_t               TxData3[8]; //for LeftVotol1
uint8_t               TxData4[8]; //for LeftVotol2
uint8_t               TxData5[8]; //for PXN Low Voltage
uint8_t               TxData6[8]; //for PXN Low Voltage 2
uint8_t               TxData7[8]; //for PXN Low Voltage 3

uint32_t              TxMailbox; //mailbox

HAL_StatusTypeDef	CAN_State;
HAL_StatusTypeDef	CAN_State2;
/* ================= Variable for Votol Data ================= */
uint8_t  L_ControllerTemp = 0xEE, L_ExternalTemp = 0xFF, L_GearStatus = 0xCC, L_ControllerStatus = 0xDD;
uint8_t  Vehicle_Speed = 0x00;
uint16_t L_MotorVoltage = 0xAABB, L_MotorCurrent = 0xCCDD, L_MotorRPM = 0x0000, L_TempCoef = 0xAABB;
uint32_t L_MotorFC = 0x08100000;

uint8_t  R_ControllerTemp = 0, R_ExternalTemp = 0, R_GearStatus = 0, R_ControllerStatus = 0;
uint16_t R_MotorVoltage = 0, R_MotorCurrent = 0, R_MotorRPM = 0x1000, R_TempCoef = 0;
uint32_t R_MotorFC = 0;

float angular_speed1 = 0.0, angular_speed2 = 0.0;
float wheel_speed1, wheel_speed2;
float angular_speed;
float turning_radius;
float vehicle_speed = 0;
float wheels_radius = 0.19; //(m)
float L = 2.3; //(m) vehicle body length
float d = 1.46; //(m) vehicle body width

int RPM_th = RPM_NORMAL;
/* ================= Variable for UART ================= */
uint8_t transmitBuffer[100];

/* ================= Variable for PXN Data ================= */
float SteeringWheel = 0;
int SteeringWheelInt = 0;
int Pedal = 0;
uint16_t type;
uint16_t code;
int32_t value = 0;

/* ================= Variable for Votol Input ================= */
uint16_t setDAC1=(float)1.25*4095/2.95, setDAC2=(float)1.25*4095/2.95, maxSpeed1=4095*0.5;
uint16_t targetDAC1=(float)1.25*4095/2.95, targetDAC2=(float)1.25*4095/2.95, maxSpeed2=4095*0.5;

/* ================= Timer Variable ===================== */
uint16_t timer_val = 0;

/* ================= Low Voltage System ===================== */
int data1, data2, LB, HB, FL, DL;
uint8_t  Byte0 = 0x1F, Byte1 = 0x0C, door = 0x02;

/* ================= bitRead & bitSet ================== */
// Define untuk mengambil bit ke-n dari sebuah byte
#define bitRead(value, bit) (((value) >> (bit)) & 0x03)
// Define untuk mengganti bit ke-n dari sebuah byte
#define bitSet(byte, n, value) ((value) ? (byte |= (1UL << (n))) : (byte &= ~(1UL << (n))))
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
  MX_CAN1_Init();
  MX_USART3_UART_Init();
  MX_DAC_Init();
  MX_TIM2_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1); //****************************
  HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, (setDAC1));
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2,DAC_ALIGN_12B_R, (setDAC2));
  //HAL_TIM_Base_Start(&htim2);
  //HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_TIM_Base_Start(&htim2);

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
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of canTransmit */
  osThreadDef(canTransmit, can_transmit, osPriorityAboveNormal, 0, 128);
  canTransmitHandle = osThreadCreate(osThread(canTransmit), NULL);

  /* definition and creation of canRcvtask */
  osThreadDef(canRcvtask, can_rcv, osPriorityHigh, 0, 128);
  canRcvtaskHandle = osThreadCreate(osThread(canRcvtask), NULL);

  /* definition and creation of parsePXNTask */
  osThreadDef(parsePXNTask, parsePXN, osPriorityNormal, 0, 128);
  parsePXNTaskHandle = osThreadCreate(osThread(parsePXNTask), NULL);

  /* definition and creation of parseVotolTask */
  osThreadDef(parseVotolTask, parseSetVotol, osPriorityBelowNormal, 0, 128);
  parseVotolTaskHandle = osThreadCreate(osThread(parseVotolTask), NULL);

  /* definition and creation of ackerman */
  osThreadDef(ackerman, ackerman_func, osPriorityAboveNormal, 0, 128);
  ackermanHandle = osThreadCreate(osThread(ackerman), NULL);

  /* definition and creation of throttleVotol */
  osThreadDef(throttleVotol, throttleVotolTask, osPriorityRealtime, 0, 128);
  throttleVotolHandle = osThreadCreate(osThread(throttleVotol), NULL);

  /* definition and creation of can_rcvPXNTask */
  osThreadDef(can_rcvPXNTask, canRcvPXN, osPriorityRealtime, 0, 128);
  can_rcvPXNTaskHandle = osThreadCreate(osThread(can_rcvPXNTask), NULL);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 12;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef canfilterconfig;
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 10;
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO1;
  canfilterconfig.FilterIdHigh = 0;
  canfilterconfig.FilterIdLow = 0x0000;
  canfilterconfig.FilterMaskIdHigh = 0;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 0;

  if(HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig) != HAL_OK)
    {
      Error_Handler();
    }

  if(HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
    {
      /* Notification Error */
      Error_Handler();
    }

  TxHeader1.IDE = CAN_ID_EXT;
  TxHeader1.ExtId = 0xCF003FE; //Left Motor Voltage 1
  TxHeader1.RTR = CAN_RTR_DATA;
  TxHeader1.DLC = 8;

  TxHeader2.IDE = CAN_ID_EXT;
  TxHeader2.ExtId = 0xCF004FE; //Left Motor Voltage 2
  TxHeader2.RTR = CAN_RTR_DATA;
  TxHeader2.DLC = 8;

  TxHeader3.IDE = CAN_ID_EXT;
  TxHeader3.ExtId = 0xCFEDFFE; //Right Motor Voltage 1
  TxHeader3.RTR = CAN_RTR_DATA;
  TxHeader3.DLC = 8;

  TxHeader4.IDE = CAN_ID_EXT;
  TxHeader4.ExtId = 0xCFEBEFE; //Right Motor Voltage 2
  TxHeader4.RTR = CAN_RTR_DATA;
  TxHeader4.DLC = 8;

  TxHeader5.IDE = CAN_ID_EXT;
  TxHeader5.ExtId = 0x18FE401E; //Low Voltage PXN
  TxHeader5.RTR = CAN_RTR_DATA;
  TxHeader5.DLC = 8;

  TxHeader6.IDE = CAN_ID_EXT;
  TxHeader6.ExtId = 0x18FE4EEC; //Low Voltage PXN 2
  TxHeader6.RTR = CAN_RTR_DATA;
  TxHeader6.DLC = 8;

  TxHeader7.IDE = CAN_ID_EXT;
  TxHeader7.ExtId = 0x98F001FE; //Low Voltage PXN 3
  TxHeader7.RTR = CAN_RTR_DATA;
  TxHeader7.DLC = 8;
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 12;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
  CAN_FilterTypeDef canfilterconfig2;
  canfilterconfig2.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig2.FilterBank = 10;
  canfilterconfig2.FilterFIFOAssignment = CAN_RX_FIFO1;
  canfilterconfig2.FilterIdHigh = 0;
  canfilterconfig2.FilterIdLow = 0x0000;
  canfilterconfig2.FilterMaskIdHigh = 0;
  canfilterconfig2.FilterMaskIdLow = 0x0000;
  canfilterconfig2.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig2.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig2.SlaveStartFilterBank = 0;

    if(HAL_CAN_ConfigFilter(&hcan2, &canfilterconfig2) != HAL_OK)
      {
        Error_Handler();
      }

    if(HAL_CAN_Start(&hcan2) != HAL_OK)
        {
          Error_Handler();
        }

    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
      {
        /* Notification Error */
        Error_Handler();
      }
  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 168-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED1_Pin|LED2_Pin|cruisePin1_Pin|highSpeedPin2_Pin
                          |reversePin2_Pin|lowSpeedPin2_Pin|cruisePin2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, highSpeedPin1_Pin|reversePin1_Pin|lowSpeedPin1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin cruisePin1_Pin highSpeedPin2_Pin
                           reversePin2_Pin lowSpeedPin2_Pin cruisePin2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|cruisePin1_Pin|highSpeedPin2_Pin
                          |reversePin2_Pin|lowSpeedPin2_Pin|cruisePin2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : highSpeedPin1_Pin reversePin1_Pin lowSpeedPin1_Pin */
  GPIO_InitStruct.Pin = highSpeedPin1_Pin|reversePin1_Pin|lowSpeedPin1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//CAN Rcv interrupt callback
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	//Receive from AAOS
	CAN_State = HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO1, &RxHeader, RxData);
	//timer_val = __HAL_TIM_GET_COUNTER(&htim2) - timer_val;

	//Receive from PXN
	CAN_State2 = HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO1, &RxHeaderPXN, RxDataPXN);
}

/*
void getSpeed(void)
{
	vehicle_speed = L_MotorRPM * 6000 / 255; //in RPM
	vehicle_speed = (vehicle_speed * (2*3.14))/60; // in rad/s
	vehicle_speed = vehicle_speed * wheels_radius; // (m/s)
}

void getWheelSpeed(void)
{
	angular_speed = vehicle_speed * tanf(SteeringWheelInt*3.14/180)/L;
	turning_radius = vehicle_speed/angular_speed;

	wheel_speed1 = angular_speed * (turning_radius+(d/2));
	wheel_speed2 = angular_speed * (turning_radius-(d/2));
}

void getAngularSpeed(void)
{
	angular_speed1 = wheel_speed1 / wheels_radius;
	angular_speed2 = wheel_speed2 / wheels_radius;
}*/

void MotorData(void)
{
	/* LEFT MOTOR */
	TxData1[1] = L_MotorVoltage;
	TxData1[0] = L_MotorVoltage >> 8;
	TxData1[3] = L_MotorCurrent;
	TxData1[2] = L_MotorCurrent >> 8;
	TxData1[7] = L_MotorFC;
	TxData1[6] = L_MotorFC >> 8;
	TxData1[5] = L_MotorFC >> 16;
	TxData1[4] = L_MotorFC >> 24;

	TxData2[1] = L_MotorRPM;
	TxData2[0] = L_MotorRPM >> 8;
	//TxData2[2] = L_Cont	rollerTemp;
	//Vehicle_Speed = 5;
	TxData2[2] = Vehicle_Speed;
	TxData2[3] = L_ExternalTemp;
	TxData2[5] = L_TempCoef;
	TxData2[4] = L_TempCoef >> 8;
	TxData2[6] = L_GearStatus;
	TxData2[7] = L_ControllerStatus;

	/* RIGHT MOTOR */
	TxData3[1] = R_MotorVoltage;
	TxData3[0] = R_MotorVoltage >> 8;
	TxData3[3] = R_MotorCurrent;
	TxData3[2] = R_MotorCurrent >> 8;
	TxData3[7] = R_MotorFC;
	TxData3[6] = R_MotorFC >> 8;
	TxData3[5] = R_MotorFC >> 16;
	TxData3[4] = R_MotorFC >> 24;

	TxData4[1] = R_MotorRPM;
	TxData4[0] = R_MotorRPM >> 8;
	TxData4[2] = R_ControllerTemp;
	TxData4[3] = R_ExternalTemp;
	TxData4[5] = R_TempCoef;
	TxData4[4] = R_TempCoef >> 8;
	TxData4[6] = R_GearStatus;
	TxData4[7] = R_ControllerStatus;
}

void PXNLowVoltageData(void)
{
	TxData5 [0] = Byte0;
	TxData5 [1] = Byte1;
	TxData5 [2] = 0xFF;
	TxData5 [3] = 0xFF;
	TxData5 [4] = 0xFC;
	TxData5 [5] = 0xFF;
	TxData5 [6] = 0xFF;
	TxData5 [7] = door;

	TxData6 [0] = door;
	TxData6 [1] = 0xF3;
	TxData6 [2] = 0xFF;
	TxData6 [3] = 0x3F;
	TxData6 [4] = 0xFF;
	TxData6 [5] = 0xFF;
	TxData6 [6] = 0xFF;
	TxData6 [7] = 0xFF;
	//Hasan
	TxData7[7] = door;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_can_transmit */
/**
  * @brief  Function implementing the canTransmit thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_can_transmit */
void can_transmit(void const * argument)
{
  /* USER CODE BEGIN 5 */

  /* Infinite loop */
	for(;;)
	  {

		MotorData();
		PXNLowVoltageData();
		//Transmit LeftVotol1 Data
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader1, TxData1, &TxMailbox) != HAL_OK)
		{
			/* Transmission request Error */
			Error_Handler();
		}
		HAL_Delay(10);
		sprintf(transmitBuffer, "CAN ID : %0X, Data : %02X %02X %02X %02X %02X %02X %02X %02X\n", TxHeader1.ExtId, TxData1[0], TxData1[1], TxData1[2], TxData1[3], TxData1[4], TxData1[5], TxData1[6], TxData1[7]);
		HAL_UART_Transmit_IT(&huart3, transmitBuffer, 50);
		memset(transmitBuffer, '\0', sizeof(transmitBuffer));

		//Transmit LeftVotol2 Data
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader2, TxData2, &TxMailbox) != HAL_OK)
		{
			/* Transmission request Error */
			Error_Handler();
		}
		HAL_Delay(10);
		sprintf(transmitBuffer, "CAN ID : %0X, Data : %02X %02X %02X %02X %02X %02X %02X %02X\n", TxHeader2.ExtId, TxData2[0], TxData2[1], TxData2[2], TxData2[3], TxData2[4], TxData2[5], TxData2[6], TxData2[7]);
		HAL_UART_Transmit_IT(&huart3, transmitBuffer, 50);
		memset(transmitBuffer, '\0', sizeof(transmitBuffer));

		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader3, TxData3, &TxMailbox) != HAL_OK)
		{
			/* Transmission request Error */
			Error_Handler();
		}
		HAL_Delay(10);
		sprintf(transmitBuffer, "CAN ID : %0X, Data : %02X %02X %02X %02X %02X %02X %02X %02X\n", TxHeader3.ExtId, TxData3[0], TxData3[1], TxData3[2], TxData3[3], TxData3[4], TxData3[5], TxData3[6], TxData3[7]);
		HAL_UART_Transmit_IT(&huart3, transmitBuffer, 50);
		memset(transmitBuffer, '\0', sizeof(transmitBuffer));

		//Transmit LeftVotol2 Data
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader4, TxData4, &TxMailbox) != HAL_OK)
		{
			/* Transmission request Error */
			Error_Handler();
		}
		HAL_Delay(10);

		//Transmit Low Voltage
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader5, TxData5, &TxMailbox) != HAL_OK)
		{
			/* Transmission request Error */
			Error_Handler();
		}
		HAL_Delay(10);

		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader6, TxData6, &TxMailbox) != HAL_OK)
		{
			/* Transmission request Error */
			Error_Handler();
		}
		HAL_Delay(10);

		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader7, TxData7, &TxMailbox) != HAL_OK)
		{
			/* Transmission request Error */
			Error_Handler();
		}
		HAL_Delay(10);
		sprintf(transmitBuffer, "CAN ID : %0X, Data : %02X %02X %02X %02X %02X %02X %02X %02X\n", TxHeader4.ExtId, TxData4[0], TxData4[1], TxData4[2], TxData4[3], TxData4[4], TxData4[5], TxData4[6], TxData4[7]);
		HAL_UART_Transmit_IT(&huart3, transmitBuffer, 50);
		memset(transmitBuffer, '\0', sizeof(transmitBuffer));

//
//		if(TxMailbox >= 3)
//		{
//			TxMailbox = 0;
//		}
//
//		if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader4, TxData4, &TxMailbox) != HAL_OK)
//		{
//			/* Transmission request Error */
//			Error_Handler();
//		}
//		HAL_Delay(10);

		osDelay(200);
	  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_can_rcv */
/**
* @brief Function implementing the canRcvtask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_can_rcv */
void can_rcv(void const * argument)
{
  /* USER CODE BEGIN can_rcv */
	osThreadSuspend(parseVotolTaskHandle);
	osThreadSuspend(ackermanHandle);
  /* Infinite loop */

  for(;;)
  {
	//printf ("In Can Rcv\n");
	 //Timer Start!
	//timer_val = __HAL_TIM_GET_COUNTER(&htim2);
	if (CAN_State == HAL_OK)
	  {
		HAL_GPIO_TogglePin (LED1_GPIO_Port, LED1_Pin);
		//Tes!
		/*sprintf(transmitBuffer, "CAN ID : %0X, Data : %02X %02X %02X %02X %02X %02X %02X %02X\n", RxHeader.ExtId, RxData[0], RxData[1], RxData[2], RxData[3], RxData[4], RxData[5], RxData[6], RxData[7]);
		HAL_UART_Transmit_IT(&huart3, transmitBuffer, 50);*/
		if(RxHeader.ExtId == 0xCFDA3FE)
		{
			//resume parse votol
			osThreadResume(parseVotolTaskHandle);
			//HAL_GPIO_TogglePin (LED2_GPIO_Port, LED2_Pin);

		}

		//HAL_Delay(10);
		//CAN_State == HAL
		// Get time elapsed

	  }

	//HAL_Delay(150);
    osDelay(1);
  }
  /* USER CODE END can_rcv */
}

/* USER CODE BEGIN Header_parsePXN */
/**
* @brief Function implementing the parsePXNTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_parsePXN */
void parsePXN(void const * argument)
{
  /* USER CODE BEGIN parsePXN */

  /* Infinite loop */
  for(;;)
  {
	type = RxDataPXN[0];
	code = (RxDataPXN[2] << 8) ^ RxDataPXN[1];
	value = (RxDataPXN[3] << 8) ^ RxDataPXN[4];
	
	FL = bitRead(Byte1, 0);//Byte1 undefined
	DL = bitRead(door, 0); //door undefined
	LB = bitRead(Byte0, 4);//Byte0 undefined
	HB = bitRead(Byte0, 6);//Byte0 undefined
	data1 = bitRead(Byte1, 6); //Byte1 undefined
	data2 = bitRead(Byte1, 4); //Byte1 undefined
	
	if(code == 0) //steering
	{
	  	value -= 32768; // to range -32768 - 32768
	  	SteeringWheel = (float)((value*1.0) / 32768) * 135; //conversion to degree
	  	SteeringWheelInt = (int)SteeringWheel;
	}
	else if(code == 1280) //pedal
	{
	  	Pedal = value;

	}
	
	//Low Voltage
	
	/**Front Light & Direct Light??*/
	if(RxDataPXN[2] == 0x3B && RxDataPXN[4] == 0x01)
	{
		if (FL == 0x00)
		{
			//HAL_GPIO_WritePin(FL_GPIO_Port, FL_Pin, GPIO_PIN_SET);
			bitSet(Byte1, 0, 1); //bitset function not defined
			bitSet(Byte1, 1, 0);
			//TxData [1] = Byte1; //TxData not defined
		} 
		
		else if (FL == 0x01) 
		{
			//HAL_GPIO_WritePin(FL_GPIO_Port, FL_Pin, GPIO_PIN_RESET);
			bitSet(Byte1, 0, 0);
			bitSet(Byte1, 1, 0);
			//TxData [1] = Byte1;
		}
		
	}
	/*Direct Light*/
	else if(RxDataPXN[2] == 0x3A && RxDataPXN[4] == 0x01)
	{
		if (DL == 0x02)
		{
			//HAL_GPIO_WritePin(DL_GPIO_Port, DL_Pin, GPIO_PIN_SET);
			bitSet(door, 0, 0);
			bitSet(door, 1, 0);
			//TxData2 [0] = door;
			//TxData3 [7] = door;
		} 
		
		else if (DL == 0x00) 
		{
			//HAL_GPIO_WritePin(DL_GPIO_Port, DL_Pin, GPIO_PIN_RESET);
			bitSet(door, 0, 0);
			bitSet(door, 1, 1);
			//TxData2 [0] = door;
			//TxData3 [7] = door;
		}
	}
	else if(RxDataPXN[2] == 0x33 && RxDataPXN[4] == 0x01)
	{
		if((LB == 0x01 && HB == 0x00) || (LB == 0x02 && HB == 0x00) || (LB == 0x03 && HB == 0x00))
		{
			bitSet(Byte0, 4, 0);
			bitSet(Byte0, 5, 0);
			bitSet(Byte0, 6, 1);
			bitSet(Byte0, 7, 0);
			//TxData [0] = Byte0;
		}
		else if ((LB == 0x00 && HB == 0x01) || (LB == 0x00 && HB == 0x02) || (LB == 0x00 && HB == 0x03))
		{
			bitSet(Byte0, 4, 1);
			bitSet(Byte0, 5, 0);
			bitSet(Byte0, 6, 0);
			bitSet(Byte0, 7, 0);
			//TxData [0] = Byte0;
		}
	}
	else if(RxDataPXN[2] == 0x02 && RxDataPXN[4] == 0xFF)
	{
		if(data1 == 0x00 || (data1 == 0x01  && data2 == 0x01))
		{
			//adjust the variable to current variable
			bitSet(Byte1, 4, 0);
			bitSet(Byte1, 5, 0);
			bitSet(Byte1, 6, 1);
			bitSet(Byte1, 7, 0);
			//TxData [1] = Byte1;
		} 
		else 
		{
			//adjust the variable to current variable
			bitSet(Byte1, 6, 0);
			bitSet(Byte1, 7, 0);
			//TxData [1] = Byte1;
		}
	}
	else if(RxDataPXN[2] == 0x05 && RxDataPXN[4] == 0xFF)
	{
		if(data2 == 0x00 || (data1 == 0x01  && data2 == 0x01))
		{
			//adjust the variable to current variable
			bitSet(Byte1, 4, 1);
			bitSet(Byte1, 5, 0);
			bitSet(Byte1, 6, 0);
			bitSet(Byte1, 7, 0);
			//TxData [1] = Byte1;
		} 
		else 
		{
			//adjust the variable to current variable
			bitSet(Byte1, 4, 0);
			bitSet(Byte1, 5, 0);
			//TxData [1] = Byte1;
		}
	}
	else if(RxDataPXN[2] == 0x34 && RxDataPXN[4] == 0x01)
	{
		if(data1 == 0x00  || data2 == 0x00)
		{
			bitSet(Byte1, 4, 1);
			bitSet(Byte1, 5, 0);
			bitSet(Byte1, 6, 1);
			bitSet(Byte1, 7, 0);
			//TxData [1] = Byte1;
		} 
		else 
		{
			bitSet(Byte1, 4, 0);
			bitSet(Byte1, 5, 0);
			bitSet(Byte1, 6, 0);
			bitSet(Byte1, 7, 0);
			//TxData [1] = Byte1;
		}
	}
	  		//sprintf(transmitBuffer, "type : %d code : %d data : $d", type, code, value);
//	  		sprintf(transmitBuffer, "steering wheel : %d, Pedal : %d\n", SteeringWheelInt, L_MotorRPM);
	  		// Get time elapsed
	  		//timer_val = __HAL_TIM_GET_COUNTER(&htim2) - timer_val;
	  		//sprintf(transmitBuffer, "%d", timer_val);
	  		//HAL_UART_Transmit_IT(&huart3, transmitBuffer, 50-1);
	  		//HAL_Delay(25);
//	  		memset(transmitBuffer, '\0', sizeof(transmitBuffer));

	  		//HAL_UART_Transmit_IT(&huart3, "\n", 1);
	osThreadResume(ackermanHandle);
	osThreadSuspend(parsePXNTaskHandle);
    osDelay(1);
  }
  /* USER CODE END parsePXN */
}

/* USER CODE BEGIN Header_parseSetVotol */
/**
* @brief Function implementing the parseVotolTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_parseSetVotol */
void parseSetVotol(void const * argument)
{
  /* USER CODE BEGIN parseSetVotol */
  uint8_t CurrMotorMode = -1;
  uint8_t SetMotorMode = -1;

  uint8_t CurrMotorGear = -1;
  uint8_t SetMotorGear = -1;
  /* Infinite loop */
  for(;;)
  {
	 SetMotorMode = RxData[0];
	 SetMotorGear = RxData[1];

	 if(SetMotorMode != CurrMotorMode)
	 {
		 switch(SetMotorMode)
		 	{
		 		case 0x00 : //High
		 					//Motor 1
		 					HAL_GPIO_WritePin(lowSpeedPin1_GPIO_Port, lowSpeedPin1_Pin, GPIO_PIN_RESET);
		 					HAL_GPIO_WritePin(highSpeedPin1_GPIO_Port, highSpeedPin1_Pin, GPIO_PIN_SET);
		 					//Motor 2
		 					HAL_GPIO_WritePin(lowSpeedPin2_GPIO_Port, lowSpeedPin2_Pin, GPIO_PIN_RESET);
		 					HAL_GPIO_WritePin(highSpeedPin2_GPIO_Port, highSpeedPin2_Pin, GPIO_PIN_SET);
		 					CurrMotorMode = 0x00;
		 					RPM_th = RPM_SPORT;
		 					break;
		 		case 0x01 : //Medium
		 //					//Motor 1
		 					HAL_GPIO_WritePin(lowSpeedPin1_GPIO_Port, lowSpeedPin1_Pin, GPIO_PIN_RESET);
		 					HAL_GPIO_WritePin(highSpeedPin1_GPIO_Port, highSpeedPin1_Pin, GPIO_PIN_RESET);
		 					//Motor 2
		 					HAL_GPIO_WritePin(lowSpeedPin2_GPIO_Port, lowSpeedPin2_Pin, GPIO_PIN_RESET);
		 					HAL_GPIO_WritePin(highSpeedPin2_GPIO_Port, highSpeedPin2_Pin, GPIO_PIN_RESET);
		 					CurrMotorMode = 0x01;
		 					RPM_th = RPM_NORMAL;
		 					break;
		 		case 0x02 : //Low
		 					//Motor 1
		 					HAL_GPIO_WritePin(lowSpeedPin1_GPIO_Port, lowSpeedPin1_Pin, GPIO_PIN_SET);
		 					HAL_GPIO_WritePin(highSpeedPin1_GPIO_Port, highSpeedPin1_Pin, GPIO_PIN_RESET);
		 					//Motor 2
		 					HAL_GPIO_WritePin(lowSpeedPin2_GPIO_Port, lowSpeedPin2_Pin, GPIO_PIN_SET);
		 					HAL_GPIO_WritePin(highSpeedPin2_GPIO_Port, highSpeedPin2_Pin, GPIO_PIN_RESET);
		 					CurrMotorMode = 0x02;
		 					RPM_th = RPM_ECO;
		 					break;
		 	}
	 }

	 if(SetMotorGear != CurrMotorGear)
	 {
	 		 switch(SetMotorGear)
	 		 	{
	 		 		case 0x08 : //Drive
	 		 					//Motor 1

	 		 					//Motor 2
	 		 					CurrMotorGear = 0x08;

	 		 					break;
	 		 		case 0x02 : //Reverse
	 		 					//Motor 1

	 		 					//Motor 2
	 		 					CurrMotorGear = 0x02;
	 		 					break;
	 		 		case 0x04 : //Park
	 		 					//Motor 1

	 		 					//Motor 2
	 		 					CurrMotorGear = 0x04;
	 		 					break;
	 		 	}
	 }
	memset(transmitBuffer, '\0', sizeof(transmitBuffer));
	osThreadSuspend(parseVotolTaskHandle);
    osDelay(1);
  }
  /* USER CODE END parseSetVotol */
}

/* USER CODE BEGIN Header_ackerman_func */
/**
* @brief Function implementing the ackerman thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ackerman_func */
void ackerman_func(void const * argument)
{
  /* USER CODE BEGIN ackerman_func */
	//float wheels_radius = 0.381/2; //(m)
	float wheels_radius = 0.06/2;//(m)
	//float wheels_radius = 0.3048/2; //(m)
	int vehicle_speedInt = 0;
	float L = 2.3; //(m) vehicle body length
	float d = 1.46; //(m) vehicle body width
  /* Infinite loop */
  for(;;)
  {
	 //timer_val = __HAL_TIM_GET_COUNTER(&htim2);
	  //Timer Start!
	  //timer_val = __HAL_TIM_GET_COUNTER(&htim2);
	  vehicle_speed = (Pedal * RPM_th / 255); //in RPM
	  vehicle_speed = (vehicle_speed * (2*3.14))/60; // in rad/s
	  vehicle_speed = vehicle_speed * wheels_radius; // (m/s)
	  vehicle_speedInt = (int) vehicle_speed;
	  Vehicle_Speed = vehicle_speedInt*3.6;

	  if(vehicle_speedInt == 0)
	  	  {
		  	  angular_speed =0;
	  		  turning_radius=0;
	  		  wheel_speed1 = 0;
	  		  wheel_speed2 = 0;
	  	  }
	  else
	  {
		  if(SteeringWheelInt == 0)
		  	  {
		  		  angular_speed =0;
		  		  turning_radius=0;
		  		  wheel_speed1 = vehicle_speed;
		  		  wheel_speed2 = vehicle_speed;
		  	  }
		  	  else
		  	  {
		  		if(SteeringWheelInt < -45)
		  		{
		  			SteeringWheelInt = -45;
		  		}
		  		else if(SteeringWheelInt > 45)
		  		{
		  			SteeringWheelInt = 45;
		  		}
		  		angular_speed = vehicle_speed * tanf(SteeringWheelInt*3.14/180)/L;
		  		turning_radius = vehicle_speed/angular_speed;

		  		wheel_speed1 = angular_speed * (turning_radius+(d/2));
		  		wheel_speed2 = angular_speed * (turning_radius-(d/2));
		  	  }
	  }

	  angular_speed1 = wheel_speed1 / wheels_radius; //in rad/s
	  angular_speed2 = wheel_speed2 / wheels_radius; //in rad/s

	  targetDAC1 = (int)(angular_speed1 * 60 / (2*3.14)); //in RPM
	  targetDAC2 = (int)(angular_speed2 * 60 / (2*3.14)); //in RPM

	  L_MotorRPM = targetDAC1;
	  R_MotorRPM = targetDAC2;

//	  sprintf(transmitBuffer, "%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %d, %d\n", SteeringWheel, vehicle_speed, L, d, angular_speed, turning_radius, wheel_speed1, wheel_speed2, targetDAC1, targetDAC2);
//	  HAL_UART_Transmit_IT(&huart3, transmitBuffer, 80-1);
//	  HAL_Delay(10);
	  //memset(transmitBuffer, '\0', sizeof(transmitBuffer));
	  //HAL_UART_Transmit_IT(&huart3, "\n", 1);

	  targetDAC1 = (targetDAC1 * 4095 / RPM_th) + 1.35*4095/2.95;
	  targetDAC2 = (targetDAC2 * 4095 / RPM_th) + 1.35*4095/2.95;
	  // Get time elapsed
	  //timer_val = __HAL_TIM_GET_COUNTER(&htim2) - timer_val;
	  osThreadSuspend(ackermanHandle);
    osDelay(1);
  }
  /* USER CODE END ackerman_func */
}

/* USER CODE BEGIN Header_throttleVotolTask */
/**
* @brief Function implementing the throttleVotol thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_throttleVotolTask */
void throttleVotolTask(void const * argument)
{
  /* USER CODE BEGIN throttleVotolTask */
  /* Infinite loop */
  for(;;)
  {
	  if(targetDAC1>setDAC1)
	  {
	  	setDAC1++;
	  	if(setDAC1>4095)
	  	{
	  		setDAC1=4095;
	  	}
	  }
	  else if(targetDAC1<setDAC1)setDAC1--;
	  	if(targetDAC2>setDAC2)
	  	{
	  		setDAC2++;
	  		if(setDAC2>4095)
	  		{
	  			setDAC2=4095;
	  		}
	  	}
	  else if(targetDAC2<setDAC2)setDAC2--;
//	sprintf(transmitBuffer, "%.2f, %.2f, %.2f, %.2f\n", targetDAC1, targetDAC2, setDAC1, setDAC2);
//	HAL_UART_Transmit_IT(&huart3, transmitBuffer, 80-1);
//	HAL_Delay(10);
//	memset(transmitBuffer, '\0', sizeof(transmitBuffer));
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, (setDAC1));
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2,DAC_ALIGN_12B_R, (setDAC2));

	//timer_val = __HAL_TIM_GET_COUNTER(&htim2) - timer_val;
    osDelay(1);
  }
  /* USER CODE END throttleVotolTask */
}

/* USER CODE BEGIN Header_canRcvPXN */
/**
* @brief Function implementing the can_rcvPXNTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_canRcvPXN */
void canRcvPXN(void const * argument)
{
  /* USER CODE BEGIN canRcvPXN */
  osThreadSuspend(parsePXNTaskHandle);
  /* Infinite loop */
  for(;;)
  {
	  if (CAN_State2 == HAL_OK)
	  {
	  		if(RxHeaderPXN.StdId == 0x100)
	  		{
	  			osThreadResume(parsePXNTaskHandle);
	  			HAL_GPIO_TogglePin (LED2_GPIO_Port, LED2_Pin);
	  		}

	  }
    osDelay(1);

  }
  /* USER CODE END canRcvPXN */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
