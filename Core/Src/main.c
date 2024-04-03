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

#include "pca9685.h"
#include <stdint.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/*
 * MiniBot Structures
 */
typedef struct{
	uint8_t  PotNum;
	uint32_t PotMin;
	uint32_t PotMax;
	uint8_t  PotInvertRange;
} MiniBot_Joint_Config_t;

typedef struct{
	GPIO_TypeDef* 	GPIOx;
	uint16_t 		GPIO_Pin;
} MiniBot_Gripper_Config_t;

typedef struct{
	MiniBot_Joint_Config_t  	Base;
	MiniBot_Joint_Config_t  	Shoulder;
	MiniBot_Joint_Config_t  	Elbow;
	MiniBot_Joint_Config_t  	Wrist;
	MiniBot_Gripper_Config_t 	Gripper;
} MiniBot_Config_t;

typedef struct{
	uint32_t BasePotValue;
	uint32_t ShoulderPotValue;
	uint32_t ElbowPotValue;
	uint32_t WristPotValue;
	uint8_t  GripperValue;
} MiniBot_Qdata;

volatile MiniBot_Qdata Qdata;
uint32_t PotRawValue[4];


/*
 * MegaBot Structures
 */
typedef struct{
	uint8_t  ServoNum;
	uint16_t ServoMin;
	uint16_t ServoMax;
	uint16_t ServoHomeAngle;
} MegaBot_Joint_Config_t;

typedef struct{
	uint8_t  ServoNum;
	uint16_t ServoMin;
	uint16_t ServoMax;
	uint16_t ServoHomeAngle;
} MegaBot_Gripper_Config_t;

typedef struct{
	MegaBot_Joint_Config_t  	Base;
	MegaBot_Joint_Config_t  	Shoulder;
	MegaBot_Joint_Config_t  	Elbow;
	MegaBot_Joint_Config_t  	Wrist;
	MegaBot_Gripper_Config_t 	Gripper;
} MegaBot_Config_t;


enum State{
  Init,
  Ready,
  Running,
  EmergencyStop
};

volatile enum State state = Init;
volatile MiniBot_Qdata minibot_data;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId BlinkLEDTaskHandle;
osThreadId MiniBotInputsHandle;
osThreadId ApplicationFSMHandle;
osThreadId EmergencyStopTaHandle;
osThreadId MotorControlTasHandle;
osThreadId LCDPrintTaskHandle;
osMessageQId MiniBotInputQueueHandle;
osMessageQId MotorControlQueueHandle;
osMessageQId LCDPrintQueueHandle;
osMutexId StateMutexHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
void StartDefaultTask(void const * argument);
void BlinkLEDTaskEntry(void const * argument);
void MiniBotInputsEntry(void const * argument);
void ApplicationFSMEntry(void const * argument);
void EmergencyStopTaskEntry(void const * argument);
void MotorControlTaskEntry(void const * argument);
void LCDPrintTaskEntry(void const * argument);

/* USER CODE BEGIN PFP */

void MiniBotInit(MiniBot_Config_t* MiniBot);
void MegaBotInit(MegaBot_Config_t* MegaBot);
void ServoDriverInit(MegaBot_Config_t* MegaBot);

void QPotDataUpdate(uint32_t PotRawValue, volatile MiniBot_Joint_Config_t *Joint, volatile uint32_t *QPotData);

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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of StateMutex */
  osMutexDef(StateMutex);
  StateMutexHandle = osMutexCreate(osMutex(StateMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of MiniBotInputQueue */
  osMessageQDef(MiniBotInputQueue, 16, MiniBot_Qdata);
  MiniBotInputQueueHandle = osMessageCreate(osMessageQ(MiniBotInputQueue), NULL);

  /* definition and creation of MotorControlQueue */
  osMessageQDef(MotorControlQueue, 16, uint16_t);
  MotorControlQueueHandle = osMessageCreate(osMessageQ(MotorControlQueue), NULL);

  /* definition and creation of LCDPrintQueue */
  osMessageQDef(LCDPrintQueue, 16, uint16_t);
  LCDPrintQueueHandle = osMessageCreate(osMessageQ(LCDPrintQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of BlinkLEDTask */
  osThreadDef(BlinkLEDTask, BlinkLEDTaskEntry, osPriorityLow, 0, 128);
  BlinkLEDTaskHandle = osThreadCreate(osThread(BlinkLEDTask), NULL);

  /* definition and creation of MiniBotInputs */
  osThreadDef(MiniBotInputs, MiniBotInputsEntry, osPriorityNormal, 0, 128);
  MiniBotInputsHandle = osThreadCreate(osThread(MiniBotInputs), NULL);

  /* definition and creation of ApplicationFSM */
  osThreadDef(ApplicationFSM, ApplicationFSMEntry, osPriorityNormal, 0, 128);
  ApplicationFSMHandle = osThreadCreate(osThread(ApplicationFSM), NULL);

  /* definition and creation of EmergencyStopTa */
  osThreadDef(EmergencyStopTa, EmergencyStopTaskEntry, osPriorityHigh, 0, 128);
  EmergencyStopTaHandle = osThreadCreate(osThread(EmergencyStopTa), NULL);

  /* definition and creation of MotorControlTas */
  osThreadDef(MotorControlTas, MotorControlTaskEntry, osPriorityNormal, 0, 128);
  MotorControlTasHandle = osThreadCreate(osThread(MotorControlTas), NULL);

  /* definition and creation of LCDPrintTask */
  osThreadDef(LCDPrintTask, LCDPrintTaskEntry, osPriorityNormal, 0, 128);
  LCDPrintTaskHandle = osThreadCreate(osThread(LCDPrintTask), NULL);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 4;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EmergencyStopBtn_Pin */
  GPIO_InitStruct.Pin = EmergencyStopBtn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(EmergencyStopBtn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GripperButton_Pin */
  GPIO_InitStruct.Pin = GripperButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GripperButton_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ResetBtn_Pin */
  GPIO_InitStruct.Pin = ResetBtn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ResetBtn_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void MiniBotInit(MiniBot_Config_t* MiniBot){
	// Configure Each of the joints

	  MiniBot->Base.PotNum = 0;
	  MiniBot->Base.PotMin = 800;
	  MiniBot->Base.PotMax = 3400;
	  MiniBot->Base.PotInvertRange = 0;

	  MiniBot->Shoulder.PotNum = 1;
	  MiniBot->Shoulder.PotMin = 600;
	  MiniBot->Shoulder.PotMax = 3400;
	  MiniBot->Shoulder.PotInvertRange = 0;

	  MiniBot->Elbow.PotNum = 2;
	  MiniBot->Elbow.PotMin = 600;
	  MiniBot->Elbow.PotMax = 3400;
	  MiniBot->Elbow.PotInvertRange = 0;

	  MiniBot->Wrist.PotNum = 3;
	  MiniBot->Wrist.PotMin = 600;
	  MiniBot->Wrist.PotMax = 3200;
	  MiniBot->Wrist.PotInvertRange = 1;

	  MiniBot->Gripper.GPIOx = GPIOA;
	  MiniBot->Gripper.GPIO_Pin = GPIO_PIN_9;
}

void MegaBotInit(MegaBot_Config_t* MegaBot){
  // Configure Each of the joints
  MegaBot->Base.ServoNum = 0;
  MegaBot->Base.ServoMin = 0;
  MegaBot->Base.ServoMax = 180;
  MegaBot->Base.ServoHomeAngle = 90;

  MegaBot->Shoulder.ServoNum = 1;
  MegaBot->Shoulder.ServoMin = 0;
  MegaBot->Shoulder.ServoMax = 180;
  MegaBot->Shoulder.ServoHomeAngle = 0;

  MegaBot->Elbow.ServoNum = 2;
  MegaBot->Elbow.ServoMin = 50;
  MegaBot->Elbow.ServoMax = 180;
  MegaBot->Elbow.ServoHomeAngle = 180;

  MegaBot->Wrist.ServoNum = 3;
  MegaBot->Wrist.ServoMin = 0;
  MegaBot->Wrist.ServoMax = 180;
  MegaBot->Wrist.ServoHomeAngle = 90;

  MegaBot->Gripper.ServoNum = 4;
  MegaBot->Gripper.ServoMin = 0;
  MegaBot->Gripper.ServoMax = 70;
  MegaBot->Gripper.ServoHomeAngle = MegaBot->Gripper.ServoMin;

  ServoDriverInit(&MegaBot);
}

void ServoDriverInit(MegaBot_Config_t* MegaBot){
  PCA9685_Init(&hi2c1);
  PCA9685_SetServoAngle(MegaBot->Base.ServoNum, MegaBot->Base.ServoHomeAngle);
  PCA9685_SetServoAngle(MegaBot->Shoulder.ServoNum, MegaBot->Shoulder.ServoHomeAngle);
  PCA9685_SetServoAngle(MegaBot->Elbow.ServoNum, MegaBot->Elbow.ServoHomeAngle);
  PCA9685_SetServoAngle(MegaBot->Wrist.ServoNum, MegaBot->Wrist.ServoHomeAngle);
  PCA9685_SetServoAngle(MegaBot->Gripper.ServoNum, MegaBot->Gripper.ServoHomeAngle);
}


void QPotDataUpdate(uint32_t PotRawValue, volatile MiniBot_Joint_Config_t *Joint, volatile uint32_t *QPotData){
  if(PotRawValue > *QPotData + 10 || PotRawValue < *QPotData - 10){
    if(PotRawValue > Joint->PotMax){
      *QPotData = Joint->PotMax;
    }else if(PotRawValue < Joint->PotMin){
      *QPotData = Joint->PotMin;
    }else{
      *QPotData = PotRawValue;
    }
    if(Joint->PotInvertRange){
      *QPotData = Joint->PotMax - *QPotData + Joint->PotMin;
    }
  }
}



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
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin, 0);
    osDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_BlinkLEDTaskEntry */
/**
* @brief Function implementing the BlinkLEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BlinkLEDTaskEntry */
void BlinkLEDTaskEntry(void const * argument)
{
  /* USER CODE BEGIN BlinkLEDTaskEntry */
  /* Infinite loop */
  for(;;)
  {
    
    osDelay(500);
    HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin, 1);
    osDelay(500);
  }
  /* USER CODE END BlinkLEDTaskEntry */
}

/* USER CODE BEGIN Header_MiniBotInputsEntry */
/**
* @brief Function implementing the MiniBotInputs thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MiniBotInputsEntry */
void MiniBotInputsEntry(void const * argument)
{
  /* USER CODE BEGIN MiniBotInputsEntry */

	MiniBot_Config_t MiniBot;
	MiniBotInit(&MiniBot);

  
  HAL_ADC_Start_DMA(&hadc1, PotRawValue, 4);

	/* Infinite loop */
	for(;;)
	{
    QPotDataUpdate(PotRawValue[0], &MiniBot.Base, &Qdata.BasePotValue);
    QPotDataUpdate(PotRawValue[1], &MiniBot.Shoulder, &Qdata.ShoulderPotValue);
    QPotDataUpdate(PotRawValue[2], &MiniBot.Elbow, &Qdata.ElbowPotValue);
    QPotDataUpdate(PotRawValue[3], &MiniBot.Wrist, &Qdata.WristPotValue);

    Qdata.GripperValue = (uint8_t)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
    
    xQueueSend(MiniBotInputQueueHandle, (void*)&Qdata, 10);

    osDelay(1);
	}
  /* USER CODE END MiniBotInputsEntry */
}

/* USER CODE BEGIN Header_ApplicationFSMEntry */
/**
* @brief Function implementing the ApplicationFSM thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ApplicationFSMEntry */
void ApplicationFSMEntry(void const * argument)
{
  /* USER CODE BEGIN ApplicationFSMEntry */

  // enum State state = Init;
  // MiniBot_Qdata minibot_data;

  /* Infinite loop */
  for(;;)
  {
    if(state == Init){
      state = Ready;

    }else if(state == Ready){
      xQueueReceive(MiniBotInputQueueHandle, (void*)&minibot_data, 10);
      if(minibot_data.GripperValue == 0){
        xQueueReceive(MiniBotInputQueueHandle, (void*)&minibot_data, 10);
        if(minibot_data.GripperValue == 1){
          state++;
        }
      }

    }else if(state == Running){
      // Send Minibot_data to the Motor Queue for the Motor Task to handle
      xQueueReceive(MiniBotInputQueueHandle, (void*)&minibot_data, 10);
      if(minibot_data.GripperValue == 0){
        xQueueReceive(MiniBotInputQueueHandle, (void*)&minibot_data, 10);
        if(minibot_data.GripperValue == 1){
          state--;
        }
      }

    }
    
    if(state == EmergencyStop){
      // If reset btn is pressed change state to Ready1
    }



    osDelay(1);
  }
  /* USER CODE END ApplicationFSMEntry */
}

/* USER CODE BEGIN Header_EmergencyStopTaskEntry */
/**
* @brief Function implementing the EmergencyStopTa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_EmergencyStopTaskEntry */
void EmergencyStopTaskEntry(void const * argument)
{
  /* USER CODE BEGIN EmergencyStopTaskEntry */
  /* Infinite loop */
  for(;;)
  {
    if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)){
      
    }


    osDelay(100);
  }
  /* USER CODE END EmergencyStopTaskEntry */
}

/* USER CODE BEGIN Header_MotorControlTaskEntry */
/**
* @brief Function implementing the MotorControlTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MotorControlTaskEntry */
void MotorControlTaskEntry(void const * argument)
{
  /* USER CODE BEGIN MotorControlTaskEntry */

  MegaBot_Config_t MegaBot;
  MegaBotInit(&MegaBot);
  ServoDriverInit(&MegaBot);


  /* Infinite loop */
  for(;;)
  {

    


    osDelay(1);
  }
  /* USER CODE END MotorControlTaskEntry */
}

/* USER CODE BEGIN Header_LCDPrintTaskEntry */
/**
* @brief Function implementing the LCDPrintTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LCDPrintTaskEntry */
void LCDPrintTaskEntry(void const * argument)
{
  /* USER CODE BEGIN LCDPrintTaskEntry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END LCDPrintTaskEntry */
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
