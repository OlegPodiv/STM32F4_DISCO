/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "pdm2pcm.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "FreeRTOS_CLI.h"
#include "task.h"
#include "timers.h"
// #include "string.h"
#include "global_var.h"
#include "ds18b20.h"	
#include "user_tasks.h"
#include "main_module.h"
#include "sensors.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MAIL_SIZE (uint32_t) 16 // Длина почтового сообщения
#define QUEUE_SIZE (uint32_t) 16  // Длина очереди
#define STDOUT_QUEUE_SIZE 32 // Длина очереди стандартного вывода
#define STDIN_QUEUE_SIZE 32 // Длина очереди стандартного ввода
#define DISPLAY_OUT_QUEUE_SIZE 32 // Длина очереди стандартного вывода
#define DISPLAY_IN_QUEUE_SIZE 32 // Длина очереди стандартного ввода
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

HCD_HandleTypeDef hhcd_USB_OTG_HS;

osThreadId defaultTaskHandle;
osTimerId DisplayDateTimeTimerHandle;
osStaticTimerDef_t DisplayDateTimeTimerControlBlock;
osTimerId BurnerAugerTimerHandle;
osTimerId DebounceTimerHandle;
osTimerId HopperAugerTimerHandle;
osTimerId AutoCleanTimerHandle;
osTimerId AshRemoveTimerHandle;
osTimerId HotWaterPumpTimerHandle;
osTimerId Aux1TimerHandle;
osTimerId CirclePumpTimerHandle;
osTimerId IgnitionTimerHandle;
osTimerId BurnerFanTimerHandle;
osTimerId SecondaryAirFanTimerHandle;
osTimerId SuctionFanTimerHandle;
osTimerId CleanTimerHandle;
osStaticTimerDef_t CleanTimerControlBlock;
osTimerId AshCleanTimerHandle;
osStaticTimerDef_t AshCleanTimerControlBlock;
/* USER CODE BEGIN PV */
extern osThreadId Ds18b20Handle;
volatile uint8_t NumberUartConsole = 2;
/* ----------------------------------------------------
Данные с 11 каналов ADC1 соответственно:
IN1 - PA1, IN4 - PA4, IN8 - PB0, IN9 - PB1, IN11 - PC1, IN12 - PC2,
IN13 - PC3,IN14 - PC4, IN15 - PC5
9 channel - Temperature sensor MCU
10 channel - Vref int
------------------------------------------------------- */
volatile uint16_t ADC_Data[11]; 
/*
Буфер данных с DS3231 
*/
volatile uint8_t buff_ds3231[8] =  {0,0,0,0,0,0,0,0};
// -----------------------------------------------------------------
volatile uint8_t last_buff_ds3231[8]  =  {0,0,0,0,0,0,0,0}; // последняя, выданная на дисплей дата и время
volatile T_BIN_OUTPUTS binout;
volatile T_PWM_OUTPUTS fanout;
volatile T_SENSORS sensors;
volatile T_BOILER_STATE boiler;
volatile T_BOILER_STATE last_boiler;
volatile T_MENU_TEMPERATURE temperatures;
volatile T_MENU_TIME times;
volatile T_MENU_SEASON season;
volatile T_EVENT on_off;
volatile T_EVENT ext_event;
volatile T_EVENT last_event;
_RTC ds3231_rtc;
uint8_t buff[16] = {0,};
uint8_t buff_display[24] = {0,};
uint8_t string_uart[60];
double cigma = 0;
double cigma2 = 0;
//osMailQId STROUT_Queue;
//osMailQId Display_In_Queue;
osMessageQId USART_Queue; // определяем дескриптор очереди из USART
osMessageQId DISPLAY_Queue; // Определяем дескритор очереди из дисплея
QueueHandle_t stdout_queue; // Определяем дескриптор очереди стандартного вывода
QueueHandle_t stdin_queue; // Определяем дескриптор очереди стандартного ввода
QueueHandle_t display_out_queue; // Определяем дескриптор очереди стандартного вывода
QueueHandle_t display_in_queue; // Определяем дескриптор очереди стандартного ввода
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM12_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_RTC_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_I2C1_Init(void);
static void MX_CRC_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM6_Init(void);
static void MX_I2C2_Init(void);
static void MX_USB_OTG_HS_HCD_Init(void);
static void MX_IWDG_Init(void);
void StartDefaultTask(void const * argument);
void CallbackDisplayDateTimeTimer(void const * argument);
void BurnerAugerCallback(void const * argument);
void CallbackDebounceTimer(void const * argument);
void HopperAugerCallback(void const * argument);
void AutoCleanCallback(void const * argument);
void AshRemoveCallback(void const * argument);
void HotWaterPumpCallback(void const * argument);
void Aux1Callback(void const * argument);
void CirclePumpCallback(void const * argument);
void IgnitionCallback(void const * argument);
void BurnerFanCallback(void const * argument);
void SecondaryAirFanCallback(void const * argument);
void SuctionFanCallback(void const * argument);
void CleanTimerCallback(void const * argument);
void AshCleanCallback(void const * argument);

/* USER CODE BEGIN PFP */
//extern void Task_Ds18b20(void const * argument);
static void vStdoutTask(void *pvParameters);
static void vDisplayOutTask(void *pvParameters);
static void vGetForDisplay(void *pvParameters);
static void vGetForUart(void *pvParameters);
static void vCommandConsoleTask(void *pvParameters);
static void vSensorsPollTask(void *pvParameters);
static void vBoilerFSMTask(void *pvParameters);
extern void GetTime(void *pvParameters);
extern int fputc(int ch, FILE *f);
extern int fgetc(FILE *f);
//void eeprom_test(void);
void RTC_From_DS3231_Init(void);
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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  MX_RTC_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_I2C1_Init();
  MX_CRC_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_I2C2_Init();
  MX_USB_OTG_HS_HCD_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
// ----------------------------------------------------------------
// Инициализация даты и времени RTC из DS3231 
DS3231_Init(&hi2c1);
RTC_From_DS3231_Init();
// Инициализация глобальных параметров
times = menu_time_init(); // Инициализация параметров времени по дефолту с записью в еепром
temperatures = menu_temperature_init(); // Инициализация параметров температур по дефолту с записью в еепром
global_var_init();
// --------------------------------------------------------------
HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
// -----------------------------------------------------
//HAL_ADC_Start_IT(&hadc1);
HAL_ADC_Start_DMA(&hadc1,(uint32_t*) &ADC_Data,11); // Пересылает через DMA - 11 каналов АЦП
// ------------------------------------------------------
boiler_init();
// ------------------------------------------------------
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of DisplayDateTimeTimer */
  osTimerStaticDef(DisplayDateTimeTimer, CallbackDisplayDateTimeTimer, &DisplayDateTimeTimerControlBlock);
  DisplayDateTimeTimerHandle = osTimerCreate(osTimer(DisplayDateTimeTimer), osTimerPeriodic, NULL);

  /* definition and creation of BurnerAugerTimer */
  osTimerDef(BurnerAugerTimer, BurnerAugerCallback);
  BurnerAugerTimerHandle = osTimerCreate(osTimer(BurnerAugerTimer), osTimerOnce, NULL);

  /* definition and creation of DebounceTimer */
  osTimerDef(DebounceTimer, CallbackDebounceTimer);
  DebounceTimerHandle = osTimerCreate(osTimer(DebounceTimer), osTimerOnce, NULL);

  /* definition and creation of HopperAugerTimer */
  osTimerDef(HopperAugerTimer, HopperAugerCallback);
  HopperAugerTimerHandle = osTimerCreate(osTimer(HopperAugerTimer), osTimerOnce, NULL);

  /* definition and creation of AutoCleanTimer */
  osTimerDef(AutoCleanTimer, AutoCleanCallback);
  AutoCleanTimerHandle = osTimerCreate(osTimer(AutoCleanTimer), osTimerOnce, NULL);

  /* definition and creation of AshRemoveTimer */
  osTimerDef(AshRemoveTimer, AshRemoveCallback);
  AshRemoveTimerHandle = osTimerCreate(osTimer(AshRemoveTimer), osTimerOnce, NULL);

  /* definition and creation of HotWaterPumpTimer */
  osTimerDef(HotWaterPumpTimer, HotWaterPumpCallback);
  HotWaterPumpTimerHandle = osTimerCreate(osTimer(HotWaterPumpTimer), osTimerOnce, NULL);

  /* definition and creation of Aux1Timer */
  osTimerDef(Aux1Timer, Aux1Callback);
  Aux1TimerHandle = osTimerCreate(osTimer(Aux1Timer), osTimerOnce, NULL);

  /* definition and creation of CirclePumpTimer */
  osTimerDef(CirclePumpTimer, CirclePumpCallback);
  CirclePumpTimerHandle = osTimerCreate(osTimer(CirclePumpTimer), osTimerOnce, NULL);

  /* definition and creation of IgnitionTimer */
  osTimerDef(IgnitionTimer, IgnitionCallback);
  IgnitionTimerHandle = osTimerCreate(osTimer(IgnitionTimer), osTimerOnce, NULL);

  /* definition and creation of BurnerFanTimer */
  osTimerDef(BurnerFanTimer, BurnerFanCallback);
  BurnerFanTimerHandle = osTimerCreate(osTimer(BurnerFanTimer), osTimerOnce, NULL);

  /* definition and creation of SecondaryAirFanTimer */
  osTimerDef(SecondaryAirFanTimer, SecondaryAirFanCallback);
  SecondaryAirFanTimerHandle = osTimerCreate(osTimer(SecondaryAirFanTimer), osTimerOnce, NULL);

  /* definition and creation of SuctionFanTimer */
  osTimerDef(SuctionFanTimer, SuctionFanCallback);
  SuctionFanTimerHandle = osTimerCreate(osTimer(SuctionFanTimer), osTimerOnce, NULL);

  /* definition and creation of CleanTimer */
  osTimerStaticDef(CleanTimer, CleanTimerCallback, &CleanTimerControlBlock);
  CleanTimerHandle = osTimerCreate(osTimer(CleanTimer), osTimerPeriodic, NULL);

  /* definition and creation of AshCleanTimer */
  osTimerStaticDef(AshCleanTimer, AshCleanCallback, &AshCleanTimerControlBlock);
  AshCleanTimerHandle = osTimerCreate(osTimer(AshCleanTimer), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
osTimerStart(DisplayDateTimeTimerHandle, 5000);
osTimerStart(CleanTimerHandle, (uint32_t)(1000*times.P06));
osTimerStart(AshCleanTimerHandle, (uint32_t)(1000*times.P07));

  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
//osMailQDef(strout_Queue, MAIL_SIZE, uint8_t);
//STROUT_Queue = osMailCreate(osMailQ(strout_Queue), NULL);

//osMailQDef(display_in_Queue, MAIL_SIZE, uint8_t);
//Display_In_Queue = osMailCreate(osMailQ(display_in_Queue), NULL);

osMessageQDef(usart_Queue, QUEUE_SIZE, uint8_t); // определяется очередь сообщений длиной QUEUE_SIZE и типом элемента uint8_t
USART_Queue = osMessageCreate(osMessageQ(usart_Queue), NULL); // создается очередь сообщений с параметрами из usart_Queue и 
																															// и не принадлежащая какому-либо процессу - NULL
osMessageQDef(display_Queue, QUEUE_SIZE, uint8_t);
DISPLAY_Queue = osMessageCreate(osMessageQ(display_Queue), NULL);

stdout_queue = xQueueCreate(STDOUT_QUEUE_SIZE, 1); // создаем очередь стандартного вывода
stdin_queue = xQueueCreate(STDIN_QUEUE_SIZE, 1); // создаем очередь стандартного ввода
display_out_queue = xQueueCreate(DISPLAY_OUT_QUEUE_SIZE, 1); // создаем очередь вывода на дисплей
display_in_queue = xQueueCreate(DISPLAY_IN_QUEUE_SIZE, 1); // создаем очередь ввода с дисплея
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 2048);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
//	osThreadDef(myTask_Ds18b20, Task_Ds18b20, osPriorityNormal, 0, 128);
//  Ds18b20Handle = osThreadCreate(osThread(myTask_Ds18b20), NULL);	
  /* add threads, ... */

//xTaskCreate(vGetForUart, "vGetForUart", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
xTaskCreate(vGetForDisplay, "vGetForDisplay", configMINIMAL_STACK_SIZE, NULL, 3, NULL); 
xTaskCreate(vGetTime, "vGetTime", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
//xTaskCreate(vStdoutTask, "vStdoutTask", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
//xTaskCreate(vDisplayOutTask, "vDisplayOutTask", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
xTaskCreate(vCommandConsoleTask, "vCommandConsoleTask", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
xTaskCreate(vSensorsPollTask, "vSensorsPollTask", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
xTaskCreate(vBoilerFSMTask, "vBoilerFSMTask", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  while (1)
//  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//  }

return 0;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_BYPASS;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  hadc1.Init.NbrOfConversion = 11;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = 11;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
    
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_SET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_THURSDAY;
  sDate.Month = RTC_MONTH_NOVEMBER;
  sDate.Date = 14;
  sDate.Year = 19;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 8191;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 84-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 8191;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

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

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 0;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 0;
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
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  huart2.Init.BaudRate = 57600;
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 57600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief USB_OTG_HS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_HS_HCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_HS_Init 0 */

  /* USER CODE END USB_OTG_HS_Init 0 */

  /* USER CODE BEGIN USB_OTG_HS_Init 1 */

  /* USER CODE END USB_OTG_HS_Init 1 */
  hhcd_USB_OTG_HS.Instance = USB_OTG_HS;
  hhcd_USB_OTG_HS.Init.Host_channels = 12;
  hhcd_USB_OTG_HS.Init.speed = HCD_SPEED_FULL;
  hhcd_USB_OTG_HS.Init.dma_enable = DISABLE;
  hhcd_USB_OTG_HS.Init.phy_itface = USB_OTG_EMBEDDED_PHY;
  hhcd_USB_OTG_HS.Init.Sof_enable = DISABLE;
  hhcd_USB_OTG_HS.Init.low_power_enable = DISABLE;
  hhcd_USB_OTG_HS.Init.vbus_sensing_enable = DISABLE;
  hhcd_USB_OTG_HS.Init.use_external_vbus = DISABLE;
  if (HAL_HCD_Init(&hhcd_USB_OTG_HS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_HS_Init 2 */

  /* USER CODE END USB_OTG_HS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|Relay_9_Pin|Relay_0_Pin|Relay_1_Pin
                          |Relay_2_Pin|Relay_3_Pin|Relay_4_Pin|Relay_5_Pin
                          |Relay_6_Pin|Relay_7_Pin|Relay_8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_ON_GPIO_Port, LED_ON_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DS18B20_Pin|DS18B20_Vcc_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DS3231_SQW_Pin */
  GPIO_InitStruct.Pin = DS3231_SQW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DS3231_SQW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin Relay_9_Pin Relay_0_Pin Relay_1_Pin
                           Relay_2_Pin Relay_3_Pin Relay_4_Pin Relay_5_Pin
                           Relay_6_Pin Relay_7_Pin Relay_8_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|Relay_9_Pin|Relay_0_Pin|Relay_1_Pin
                          |Relay_2_Pin|Relay_3_Pin|Relay_4_Pin|Relay_5_Pin
                          |Relay_6_Pin|Relay_7_Pin|Relay_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : D1_Pin D2_Pin */
  GPIO_InitStruct.Pin = D1_Pin|D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_ON_Pin OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = LED_ON_Pin|OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ON_Pin */
  GPIO_InitStruct.Pin = ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DS18B20_Pin DS18B20_Vcc_Pin */
  GPIO_InitStruct.Pin = DS18B20_Pin|DS18B20_Vcc_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : D4_Pin D8_Pin */
  GPIO_InitStruct.Pin = D4_Pin|D8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : A1_Pin */
  GPIO_InitStruct.Pin = A1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(A1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}

/* USER CODE BEGIN 4 */
//#define USE_FREERTOS
// -------------------------------------------------------------------------------------
// Переопределение функции fputc для возможности работы с printf
// -------------------------------------------------------------------------------------
int fputc(int ch, FILE *f){

	char item = (char) ch;
#ifdef USE_FREERTOS	
	
	switch (NumberUartConsole)
					{
		case 2:	{
						xQueueSendToBack(stdout_queue, &item, 0);
						break;	
						}	
		case 6: {
						xQueueSendToBack(display_out_queue, &item, 0);
						break;
						}
					}
#else	
	switch (NumberUartConsole)
						{
					case 2:{HAL_UART_Transmit(&huart2, (uint8_t *)&item, 1, 0x1000); // по умолчанию 0xFFFF
									break;}	
					case 6:{HAL_UART_Transmit(&huart6, (uint8_t *)&item, 1, 0x1000); 
									break;}
						default: break;
						}	
  #endif
  return ch;}
// ---------------------------------------------------------------------------------------
// Переопределение функции fgetc для возможности работы с scanf
// ---------------------------------------------------------------------------------------
int fgetc(FILE *f){
	
//	uint8_t ch;
//#ifdef USE_FREERTOS
//    return (xQueueReceive(stdin_queue, &ch, portMAX_DELAY) == pdPASS) ? (int)ch : (int)'\0';
//#else
//	switch (NumberUartConsole)
//						{
//				case 2:{(void)HAL_UART_Receive_DMA(&huart2, (uint8_t *)&ch, 1);
//							break;}
//				case 6:{HAL_UART_Receive_IT(&huart6, (uint8_t *)&ch, 1);
//							break;}
//						}
//	return ((int)ch);
//#endif						
}
// -----------------------------------------------------------------------------------
//int ferror(FILE *f){return EOF;}	
// -----------------------------------------------------------------------------------
void vStdoutTask(void *pvParameters)
{
static uint8_t buf[STDOUT_QUEUE_SIZE];
uint16_t length = 0;
  for(;;)
  {
		    if (xQueueReceive(stdout_queue, &buf[length], portMAX_DELAY) == pdPASS)
    {
        while((++length < STDOUT_QUEUE_SIZE) && (xQueueReceive(stdout_queue, &buf[length], 0) == pdPASS)) {};
        (void)HAL_UART_Transmit_IT(&huart2, buf, length);
        while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY && HAL_UART_GetState(&huart2) != HAL_UART_STATE_BUSY_RX)
        {
            vTaskDelay(10); // Задержка работы задачи 10 мсек
        }
    }
	}
    __ASM volatile("BKPT #01");
//    vTaskDelete(NULL);
}
// --------------------------------------------------------------------------------------
void vDisplayOutTask(void *pvParameters)
{
static uint8_t buf[DISPLAY_OUT_QUEUE_SIZE];
uint16_t length = 0;   
	for(;;)
   {
		if (xQueueReceive(display_out_queue, &buf[length], portMAX_DELAY) == pdPASS)
    {
        while((++length < DISPLAY_OUT_QUEUE_SIZE) && (xQueueReceive(display_out_queue, &buf[length], 0) == pdPASS)) {};
        (void)HAL_UART_Transmit_IT(&huart6, buf, length);
        while (HAL_UART_GetState(&huart6) != HAL_UART_STATE_READY && HAL_UART_GetState(&huart6) != HAL_UART_STATE_BUSY_RX)
        {
            vTaskDelay(10); // Задержка работы задачи 10 мсек
        }
    }
	}	
    __ASM volatile("BKPT #01");
//    vTaskDelete(NULL);
}
// ---------------------------------------------------------------------------------------------------
void vGetForUart(void *pvParameters)
{	
		osEvent event;

	while(1){
//osDelay(100);
NumberUartConsole=2;
//scanf("%c",&buff[0]);
HAL_UART_Receive_IT(&huart2, (uint8_t*)buff, 1);	
event = osMessageGet(USART_Queue, 100);
//event = osMessageGet(stdin_queue, 100);		
		if (event.status == osEventMessage)
		{
//		HAL_UART_Transmit_IT(&huart2,(uint8_t*)buff,1);
		printf("%c",buff[0]);	
		} 
//		ParseCommandForUART((uint8_t*)buff, 2);	
//		ParseString((uint8_t*)buff, 16);
	}
}	
// --------------------------------------------------------------------------------------
void vGetForDisplay(void *pvParameters)
{	
		osEvent event;

	while(1){
HAL_UART_Receive_IT(&huart6, (uint8_t*)buff_display, 24);	
event = osMessageGet(DISPLAY_Queue, 100);
if (event.status == osEventMessage){
//									HAL_UART_Transmit(&huart2,(uint8_t*)buff_display,16,0x1000);	
									ParseCommandForDisplay((uint8_t*)buff_display, 24);
										}
	}
}	
// ----------------------------------------------------------------------------------------
void vSensorsPollTask(void *pvParameters)
{
while(1)
	{
	sensors =  sensors_poll();
	vTaskDelay(200);	
	}
}	
// ----------------------------------------------------------------------------------------
void vBoilerFSMTask(void *pvParameters)
{
while(1) 
	{
	boiler_step();
			NumberUartConsole = 6;
			printf("st.val=%u%c%c%c", boiler,0xFF,0xFF,0xFF); 
	if (TIM4->CCR4 == 0x0) TIM4->CCR4 = 0xFF; // тестовое мигание для проверки отсутствия зависания
	else TIM4->CCR4 = 0x0;	
NumberUartConsole=2;
printf("D10= %5.2f ", sensors.D10);
cigma += sensors.D10; 
cigma2++;
printf("Average= %5.2f \r\n", cigma/cigma2);	
		HAL_IWDG_Refresh(&hiwdg); // Перезагрузка watchdog таймера	
		vTaskDelay(1000);
	}
	
}	
// ----------------------------------------------------------------------------------------
// Задача интерпретатора командной строки
// ----------------------------------------------------------------------------------------
/* Dimensions the buffer into which input characters are placed. */
#define cmdMAX_INPUT_SIZE   60
/* Dimensions the buffer into which string outputs can be placed. */
#define cmdMAX_OUTPUT_SIZE  1024
void vCommandConsoleTask( void *pvParameters )
{
    char cRxedChar = 0;
    uint8_t cInputIndex = 0;
    BaseType_t xMoreDataToFollow = 0;

    static char pcOutputString[ cmdMAX_OUTPUT_SIZE ]; 
    static char pcInputString[ cmdMAX_INPUT_SIZE ];

    for( ;; )
    {
//			scanf("%c", &cRxedChar);
//		cRxedChar = (char)getchar();
HAL_UART_Receive_IT(&huart2, (uint8_t*)&cRxedChar, 1);	
        if( cRxedChar == '\r' )
        {
            printf("\r\n");
            do
            {
                xMoreDataToFollow = FreeRTOS_CLIProcessCommand( pcInputString, pcOutputString, cmdMAX_OUTPUT_SIZE );
                printf("%s", pcOutputString);
                memset( pcOutputString, 0x00, cmdMAX_OUTPUT_SIZE );

            } 
            while( xMoreDataToFollow != pdFALSE );

            cInputIndex = 0;
            memset( pcInputString, 0x00, cmdMAX_INPUT_SIZE );
        }
        else
        {
            if( cRxedChar == '\b' )
            {
                if( cInputIndex > 0 )
                {
                    cInputIndex--;
                    pcInputString[ cInputIndex ] = '\0';
                }
            }
            else
            {
                if( cInputIndex < cmdMAX_INPUT_SIZE )
                {
                    pcInputString[ cInputIndex ] = cRxedChar;
                    cInputIndex++;
                }
            }
        }
    }
}
// ---------------------------------------------------------------------------------------
// Функция инициализация даты и времени RTC из DS3231 
// ----------------------------------------------------------------------------------------
void RTC_From_DS3231_Init(void)
{
	RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

// ----------------------------------------------------------------	
if (DS3231_GetTime(&ds3231_rtc))
{
	sTime.Hours = ds3231_rtc.Hour;
  sTime.Minutes = ds3231_rtc.Min;
  sTime.Seconds = ds3231_rtc.Sec;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_SET;
	
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = ds3231_rtc.DaysOfWeek;
  sDate.Month = ds3231_rtc.Month;
  sDate.Date = ds3231_rtc.Date;
  sDate.Year = ds3231_rtc.Year;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
}
// --------- Конец функции инициализации даты и времени в RTC из DS3231 -------------------	
	
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint8_t ch;
	if(huart==&huart2)
  {
		ch = huart->Instance->DR;
//		osMessagePut(USART_Queue, buff[0], 100);
		xQueueSendToBack(stdin_queue, &ch, 100); 		
	}
	
	if(huart==&huart6)
  {
		osMessagePut(DISPLAY_Queue, buff_display[0], 100);  
	}		
}	
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc1)
{
	// ADC_Data[0] = HAL_ADC_GetValue(hadc1);
}
// -----------------------------------------------------------------------------
// Обработка колбэка внешнего прерывания
// -----------------------------------------------------------------------------
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
if (GPIO_Pin == ON_Pin)
	{
	on_key_event();
//		if (xTimerIsTimerActive(DebounceTimerHandle) == pdFALSE) 
//		{
//		on_key_event();
//		xTimerStart(DebounceTimerHandle, 100);
//		}
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

  /* init code for FATFS */
  MX_FATFS_Init();

  /* init code for USB_HOST */
  MX_USB_HOST_Init();

  /* init code for PDM2PCM */
  MX_PDM2PCM_Init();

  /* USER CODE BEGIN 5 */
//RTC_TimeTypeDef sTime = {0};
//RTC_DateTypeDef sDate = {0};
  /* Infinite loop */
  for(;;)
  {
		vTaskDelay(1000); //
		NumberUartConsole = 2;
		
// eeprom_test();	
//T_MENU_TEMPERATURE t_eeprom;
//T_MENU_TIME mt_eeprom;
//menu_temperature_save_eeprom(temperatures);
//t_eeprom = menu_temperature_load_eeprom();
//menu_time_save_eeprom(times);
//mt_eeprom = menu_time_load_eeprom();
//eeprom_comparing(t_eeprom, mt_eeprom, temperatures, times);						
}
  /* USER CODE END 5 */
}

/* CallbackDisplayDateTimeTimer function */
void CallbackDisplayDateTimeTimer(void const * argument)
{
  /* USER CODE BEGIN CallbackDisplayDateTimeTimer */
//	uint8_t buf[20];
//		sprintf((char *)buf, "time.sec.val=%u%c%c%c", buff_ds3231[7],0xFF,0xFF,0xFF);
//		HAL_UART_Transmit_IT(&huart2, (uint8_t *)&buf, 20);
//		//--------------------------------------------------------------------
//		sprintf((char *)buf,"time.min.val=%u%c%c%c", buff_ds3231[6],0xFF,0xFF,0xFF);
//		HAL_UART_Transmit_IT(&huart2, (uint8_t *)&buf, 20);
//		//--------------------------------------------------------------------
//		sprintf((char *)buf,"time.hour.val=%u%c%c%c", buff_ds3231[5],0xFF,0xFF,0xFF);
//		HAL_UART_Transmit_IT(&huart2, (uint8_t *)&buf, 20);
//		//--------------------------------------------------------------------
//		sprintf((char *)buf,"time.day.val=%u%c%c%c", buff_ds3231[4],0xFF,0xFF,0xFF);
//		HAL_UART_Transmit_IT(&huart2, (uint8_t *)&buf, 20);
// HAL_GPIO_TogglePin(GPIOE, Relay_1_Pin);
	
NumberUartConsole=6;
  	printf("time.tm0.en=%u%c%c%c", 0,0xFF,0xFF,0xFF);
NumberUartConsole=6;	
		printf("time.sec.val=%u%c%c%c", buff_ds3231[7],0xFF,0xFF,0xFF);
NumberUartConsole=6;	
		printf("time.tm0.en=%u%c%c%c", 1,0xFF,0xFF,0xFF);
NumberUartConsole=6;		
		printf("time.sec.val=%u%c%c%c", buff_ds3231[7],0xFF,0xFF,0xFF);
NumberUartConsole=6;		
if (last_buff_ds3231[6] != buff_ds3231[6])	printf("time.min.val=%u%c%c%c", buff_ds3231[6],0xFF,0xFF,0xFF);
NumberUartConsole=6;
if (last_buff_ds3231[5] != buff_ds3231[5])	printf("time.hour.val=%u%c%c%c", buff_ds3231[5],0xFF,0xFF,0xFF);
NumberUartConsole=6;
if (last_buff_ds3231[1] != buff_ds3231[1])	printf("time.day.val=%u%c%c%c", buff_ds3231[1],0xFF,0xFF,0xFF);
NumberUartConsole=6;
if (last_buff_ds3231[3] != buff_ds3231[3])	printf("time.month.val=%u%c%c%c", buff_ds3231[3],0xFF,0xFF,0xFF);
NumberUartConsole=6;
if (last_buff_ds3231[2] != buff_ds3231[2])	printf("time.year.val=%u%c%c%c", buff_ds3231[2],0xFF,0xFF,0xFF); 		

for (int i = 0; i<8; i++)	last_buff_ds3231[i] = buff_ds3231[i];

sensors = sensors_poll();
double sens = 1.27*sensors.D3; 
NumberUartConsole=6;
if (sens>=37) printf("sensors.z0.val=%u%c%c%c", (int)(sens - 37),0xFF,0xFF,0xFF);
else printf("sensors.z0.val=%u%c%c%c", (int)(sens + 323),0xFF,0xFF,0xFF);

sens = 1.27*sensors.D5;
NumberUartConsole=6;
if (sens>=37) printf("sensors.z1.val=%u%c%c%c", (int)(sens - 37),0xFF,0xFF,0xFF);
else printf("sensors.z1.val=%u%c%c%c", (int)(sens + 323),0xFF,0xFF,0xFF);

sens = 1.27*sensors.D6;
NumberUartConsole=6;
if (sens>=37) printf("sensors.z2.val=%u%c%c%c", (int)(sens - 37),0xFF,0xFF,0xFF);
else printf("sensors.z2.val=%u%c%c%c", (int)(sens + 323),0xFF,0xFF,0xFF);

sens = 1.27*sensors.D7;
NumberUartConsole=6;
if (sens>=37) printf("sensors.z3.val=%u%c%c%c", (int)(sens - 37),0xFF,0xFF,0xFF);
else printf("sensors.z3.val=%u%c%c%c", (int)(sens + 323),0xFF,0xFF,0xFF);

//sens = 1.27*sensors.D10; 
//if (sens>=37) printf("sensors.z3.val=%u%c%c%c", (int)(sens - 37),0xFF,0xFF,0xFF);
//else printf("sensors.z3.val=%u%c%c%c", (int)(sens + 323),0xFF,0xFF,0xFF);
NumberUartConsole=6;
printf("sensors.D1V.val=%u%c%c%c", sensors.D1,0xFF,0xFF,0xFF);
NumberUartConsole=6;
printf("sensors.D2V.val=%u%c%c%c", sensors.D2,0xFF,0xFF,0xFF);
NumberUartConsole=6;
printf("sensors.D4V.val=%u%c%c%c", sensors.D4,0xFF,0xFF,0xFF);
NumberUartConsole=6;
printf("sensors.D8V.val=%u%c%c%c", sensors.D8,0xFF,0xFF,0xFF);
NumberUartConsole=6;
printf("sensors.A1V.val=%u%c%c%c", sensors.A1,0xFF,0xFF,0xFF); 

NumberUartConsole=2;

//printf("D10= %5.2f ", sensors.D10);
//cigma += sensors.D10; 
//cigma2++;
//printf("Average= %5.2f \r\n", cigma/cigma2);
//printf("D10= %u ", ADC_Data[4]);
//printf("Average= %5.2f \r\n", cigma/cigma2);
//printf("st.val=%u%c%c%c", boiler,0xFF,0xFF,0xFF); 

  /* USER CODE END CallbackDisplayDateTimeTimer */
}

/* BurnerAugerCallback function */
void BurnerAugerCallback(void const * argument)
{
  /* USER CODE BEGIN BurnerAugerCallback */
if (binout.BurnerAuger.enable != 0)	HAL_GPIO_WritePin(binout.BurnerAuger.GPIOx, binout.BurnerAuger.GPIO_Pin, GPIO_PIN_RESET);	
  /* USER CODE END BurnerAugerCallback */
}

/* CallbackDebounceTimer function */
void CallbackDebounceTimer(void const * argument)
{
  /* USER CODE BEGIN CallbackDebounceTimer */
  
  /* USER CODE END CallbackDebounceTimer */
}

/* HopperAugerCallback function */
void HopperAugerCallback(void const * argument)
{
  /* USER CODE BEGIN HopperAugerCallback */
if (binout.HopperAuger.enable != 0) HAL_GPIO_WritePin(binout.HopperAuger.GPIOx, binout.HopperAuger.GPIO_Pin, GPIO_PIN_RESET);  
  /* USER CODE END HopperAugerCallback */
}

/* AutoCleanCallback function */
void AutoCleanCallback(void const * argument)
{
  /* USER CODE BEGIN AutoCleanCallback */
if (binout.AutoClean.enable != 0)	HAL_GPIO_WritePin(binout.AutoClean.GPIOx, binout.AutoClean.GPIO_Pin, GPIO_PIN_RESET);  
  /* USER CODE END AutoCleanCallback */
}

/* AshRemoveCallback function */
void AshRemoveCallback(void const * argument)
{
  /* USER CODE BEGIN AshRemoveCallback */
 if (binout.AshRemove.enable != 0)	HAL_GPIO_WritePin(binout.AshRemove.GPIOx, binout.AshRemove.GPIO_Pin, GPIO_PIN_RESET);		 
  /* USER CODE END AshRemoveCallback */
}

/* HotWaterPumpCallback function */
void HotWaterPumpCallback(void const * argument)
{
  /* USER CODE BEGIN HotWaterPumpCallback */
if (binout.HotWaterPump.enable != 0) HAL_GPIO_WritePin(binout.HotWaterPump.GPIOx, binout.HotWaterPump.GPIO_Pin, GPIO_PIN_RESET);  
  /* USER CODE END HotWaterPumpCallback */
}

/* Aux1Callback function */
void Aux1Callback(void const * argument)
{
  /* USER CODE BEGIN Aux1Callback */
if (binout.Aux1.enable != 0) HAL_GPIO_WritePin(binout.Aux1.GPIOx, binout.Aux1.GPIO_Pin, GPIO_PIN_RESET);			  
  /* USER CODE END Aux1Callback */
}

/* CirclePumpCallback function */
void CirclePumpCallback(void const * argument)
{
  /* USER CODE BEGIN CirclePumpCallback */
 if (binout.CirclePump.enable != 0) HAL_GPIO_WritePin(binout.CirclePump.GPIOx, binout.CirclePump.GPIO_Pin, GPIO_PIN_RESET); 
  /* USER CODE END CirclePumpCallback */
}

/* IgnitionCallback function */
void IgnitionCallback(void const * argument)
{
  /* USER CODE BEGIN IgnitionCallback */
if (binout.Ignition.enable != 0 )	HAL_GPIO_WritePin(binout.Ignition.GPIOx, binout.Ignition.GPIO_Pin, GPIO_PIN_RESET);  
  /* USER CODE END IgnitionCallback */
}

/* BurnerFanCallback function */
void BurnerFanCallback(void const * argument)
{
  /* USER CODE BEGIN BurnerFanCallback */
  TIM4->CCR1= 0x0;
  /* USER CODE END BurnerFanCallback */
}

/* SecondaryAirFanCallback function */
void SecondaryAirFanCallback(void const * argument)
{
  /* USER CODE BEGIN SecondaryAirFanCallback */
  TIM4->CCR2= 0x0;
  /* USER CODE END SecondaryAirFanCallback */
}

/* SuctionFanCallback function */
void SuctionFanCallback(void const * argument)
{
  /* USER CODE BEGIN SuctionFanCallback */
  TIM4->CCR3= 0x0;
  /* USER CODE END SuctionFanCallback */
}

/* CleanTimerCallback function */
void CleanTimerCallback(void const * argument)
{
  /* USER CODE BEGIN CleanTimerCallback */
  if (binout.AutoClean.enable != 0)	{HAL_GPIO_WritePin(binout.AutoClean.GPIOx, binout.AutoClean.GPIO_Pin, GPIO_PIN_SET);
	osTimerStart(AutoCleanTimerHandle, (uint32_t)(1000*times.T17));}

  /* USER CODE END CleanTimerCallback */
}

/* AshCleanCallback function */
void AshCleanCallback(void const * argument)
{
  /* USER CODE BEGIN AshCleanCallback */
	if (binout.AshRemove.enable != 0)	{HAL_GPIO_WritePin(binout.AshRemove.GPIOx, binout.AshRemove.GPIO_Pin, GPIO_PIN_SET);
	osTimerStart(AshRemoveTimerHandle, (uint32_t)(1000*times.T18));}	  
  /* USER CODE END AshCleanCallback */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
