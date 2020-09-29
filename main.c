
/*
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
RTC_HandleTypeDef hrtc;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/*I2C Addresses*/
#define A_DEVICE_ADDRESS   0x19
#define ST_1_DEVICE_ADDRESS  0x48
#define TPH_2_DEVICE_ADDRESS 0x76
#define TPH_1_DEVICE_ADDRESS 0x77
#define ST_1_DEVICE_ADDRESS  0x48
#define ST_2_DEVICE_ADDRESS  0x4D
#define RT_1_DEVICE_ADDRESS  0x1B
#define RT_2_DEVICE_ADDRESS  0x5A

/*Types of I2C interrupts*/
#define ACCEL_RX_CPLT 1

/*Limit at which the transition between battery and full run mode occurs. Units: RPM*100*/
#define BATT_FR_TRANSITION 1000

/*Counter length definitions. FR Counter counts in 120ths of a second.*/
#define COUNT_1_FRMIN 7199
#define COUNT_FR10TH_SEC 11
#define COUNT_10_FRSECS 1199
#define COUNT_1_FRSEC 119

/*Counter length definitions. BATT Counter counts at 1Hz.*/
#define COUNT_15_BATTMINS 899
#define COUNT_1_BATTMIN 59
#define COUNT_15_BATTSECS 14
#define COUNT_2_BATTSECS 1

/*Buffer Sizes*/
#define A_BUFFER_SIZE 100
#define UART_TX_BUFFER_LENGTH 280
#define RS_BUFFER_SIZE 100
#define TPH_BUFFER_SIZE 100
#define ST_BUFFER_SIZE 100
#define RT_BUFFER_SIZE 100
#define WD_BUFFER_SIZE 100
#define WS_BUFFER_SIZE 100

/*Definition used in exponential moving average (dsp_ema_i32) function.*/
#define DSP_EMA_I32_ALPHA(x) ( (uint16_t)(x * 65535) )

/*Packet type designators*/
uint8_t FR_Env1_Type =  0b00100000;
uint8_t FR_Env2_Type =  0b01000000;
uint8_t FR_Wind_Type =  0b10000000;
uint8_t Motion_Type = 0b11000000;
uint8_t BATT_Wind_Type = 0b01100000;
uint8_t BATT_Env1_Type = 0b11100000;
uint8_t BATT_Env2_Type = 0b10100000;

/*State variable used to define whether the device is in Full Run (FR) or Battery (BATT) mode*/
uint8_t Running_Mode;

/*Fifo Structure Definitions*/
struct Flash_Fifo{
	uint32_t head;
	uint32_t tail;
};

struct A_Fifo{
	uint16_t buffer[A_BUFFER_SIZE];
	uint16_t head;
	uint16_t tail;
	int16_t output;
	uint8_t fullifone;
	uint8_t emptyifone;
};

struct RS_Fifo{
	uint16_t buffer[RS_BUFFER_SIZE];
	uint16_t head;
	uint16_t tail;
	int16_t output;
	uint8_t fullifone;
	uint8_t emptyifone;
};

struct TPH_Fifo{
	int32_t buffer[TPH_BUFFER_SIZE];
	uint16_t head;
	uint16_t tail;
	int32_t output;
	uint8_t fullifone;
	uint8_t emptyifone;
};

struct ST_Fifo{
	int8_t buffer[ST_BUFFER_SIZE];
	uint16_t head;
	uint16_t tail;
	int8_t output;
	uint8_t fullifone;
	uint8_t emptyifone;
};

struct RT_Fifo{
	uint16_t buffer[RT_BUFFER_SIZE];
	uint16_t head;
	uint16_t tail;
	int16_t output;
	uint8_t fullifone
	uint8_t emptyifone;
};

struct WD_Fifo{
	uint8_t buffer[WD_BUFFER_SIZE];
	uint16_t head;
	uint16_t tail;
	int8_t output;
	uint8_t fullifone;
	uint8_t emptyifone;
};

struct TimeStamp {
	uint16_t Tenths_After_Hour;
	uint16_t Hours_After_New_Year;
	};

/*Variables for Time Stamp*/
struct TimeStamp TS = {0};

/*Variables for packetizing functions*/
uint8_t Packet_Buffer[14];

/*Variables for flash management*/
struct Flash_Fifo FF = {0};

/*Tracks which I2C device is active for i2c received data interrupt*/
uint8_t RxCpltType = 0;

/*Variables for UART communication*/
uint16_t UART_bytes_in_send = 0;
uint8_t UART_rxBuffer[1];
uint8_t UART_Tx_Buffer[UART_TX_BUFFER_LENGTH];
uint8_t UART_Tx_Backup[UART_TX_BUFFER_LENGTH];

/*Buffer for incoming i2c data*/
uint8_t i2c_storage_buffer[6];

/*Flag variables for sleep modes*/
uint8_t RTC_ITF = 0;

/*Counter variables*/
uint16_t cnt_FR10thsec = 0;
uint16_t cnt_FR1sec = 0;
uint16_t cnt_FR10secs = 0;
uint16_t cnt_FR1min = 0;
uint16_t cnt_BATT15mins = 0;
uint16_t cnt_BATT1min = 0;
uint16_t cnt_BATT15secs = 0;
uint16_t cnt_BATT2secs = 0;
uint8_t sequence_counter = 0;

/*Variables for Accel*/
struct A_Fifo AX = {0};
struct A_Fifo AY = {0};
struct A_Fifo AZ = {0};
uint16_t AX_to_save;
uint16_t AY_to_save;
uint16_t AZ_to_save;
uint16_t AX_average;
uint16_t AY_average;
uint16_t AZ_average;

/*Variables for Stator Temp*/
uint16_t ST_1_to_save = 0;
uint16_t ST_2_to_save = 0;
uint16_t ST_1_average = 0;
uint16_t ST_2_average = 0;
uint32_t ST_1_sum_average = 0;
uint32_t ST_2_sum_average = 0;
struct ST_Fifo ST1 = {0};
struct ST_Fifo ST2 = {0};

/*Variables for Rotor Temp*/
uint16_t RT_to_save = 0;
uint16_t RT_average = 0;
uint16_t RT_sum_average = 0;
struct RT_Fifo RT = {0};

/*Variables for Wind Direction -- Change WD_to_save to uint8 in future*/
uint16_t WD_to_save = 0;
uint16_t WD_to_save_1 = 0;
uint16_t WD_to_save_2 = 0;
uint16_t WD_to_save_3 = 0;
uint16_t WD_to_save_4 = 0;
struct WD_Fifo WD = {0};

/*Variables for Wind Speed*/
uint16_t WS_to_save;
uint16_t WS_max_to_save;
uint16_t WS_avg_to_save;
uint32_t WS_sum_to_avg;
uint8_t WS_count_to_avg;
uint16_t WS_storage;
uint16_t WS_minimum;
uint16_t WS_Time_Elapsed;
struct RS_Fifo WS = {0};
uint16_t WS_DebounceLimit = 20;

/*TPH Variable - Shared*/
uint8_t ph_storage[1];

/*TPH Variables - Device 1*/
uint16_t dig_T1_1;
uint16_t dig_T2_1;
int16_t dig_T3_1;
uint16_t dig_P1_1;
int16_t dig_P2_1;
int16_t dig_P3_1;
int16_t dig_P4_1;
int16_t dig_P5_1;
int16_t dig_P6_1;
int16_t dig_P7_1;
int16_t dig_P8_1;
int16_t dig_P9_1;
char dig_H1_1;
int16_t dig_H2_1;
char dig_H3_1;
int16_t dig_H4_1;
int16_t dig_H5_1;
char dig_H6_1;
int32_t PH_T_1;
int32_t PH_P_1;
int32_t PH_H_1;
int32_t var1_1;
int32_t t_fine_1;
struct TPH_Fifo T1 = {0};
struct TPH_Fifo P1 = {0};
struct TPH_Fifo H1 = {0};
uint16_t T1_to_save;
uint32_t T1_sum_average = 0;
int32_t T1_average = 0;
uint16_t P1_to_save;
uint32_t P1_sum_average = 0;
int32_t P1_average = 0;
uint16_t H1_to_save;
uint32_t H1_sum_average = 0;
int32_t H1_average = 0;

/*TPH Variables - Device 2*/
uint16_t dig_T1_2;
int16_t dig_T2_2;
int16_t dig_T3_2;
uint16_t dig_P1_2;
int16_t dig_P2_2;
int16_t dig_P3_2;
int16_t dig_P4_2;
int16_t dig_P5_2;
int16_t dig_P6_2;
int16_t dig_P7_2;
int16_t dig_P8_2;
int16_t dig_P9_2;
char dig_H1_2;
int16_t dig_H2_2;
char dig_H3_2;
int16_t dig_H4_2;
int16_t dig_H5_2;
char dig_H6_2;
int32_t PH_T_2;
int32_t PH_P_2;
int32_t PH_H_2;
int32_t var1_2;
int32_t t_fine_2;
struct TPH_Fifo T2 = {0};
struct TPH_Fifo P2 = {0};
struct TPH_Fifo H2 = {0};
uint16_t T2_to_save;
uint32_t T2_sum_average = 0;
int32_t T2_average = 0;
uint16_t P2_to_save;
uint32_t P2_sum_average = 0;
int32_t P2_average = 0;
uint16_t H2_to_save;
uint32_t H2_sum_average = 0;
int32_t H2_average = 0;

/*Variables for Rotor Speed*/
uint8_t RS_Instance_Switch = 0;
uint8_t stayawake_RS = 0;
uint16_t RS_max_to_save;
uint16_t RS_storage;
uint16_t RS_minimum;
uint16_t RS_Time_Elapsed;
uint16_t RS_to_save;
struct RS_Fifo RS = {0};

/*Flash Fifo addresses - made global for easy monitoring*/
uint16_t head_page_address;
uint16_t head_byte_address;
uint16_t tail_page_address;
uint16_t tail_byte_address;

/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void RTC_Wakeup_120Hz_Init(void);
void RTC_Wakeup_1Hz_Init(void);
void InitAccel(void);
void InitTPH(void);
void GetAccel(void);
void GetRS(void);
void GetTPH(void);
void GetST(void);
void GetRT(void);
void GetWD(void);
void GetWS(void);
void A_Fifo_write(struct A_Fifo *f, uint16_t input);
uint8_t A_Fifo_read(struct A_Fifo *f);
void RS_Fifo_write(struct RS_Fifo *f, uint16_t input);
uint8_t RS_Fifo_read(struct RS_Fifo *f);
void TPH_Fifo_write(struct TPH_Fifo *f, int32_t input);
uint8_t TPH_Fifo_read(struct TPH_Fifo *f);
uint8_t ST_Fifo_read(struct ST_Fifo *f);
uint8_t RT_Fifo_read(struct RT_Fifo *f);
void WD_Fifo_write(struct WD_Fifo *f, uint8_t input);
uint8_t WD_Fifo_read(struct WD_Fifo *f);
int16_t dsp_ema_i32(int32_t in, int32_t average, uint16_t alpha);
uint16_t A_converter(uint16_t input);
uint8_t FR_Counter_10thsec(void);
uint8_t FR_Counter_1sec(void);
uint8_t FR_Counter_10secs(void);
uint8_t BATT_Counter_15mins(void);
uint8_t BATT_Counter_1min(void);
uint8_t BATT_Counter_15secs(void);
uint8_t BATT_Counter_2secs(void);
uint16_t RS_converter(uint16_t input);
uint16_t T1_converter(int32_t raw_t);
uint16_t P1_converter(int32_t raw_p);
uint16_t H1_converter(int32_t raw_h);
uint16_t T2_converter(int32_t raw_t);
uint16_t P2_converter(int32_t raw_p);
uint16_t H2_converter(int32_t raw_h);
uint16_t RT_converter(uint16_t input);
uint8_t WD_converter(uint8_t input);
uint16_t WS_converter(uint16_t input);
void FR_Routine_10thsec(void);
void FR_Routine_10secs(void);
void FR_Routine_1sec(void);
void BATT_Routine_15mins(void);
void BATT_Routine_1min(void);
void BATT_Routine_15secs(void);
void BATT_Routine_2secs(void);
void Motion_packet(void);
void FR_Env1_packet(void);
void BATT_Env1_packet(void);
void FR_Env2_packet(void);
void BATT_Env2_packet(void);
void FR_Wind_packet(void);
void BATT_Wind_packet(void);
struct TimeStamp GetTS(void);
void EraseFlash(uint32_t BlockAddress);
void WriteFlash(struct Flash_Fifo *f);
uint8_t FillUARTBuffer(struct Flash_Fifo *f);
void send_UART_Backup_Fifo(void);
void SwitchToBATTMode(void);
void SwitchToFRMode(void);
/* USER CODE END PFP */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* MCU Configuration----------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  /*The two sensors that require set up are Accel and TPH*/
  InitAccel();
  InitTPH();

  /*This function is called to begin the RTC wakeup interrupt at 120Hz*/
  RTC_Wakeup_120Hz_Init();

  /*Erase the first flash sector*/
  EraseFlash(0);
  HAL_Delay(110);
  EraseFlash(0b10000000000000);
  HAL_Delay(110);

  /*Enable the send and recieve UART interrupts*/
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_TC);
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);

  /*Define initial Run Mode:
   * Running_Mode = 1, RS_Instance_Switch = 2 -> Full Run (FR) Mode
   * Running_Mode = 0, RS_Instance_Switch = 0 -> Battery (BATT) Mode
   * */

  RS_Instance_Switch = 2;
  Running_Mode = 1;

  /* USER CODE END 2 */

  /* Infinite loop */
  while (1){
  /* USER CODE BEGIN 3 */
  /*RTC_ITF is set to 1 at 120 Hz by RTC wakeup interrupt*/
	  if(RTC_ITF == 1)
		{
		  switch(Running_Mode)
			{
		  /*Full Run Mode routine*/
		  case 1:
			  /*Do this at 120Hz*/
			  GetAccel();
			  /*If a tenth of a second has passed*/
			  if(FR_Counter_10thsec() == 1)
				{
				 FR_Routine_10thsec();
				}
			  /*If one second has passed*/
			  if(FR_Counter_1sec() == 1)
				{
				  FR_Routine_1sec();
				}
			  /*If 10 seconds has passed*/
			  if(FR_Counter_10secs() == 1)
				{
				  FR_Routine_10secs();
				}
			  /*Reset the interrupt type flag*/
			  RTC_ITF = 0;
			  break;
		  /*Battery Run Mode Routine*/
		  case 0:
			  /*If 15 minutes have passed, call the 15 minute routine.*/
			 if (BATT_Counter_15mins() == 1)
			 {
				 BATT_Routine_15mins();
			}
			 /*If one minute has passed, call the 1 minute routine.*/
			 if (BATT_Counter_1min() == 1)
			 {
				 BATT_Routine_1min();
			 }
			 /*If fifteen seconds has passed, call the 15 second routine.*/
			 if (BATT_Counter_15secs() == 1)
			 {
				 BATT_Routine_15secs();
			 }
			 /*If two seconds has passed, call the 2 second routine.*/
			 if (BATT_Counter_2secs() == 1)
			 {
				 BATT_Routine_2secs();
			 }
			 /*Reset the interrupt type flag*/
			 RTC_ITF = 0;
		     break;
	  	}
	  }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;
    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_9;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_SYSCLK;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{
  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;
    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the ADC multi-mode 
    */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00506682;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0x32F2){
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initialize RTC and set the Time and Date 
    */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Enable the WakeUp 
    */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR0,0x32F2);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

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
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 2401;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 5001;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 24001;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1001;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 24001;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1001;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS___FLASH_GPIO_Port, CS___FLASH_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS___FLASH_Pin */
  GPIO_InitStruct.Pin = CS___FLASH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS___FLASH_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Rotor_speed_Pin */
  GPIO_InitStruct.Pin = Rotor_speed_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Rotor_speed_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : WS_Pin */
  GPIO_InitStruct.Pin = WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/*Routine is called every 10th of a second in FR mode*
 *
 * Reads data out of Accel Fifos, through a low pass filter, and assigns to _to_save variables ready to be
 * used in packetizing function.
 * Saves current RS
 * Creates motion packet
 * Handles UART requests
 * Saves motion packet to flash.
 */
void FR_Routine_10thsec(void){

	/*Move data from accels through low pass filter and prepare to save as part of motion packet*/
	while(A_Fifo_read(&AX) != 1){
		uint16_t ax_raw = AX.output;
		AX_average = dsp_ema_i32((uint32_t)ax_raw, (uint32_t)AX_average, DSP_EMA_I32_ALPHA(.05));
		AX_to_save = A_converter(AX_average);
		}

	/*Move data from accels through low pass filter and prepare to save as part of motion packet*/
	while(A_Fifo_read(&AY) != 1){
		uint16_t ay_raw = AY.output;
		AY_average = dsp_ema_i32((uint32_t)ay_raw, (uint32_t)AY_average, DSP_EMA_I32_ALPHA(.05));
		AY_to_save = A_converter(AY_average);
		}
	/*Move data from accels through low pass filter and prepare to save as part of motion packet*/
	while(A_Fifo_read(&AZ) != 1){
		uint16_t az_raw = AZ.output;
		AZ_average = dsp_ema_i32((uint32_t)az_raw, (uint32_t)AZ_average, DSP_EMA_I32_ALPHA(.05));
		AZ_to_save = A_converter(AZ_average);
		}

	/*Save and convert current RS*/
	RS_to_save = RS_converter(RS_Time_Elapsed);

	/*Organize and save data in packet buffer*/
	Motion_packet();

	/*If a send data request has been made by the host, fill the uart buffer and send the contents*/
	if (UART_rxBuffer[0] == 1){
		FillUARTBuffer(&FF);
		HAL_UART_Transmit_IT(&huart2,&UART_Tx_Buffer[0], UART_bytes_in_send);
		UART_rxBuffer[0] = 0;
		}
	else if (UART_rxBuffer[0] == 2){
		send_UART_Backup_Fifo();
		UART_rxBuffer[0] = 0;
		}

	/*Write the newly collected data to flash. This is performed last to allow for write time*/
	WriteFlash(&FF);

	/*If RS drops below the BATT_FR_TRANSITION limit, enter batt mode*/
	if(RS_to_save < BATT_FR_TRANSITION){
		SwitchToBATTMode();
	}

}
/*Routine is called every 10 seconds in FR Mode
 * Averages TPH data and places into _to_save variables for packetization.
 * Average ST and RT data and save into _to_save variables for packetization.
 * Create Env1 packet and save to flash.
 * Create Env2 packet and save to flash.
 */
void FR_Routine_10secs(void){

/*Average the TPH data and save as part of env packets*/
	T1_sum_average = 0;
	uint8_t Sample_Counter = 0;
	while(TPH_Fifo_read(&T1) != 1){
		T1_sum_average += T1.output;
		Sample_Counter ++;
			}
		T1_average = T1_sum_average/Sample_Counter;
		T1_to_save = T1_average+27315;

	P1_sum_average = 0;
	Sample_Counter = 0;
	while(TPH_Fifo_read(&P1) != 1){
		P1_sum_average += P1.output;
		Sample_Counter ++;
			}
		P1_average = P1_sum_average/Sample_Counter;
		P1_to_save = P1_average;

	H1_sum_average = 0;
	Sample_Counter = 0;
	while(TPH_Fifo_read(&H1) != 1){
		H1_sum_average += H1.output;
		Sample_Counter ++;
			}
		H1_average = H1_sum_average/Sample_Counter;
		H1_to_save = H1_average;

	T2_sum_average = 0;
	Sample_Counter = 0;
	while(TPH_Fifo_read(&T2) != 1){
		T2_sum_average += T2.output;
		Sample_Counter ++;
			}
		T2_average = T2_sum_average/Sample_Counter;
		T2_to_save = T2_average + 27315;

	P2_sum_average = 0;
	Sample_Counter = 0;
	while(TPH_Fifo_read(&P2) != 1){
		P2_sum_average += P2.output;
		Sample_Counter ++;
			}
		P2_average = P2_sum_average/Sample_Counter;
		P2_to_save = P2_average;

	H2_sum_average = 0;
	Sample_Counter = 0;
	while(TPH_Fifo_read(&H2) != 1){
		H2_sum_average += H2.output;
		Sample_Counter ++;
			}
		H2_average = H2_sum_average/Sample_Counter;
		H2_to_save = H2_average;

	/*Average the ST data and save as part of env packets*/
	ST_1_sum_average = 0;
	Sample_Counter = 0;
	while(ST_Fifo_read(&ST1) != 1){
		ST_1_sum_average += ST1.output;
		Sample_Counter ++;
			}
		ST_1_average = ST_1_sum_average/Sample_Counter;
		/*convert to kelvin upon save*/
		ST_1_to_save = (ST_1_average + 273)*100;

	ST_2_sum_average = 0;
	Sample_Counter = 0;
	while(ST_Fifo_read(&ST2) != 1){
		ST_2_sum_average += ST2.output;
		Sample_Counter ++;
			}
		ST_2_average = ST_2_sum_average/Sample_Counter;
		/*convert to kelvin upon save*/
		ST_2_to_save = (ST_2_average + 273)*100;

	/*Average the RT data and save as part of env packets*/
	RT_sum_average = 0;
	Sample_Counter = 0;
	while(RT_Fifo_read(&RT) != 1){
		RT_sum_average += RT.output;
		Sample_Counter ++;
			}
		RT_average = RT_sum_average/Sample_Counter;
		RT_to_save = RT_average + 27315;

	/*Assemble the FR_Env1 packet*/
	FR_Env1_packet();

	/*Write the newly collected data to flash*/
	WriteFlash(&FF);

	/*Assemble the FR_Env1 packet*/
	FR_Env2_packet();

	/*Write the newly collected data to flash*/
	WriteFlash(&FF);
}

/*Routine that is called every second in FR Mode
 * Triggers collection of TPH, ST, RT, and WD data
 * Reads WD data out of fifo.
 * Converts WD and WS data and saves in _to_save variable for packetization.
 * Creates sind packet
 * Saves the wind packet to flash
 */
void FR_Routine_1sec(void){

	 uint8_t raw_wd;

	  GetTPH();
	  GetST();
	  GetRT();
	  GetWD();

	  while(WD_Fifo_read(&WD) != 1){
		 raw_wd = WD.output;
	  }

	  WD_to_save = WD_converter(raw_wd);

	  WS_to_save = WS_converter(WS_Time_Elapsed);

	  FR_Wind_packet();

	  WriteFlash(&FF);
}

/*Routine that occurs once every 15 mins in BATT mode.
 * Get data from TPH, ST, and RT.
 * Convert and save the data from TPH, ST, and RT fifos.
 * Compute the RS maximum during the 15 minute period.
 * Assemble the BATT_Env1 packet and save to flash
 * Assemble the Env2 packet and save to flash
 */
void BATT_Routine_15mins(void){

	/*Take single-shot measurements from TPH, ST, and RT sensors*/
	GetTPH();
	GetST();
	GetRT();

	/*Read values out of TPH fifos and convert to engineering units*/
	while(TPH_Fifo_read(&T1) != 1){
		T1_to_save = T1.output + 27315;
	}

	while(TPH_Fifo_read(&P1) != 1){
		P1_to_save = P1.output;
	}

	while(TPH_Fifo_read(&H1) != 1){
		H1_to_save = H1.output;
	}

	while(TPH_Fifo_read(&T2) != 1){
		T2_to_save = T2.output + 27315;
	}

	while(TPH_Fifo_read(&P2) != 1){
		P2_to_save = P2.output;
	}

	while(TPH_Fifo_read(&H2) != 1){
		H2_to_save = H2.output;
	}

	/*Read values out of ST fifos and convert to engineering units*/
	while(ST_Fifo_read(&ST1) != 1){
		ST_1_to_save = (ST1.output + 273) * 100;
	}

	while(ST_Fifo_read(&ST2) != 1){
		ST_2_to_save = (ST2.output + 273) * 100;
	}

	while(RT_Fifo_read(&RT) != 1){
		RT_to_save = RT.output + 27315;
	}

	/*Read values from 15 minutes worth of RS data. Find the shortest duration - this is converted to the maximum speed before being saved in RS_to_save*/
	RS_minimum = 10000;

	while(RS_Fifo_read(&RS) != 1){

		RS_storage = RS.output;
		if(RS_storage < RS_minimum && RS_storage != 0){
			RS_minimum = RS_storage;
		}
		}

		if (RS_minimum == 10000){
			RS_max_to_save = 0;
		}

		else{
		RS_max_to_save = RS_converter(RS_minimum);
		}

	/*Packetize Env1*/
	BATT_Env1_packet();

	WriteFlash(&FF);

	/*Packetize Env2*/
	BATT_Env2_packet();

	WriteFlash(&FF);

}

/*Routine occurs once a minute in BATT mode.
 * Read values from 1 minute worth of WS data. Find the shortest duration - this is converted to the maximum speed before being saved in WS_max_to_save
 * Computes the sum of all of the readings, divides by the number of readings and converts to save an average in WS_avg_to_save.
 * Reads out four values for WD stored in fifos, converts and puts in _to_save variables.
 * Assembles BATT wind packet
 * Handles UART requests
 * Saves BATT wind packet to flash.
 * */
void BATT_Routine_1min(void){

	/*Reset variables for averages and minimums*/
	WS_minimum = 10000;
	WS_sum_to_avg = 0;
	WS_count_to_avg = 0;

	/*Read out contents of WS_Fifo and calculate maximum and average. Convert to engineering units*/
	while(RS_Fifo_read(&WS) != 1){

		WS_count_to_avg ++;

		WS_sum_to_avg = WS_sum_to_avg + WS_converter(WS.output);

		WS_storage = WS.output;

		if(WS_storage < WS_minimum && WS_storage != 0){
			WS_minimum = WS_storage;
		}

	}

	if (WS_minimum == 10000){
		WS_max_to_save = 0;
	}

	else{
		WS_max_to_save = WS_converter(WS_minimum);
	}

	WS_avg_to_save = WS_sum_to_avg/WS_count_to_avg;

	/*Read out four values from WD_fifo and save them as WD_to_save_1, 2, 3, & 4. */
	WD_Fifo_read(&WD);
	WD_to_save_1 = WD_converter(WD.output);

	WD_Fifo_read(&WD);
	WD_to_save_2 = WD_converter(WD.output);

	WD_Fifo_read(&WD);
	WD_to_save_3 = WD_converter(WD.output);

	WD_Fifo_read(&WD);
	WD_to_save_4 = WD_converter(WD.output);

	BATT_Wind_packet();

	/*If a send data request has been made by the host, fill the uart buffer and send the contents*/
	if (UART_rxBuffer[0] == 1){
		FillUARTBuffer(&FF);
		HAL_UART_Transmit_IT(&huart2,&UART_Tx_Buffer[0], UART_bytes_in_send);
		UART_rxBuffer[0] = 0;
		}

	else if (UART_rxBuffer[0] == 2){
		send_UART_Backup_Fifo();
		UART_rxBuffer[0] = 0;
		}

	WriteFlash(&FF);

}

/*Routine occurs every 15s in BATT mode.
 *
 * Calls GetRS() to take a single rotor speed measurement.
 * Takes a single WD reading.
 */
void BATT_Routine_15secs(void){

/*Allows for one rotor speed measurement until GetRS is called again*/
GetRS();

/*Get a single ADC conversion of the Wind Vane position*/
GetWD();

}

/*Routine occurs every 2 seconds in BATT mode.
 * Calls GetWS(); to take a single windspeed measurement.
 * checks the value of most recently saved RS. If it is above the transition speed, switch to full run mode
 * */
void BATT_Routine_2secs(void){

/*Allows for one wind speed measurement*/
GetWS();

if(RS_converter(RS_Time_Elapsed) > BATT_FR_TRANSITION){
	SwitchToFRMode();
}

}

/*Assembles Motion Packet data and saves to packet buffer*/
void Motion_packet(void){

	uint16_t tenths_to_save = 0;
	uint crc = 0;
	uint8_t generator = 0x07;
	uint8_t type_sequence;

	/*Get the current time stamp info and save it into the TS struct.*/
	TS = GetTS();

	/*Read the number of tenths after the hour and save it in variable to be saved*/
	tenths_to_save = TS.Tenths_After_Hour;

	/*sequence_counter increments by one for each packet creation. Used for data integrity check in transmission*/
	if (sequence_counter < 31){
		sequence_counter ++;
	}
	else{
		sequence_counter = 0;
	}

	/*Create a type sequence byte - first three bits are type signifier. Last 5 are sequence counter value*/
	type_sequence = Motion_Type | sequence_counter;

		Packet_Buffer[0] = type_sequence;
		Packet_Buffer[1] = tenths_to_save >> 8;
		Packet_Buffer[2] = tenths_to_save;
		Packet_Buffer[3] = AX_to_save >> 8;
		Packet_Buffer[4] = AX_to_save;
		Packet_Buffer[5] = AY_to_save >> 8;
		Packet_Buffer[6] = AY_to_save;
		Packet_Buffer[7] = AZ_to_save >> 8;
		Packet_Buffer[8] = AZ_to_save;
		Packet_Buffer[9] = RS_to_save >> 8;
		Packet_Buffer[10] = RS_to_save;
		Packet_Buffer[11] = 0b00000000;
		Packet_Buffer[12] = 0b00000000;


		for (int i = 0; i < 13; i++){
			crc^=Packet_Buffer[i];

			for(int j = 0; j < 8; j++){
				if ((crc & 0x80) != 0){
					crc = (uint8_t)((crc << 1) ^ generator);
					}
					else{
						crc <<= 1;
					}
				}
			}
		Packet_Buffer[13] = crc;
}

/*Assembles free run environmental data and saves to packet buffer*/
void FR_Env1_packet(void){

	uint16_t tenths_to_save = 0;
	uint crc = 0;
	uint8_t generator = 0x07;
	uint8_t type_sequence;

	/*Get the current time stamp info and save it into the TS struct.*/
	TS = GetTS();

	/*Read the number of tenths after the hour and save it in variable to be saved*/
	tenths_to_save = TS.Tenths_After_Hour;

	/*sequence_counter increments by one for each packet creation. Used for data integrity check in transmission*/
	if (sequence_counter < 32){
		sequence_counter ++;
	}
	else{
		sequence_counter = 0;
	}

	/*Create a type sequence byte - first three bits are type signifier. Last 5 are sequence counter value*/
	type_sequence = FR_Env1_Type | sequence_counter;

	/*Fill packet buffer*/
	Packet_Buffer[0] = type_sequence;
	Packet_Buffer[1] = tenths_to_save >> 8;
	Packet_Buffer[2] = tenths_to_save;
	Packet_Buffer[3] = H1_to_save >> 8;
	Packet_Buffer[4] = H1_to_save;
	Packet_Buffer[5] = T1_to_save >> 8;
	Packet_Buffer[6] = T1_to_save;
	Packet_Buffer[7] = P1_to_save >> 8;
	Packet_Buffer[8] = P1_to_save;
	Packet_Buffer[9] = ST_1_to_save >> 8;
	Packet_Buffer[10] = ST_1_to_save;
	Packet_Buffer[11] = 0b00000000;
	Packet_Buffer[12] = 0b00000000;

	/*Compute crc and store it as the 14th byte of information sent*/
	for (int i = 0; i < 13; i++){
		crc^=Packet_Buffer[i];

		for(int j = 0; j < 8; j++){
			if ((crc & 0x80) != 0){
				crc = (uint8_t)((crc << 1) ^ generator);
				}
				else{
					crc <<= 1;
				}
			}
		}
	Packet_Buffer[13] = crc;


}

/*Assembles battery environmental data and saves to packet buffer*/
void BATT_Env1_packet(void){


	uint16_t tenths_to_save = 0;
	uint crc = 0;
	uint8_t generator = 0x07;
	uint8_t type_sequence;

	/*Get the current time stamp info and save it into the TS struct.*/
	TS = GetTS();

	/*Read the number of tenths after the hour and save it in variable to be saved*/
	tenths_to_save = TS.Tenths_After_Hour;

	/*sequence_counter increments by one for each packet creation. Used for data integrity check in transmission*/
	if (sequence_counter < 31){
		sequence_counter ++;
	}
	else{
		sequence_counter = 0;
	}

	/*Create a type sequence byte - first three bits are type signifier. Last 5 are sequence counter value*/
	type_sequence = BATT_Env1_Type | sequence_counter;

	/*Fill packet buffer*/
	Packet_Buffer[0] = type_sequence;

	Packet_Buffer[1] = tenths_to_save >> 8;
	Packet_Buffer[2] = tenths_to_save;
	Packet_Buffer[3] = H1_to_save >> 8;
	Packet_Buffer[4] = H1_to_save;
	Packet_Buffer[5] = T1_to_save >> 8;
	Packet_Buffer[6] = T1_to_save;
	Packet_Buffer[7] = P1_to_save >> 8;
	Packet_Buffer[8] = P1_to_save;
	Packet_Buffer[9] = ST_1_to_save >> 8;
	Packet_Buffer[10] = ST_1_to_save;
	Packet_Buffer[11] = RS_max_to_save >> 8;
	Packet_Buffer[12] = RS_max_to_save;

	/*Compute crc and store it as the 14th byte of information sent*/
	for (int i = 0; i < 13; i++){
		crc^=Packet_Buffer[i];

		for(int j = 0; j < 8; j++){
			if ((crc & 0x80) != 0){
				crc = (uint8_t)((crc << 1) ^ generator);
				}
				else{
					crc <<= 1;
				}
			}
		}
	Packet_Buffer[13] = crc;

}

/*Assembles free run environmental data and saves to packet buffer*/
void FR_Env2_packet(void){

	uint16_t tenths_to_save = 0;
	uint crc = 0;
	uint8_t generator = 0x07;
	uint8_t type_sequence;

	/*Get the current time stamp info and save it into the TS struct.*/
	TS = GetTS();

	/*Read the number of tenths after the hour and save it in variable to be saved*/
	tenths_to_save = TS.Tenths_After_Hour;

	/*sequence_counter increments by one for each packet creation. Used for data integrity check in transmission*/
	if (sequence_counter < 32){
		sequence_counter ++;
	}
	else{
		sequence_counter = 0;
	}

	/*Create a type sequence byte - first three bits are type signifier. Last 5 are sequence counter value*/
	type_sequence = FR_Env2_Type | sequence_counter;

	/*Fill packet buffer*/
	Packet_Buffer[0] = type_sequence;
	Packet_Buffer[1] = tenths_to_save >> 8;
	Packet_Buffer[2] = tenths_to_save;
	Packet_Buffer[3] = H2_to_save >> 8;
	Packet_Buffer[4] = H2_to_save;
	Packet_Buffer[5] = T2_to_save >> 8;
	Packet_Buffer[6] = T2_to_save;
	Packet_Buffer[7] = P2_to_save >> 8;
	Packet_Buffer[8] = P2_to_save;
	Packet_Buffer[9] = ST_2_to_save >> 8;
	Packet_Buffer[10] = ST_2_to_save;
	Packet_Buffer[11] = RT_to_save >> 8;
	Packet_Buffer[12] = RT_to_save;

	/*Compute crc and store it as the 14th byte of information sent*/
	for (int i = 0; i < 13; i++){
		crc^=Packet_Buffer[i];

		for(int j = 0; j < 8; j++){
			if ((crc & 0x80) != 0){
				crc = (uint8_t)((crc << 1) ^ generator);
				}
				else{
					crc <<= 1;
				}
			}
		}
	Packet_Buffer[13] = crc;

}

/*Assembles battery environmental data and saves to packet buffer*/
void BATT_Env2_packet(void){

	uint16_t tenths_to_save = 0;
	uint crc = 0;
	uint8_t generator = 0x07;
	uint8_t type_sequence;

	/*Get the current time stamp info and save it into the TS struct.*/
	TS = GetTS();

	/*Read the number of tenths after the hour and save it in variable to be saved*/
	tenths_to_save = TS.Tenths_After_Hour;

	/*sequence_counter increments by one for each packet creation. Used for data integrity check in transmission*/
	if (sequence_counter < 32){
		sequence_counter ++;
	}
	else{
		sequence_counter = 0;
	}

	/*Create a type sequence byte - first three bits are type signifier. Last 5 are sequence counter value*/
	type_sequence = BATT_Env2_Type | sequence_counter;

	/*Fill packet buffer*/
	Packet_Buffer[0] = type_sequence;
	Packet_Buffer[1] = tenths_to_save >> 8;
	Packet_Buffer[2] = tenths_to_save;
	Packet_Buffer[3] = H2_to_save >> 8;
	Packet_Buffer[4] = H2_to_save;
	Packet_Buffer[5] = T2_to_save >> 8;
	Packet_Buffer[6] = T2_to_save;
	Packet_Buffer[7] = P2_to_save >> 8;
	Packet_Buffer[8] = P2_to_save;
	Packet_Buffer[9] = ST_2_to_save >> 8;
	Packet_Buffer[10] = ST_2_to_save;
	Packet_Buffer[11] = RT_to_save >> 8;
	Packet_Buffer[12] = RT_to_save;

	/*Compute crc and store it as the 14th byte of information sent*/
	for (int i = 0; i < 13; i++){
		crc^=Packet_Buffer[i];

		for(int j = 0; j < 8; j++){
			if ((crc & 0x80) != 0){
				crc = (uint8_t)((crc << 1) ^ generator);
				}
				else{
					crc <<= 1;
				}
			}
		}
	Packet_Buffer[13] = crc;

}

/*Assembles free run wind data and saves to packet buffer*/
void FR_Wind_packet(void){

	uint16_t tenths_to_save = 0;
	uint crc = 0;
	uint8_t generator = 0x07;
	uint8_t type_sequence;

	/*Get the current time stamp info and save it into the TS struct.*/
	TS = GetTS();

	/*Read the number of tenths after the hour and save it in variable to be saved*/
	tenths_to_save = TS.Tenths_After_Hour;

	/*sequence_counter increments by one for each packet creation. Used for data integrity check in transmission*/
	if (sequence_counter < 31){
		sequence_counter ++;
	}
	else{
		sequence_counter = 0;
	}

	/*Create a type sequence byte - first three bits are type signifier. Last 5 are sequence counter value*/
	type_sequence = FR_Wind_Type | sequence_counter;



	/*Fill packet buffer*/
	Packet_Buffer[0] = type_sequence;
	Packet_Buffer[1] = tenths_to_save >> 8;
	Packet_Buffer[2] = tenths_to_save;
	Packet_Buffer[3] = WS_to_save >> 8;
	Packet_Buffer[4] = WS_to_save;
	Packet_Buffer[5] = 0b00000000;
	Packet_Buffer[6] = 0b00000000;
	Packet_Buffer[7] = WD_to_save;
	Packet_Buffer[8] = 0b00000000;
	Packet_Buffer[9] = 0b00000000;
	Packet_Buffer[10] = 0b00000000;
	Packet_Buffer[11] = 0b00000000;
	Packet_Buffer[12] = 0b00000000;

	/*Compute crc and store it as the 14th byte of information sent*/
	for (int i = 0; i < 13; i++){
		crc^=Packet_Buffer[i];

		for(int j = 0; j < 8; j++){
			if ((crc & 0x80) != 0){
				crc = (uint8_t)((crc << 1) ^ generator);
				}
				else{
					crc <<= 1;
				}
			}
		}
	Packet_Buffer[13] = crc;

}

/*Assembles battery wind data and saves to packet buffer*/
void BATT_Wind_packet(void){

	uint16_t tenths_to_save = 0;
	uint crc = 0;
	uint8_t generator = 0x07;
	uint8_t type_sequence;

	/*Get the current time stamp info and save it into the TS struct.*/
	TS = GetTS();

	/*Read the number of tenths after the hour and save it in variable to be saved*/
	tenths_to_save = TS.Tenths_After_Hour;

	/*sequence_counter increments by one for each packet creation. Used for data integrity check in transmission*/
	if (sequence_counter < 31){
		sequence_counter ++;
	}
	else{
		sequence_counter = 0;
	}

	/*Create a type sequence byte - first three bits are type signifier. Last 5 are sequence counter value*/
	type_sequence = BATT_Wind_Type | sequence_counter;

	/*Fill packet buffer*/
	Packet_Buffer[0] = type_sequence;
	Packet_Buffer[1] = tenths_to_save >> 8;
	Packet_Buffer[2] = tenths_to_save;
	Packet_Buffer[3] = WS_avg_to_save >> 8;
	Packet_Buffer[4] = WS_avg_to_save;
	Packet_Buffer[5] = WS_max_to_save >> 8;
	Packet_Buffer[6] = WS_max_to_save;
	Packet_Buffer[7] = WD_to_save_1;
	Packet_Buffer[8] = WD_to_save_2;
	Packet_Buffer[9] = WD_to_save_3;
	Packet_Buffer[10] = WD_to_save_4;
	Packet_Buffer[11] = 0b00000000;
	Packet_Buffer[12] = 0b00000000;

	/*Compute crc and store it as the 14th byte of information sent*/
	for (int i = 0; i < 13; i++){
		crc^=Packet_Buffer[i];

		for(int j = 0; j < 8; j++){
			if ((crc & 0x80) != 0){
				crc = (uint8_t)((crc << 1) ^ generator);
				}
				else{
					crc <<= 1;
				}
			}
		}
	Packet_Buffer[13] = crc;

}

/*Reads RTC and converts time into hours after new year and tenths of a second after the hour*/
struct TimeStamp GetTS(void)
	{

	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;

	float tenths;
	uint16_t tenths_after_hour = 0;
	uint16_t month_hours = 0;
	uint16_t hours_after_new_year = 0;

	HAL_RTC_GetTime(&hrtc, &sTime,RTC_FORMAT_BIN);
	tenths = (sTime.Minutes*600)+(sTime.Seconds*10)+(10*(256-sTime.SubSeconds)*.00390625);
	tenths_after_hour = (uint16_t)tenths;
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	switch(sDate.Month){
		case 1:
			month_hours = 0;
			break;
		case 2:
			month_hours = 744;
			break;
		case 3:
			month_hours = 744+672;
			if(sDate.Year % 4 == 0){
				month_hours = month_hours+24;
			}
			break;
		case 4:
			month_hours = 744+672+744;
			if(sDate.Year % 4 == 0){
				month_hours = month_hours+24;
			}
			break;
		case 5:
			month_hours = 744+672+744+720;
			if(sDate.Year % 4 == 0){
				month_hours = month_hours+24;
			}
			break;
		case 6:
			month_hours = 744+672+744+720+744;
			if(sDate.Year % 4 == 0){
				month_hours = month_hours+24;
			}
			break;
		case 7:
			month_hours = 744+672+744+720+744+720;
			if(sDate.Year % 4 == 0){
				month_hours = month_hours+24;
			}
			break;
		case 8:
			month_hours = 744+672+744+720+744+720+744;
			if(sDate.Year % 4 == 0){
				month_hours = month_hours+24;
			}
			break;
		case 9:
			month_hours = 744+672+744+720+744+720+744+744;
			if(sDate.Year % 4 == 0){
				month_hours = month_hours+24;
			}
			break;
		case 10:
			month_hours = 744+672+744+720+744+720+744+744+720;
			if(sDate.Year % 4 == 0){
				month_hours = month_hours+24;
			}
			break;
		case 11:
			month_hours = 744+672+744+720+744+720+744+744+720+744;
			if(sDate.Year % 4 == 0){
				month_hours = month_hours+24;
			}
			break;
		case 12:
			month_hours = 744+672+744+720+744+720+744+744+720+744+720;
			if(sDate.Year % 4 == 0){
				month_hours = month_hours+24;
			}
			break;
	}
	hours_after_new_year = month_hours + ((sDate.Date-1)*24) + sTime.Hours;
	struct TimeStamp x = {tenths_after_hour,hours_after_new_year};
	return x;
	}

/*Converts from time elapsed to rotor speed.
 * Output is RPM * 100. The last decimal is truncated, not rounded.
 */
uint16_t RS_converter(uint16_t input){

	uint16_t Rotor_Speed;
	uint32_t x_1 = 1000000/input;
    uint32_t x_2 = x_1*60;
    uint32_t RPM = x_2/16;
    Rotor_Speed = (uint16_t)RPM;
    return Rotor_Speed;

}

/* Conversion functions for TPH 1 + 2. Different functions for each due to different trim values*/
uint16_t T1_converter(int32_t raw_t ){

	//for TPH Sensor 1:

	//convert temperature with trim values:
	var1_1 = ((((raw_t>>3) - (dig_T1_1<<1)))*(dig_T2_1)) >> 11;
	int32_t var2 = (((((raw_t>>4) - (dig_T1_1)) * ((raw_t>>4) - (dig_T1_1))) >> 12) * (dig_T3_1)) >> 14;
	t_fine_1 = var1_1 + var2;
	PH_T_1 = (t_fine_1 * 5 + 128) >> 8;
	return (uint16_t)PH_T_1;

}

uint16_t P1_converter(int32_t raw_p){

	//convert pressure with trim values:
	int64_t p;
	int64_t var3 = (int64_t)t_fine_1 - 128000;
	int64_t var4 = var3 * var3 * (int64_t)dig_P6_1;
	var4 = var4 +((var3*(int64_t)dig_P5_1)<<17);
	var4 = var4 + ((int64_t)(dig_P4_1) << 35);
	var3 = ((var3 * var1_1 * (int64_t)dig_P3_1)>>8) + ((var3 *(int64_t)dig_P2_1)<<12);
	var3 = ((((int64_t)(1)<<47) + var3)) * (int64_t)(dig_P1_1) >>33;
	if (var3 == 0)
	{
		p = 0; // avoid exception caused by division by zero
	}
	p = 1048579 - raw_p;
	p = (((p<<31)-var4)*3125)/var3;
	var3 = ((int64_t)(dig_P9_1) * (p>>13) * (p>>13)) >> 25;
	var4 = ((int64_t)(dig_P8_1) * p) >> 19;
	p = ((p + var3 + var4) >> 8) + ((int64_t)(dig_P7_1)<<4);
	PH_P_1 = (uint32_t)p;
	PH_P_1 = (float)PH_P_1/(250.6*10);//added 100 to scale down.256
	return (uint16_t)PH_P_1;
}

uint16_t H1_converter(int32_t raw_h){

	uint32_t h = (t_fine_1 - ((int32_t)76800));
	h = (((((raw_h << 14) - (((int32_t)dig_H4_1) << 20) - (((int32_t)dig_H5_1) * h)) +
	((int32_t)16384)) >> 15) * (((((((h * ((int32_t)dig_H6_1)) >> 10) * (((h *
	((int32_t)dig_H3_1)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
	((int32_t)dig_H2_1) + 8192) >> 14));
	h = (h - (((((h >> 15) * (h >> 15)) >> 7) * ((int32_t)dig_H1_1)) >> 4));
	h = (h > 419430400 ? 419430400 : h);
	PH_H_1 = ((uint32_t)(h>>12))/1024;
	return (uint16_t)PH_H_1;

}

uint16_t T2_converter(int32_t raw_t ){

	//for TPH Sensor 2:

	//convert temperature with trim values:
	var1_2 = ((((raw_t>>3) - (dig_T1_2<<1)))*(dig_T2_2)) >> 11;
	int32_t var2 = (((((raw_t>>4) - (dig_T1_2)) * ((raw_t>>4) - (dig_T1_2))) >> 12) * (dig_T3_2)) >> 14;
	t_fine_2 = var1_2 + var2;
	PH_T_2 = (t_fine_2 * 5 + 128) >> 8;
	return (uint16_t)PH_T_2;

}

uint16_t P2_converter(int32_t raw_p){

	//convert pressure with trim values:
	int64_t p;
	int64_t var3 = (int64_t)t_fine_2 - 128000;
	int64_t var4 = var3 * var3 * (int64_t)dig_P6_2;
	var4 = var4 +((var3*(int64_t)dig_P5_2)<<17);
	var4 = var4 + ((int64_t)(dig_P4_2) << 35);
	var3 = ((var3 * var1_2 * (int64_t)dig_P3_2)>>8) + ((var3 *(int64_t)dig_P2_2)<<12);
	var3 = ((((int64_t)(1)<<47) + var3)) * (int64_t)(dig_P1_2) >>33;
	if (var3 == 0)
	{
		p = 0; // avoid exception caused by division by zero
	}
	p = 1048579 - raw_p;
	p = (((p<<31)-var4)*3125)/var3;
	var3 = ((int64_t)(dig_P9_2) * (p>>13) * (p>>13)) >> 25;
	var4 = ((int64_t)(dig_P8_2) * p) >> 19;
	p = ((p + var3 + var4) >> 8) + ((int64_t)(dig_P7_2)<<4);
	PH_P_2 = (uint32_t)p;
	PH_P_2 = (float)PH_P_2/(250.7*10);//added 100 to scale down.256
	return (uint16_t)PH_P_2;
}

uint16_t H2_converter(int32_t raw_h){

	uint32_t h = (t_fine_2 - ((int32_t)76800));
	h = (((((raw_h << 14) - (((int32_t)dig_H4_2) << 20) - (((int32_t)dig_H5_2) * h)) +
	((int32_t)16384)) >> 15) * (((((((h * ((int32_t)dig_H6_2)) >> 10) * (((h *
	((int32_t)dig_H3_2)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
	((int32_t)dig_H2_2) + 8192) >> 14));
	h = (h - (((((h >> 15) * (h >> 15)) >> 7) * ((int32_t)dig_H1_2)) >> 4));
	h = (h > 419430400 ? 419430400 : h);
	PH_H_2 = ((uint32_t)(h>>12))/1024;
	return (uint16_t)PH_H_2;

}

/*Converts raw RT to degress K*100*/
uint16_t RT_converter(uint16_t input){

	float var1 = (float)input;
	float IR_T_obj = ((.02*var1)-273.15)*100;
	uint16_t result = (uint16_t)IR_T_obj;
	return result;

}

/*Converts raw WD readings to degrees/2*/
uint8_t WD_converter(uint8_t input){

	uint16_t wind_dir_degrees;

	/*Due to an 8 degree deadband where the reading is 255, the best we can do is say that everytime the deadband is hit, it is in the center of the deadband*/
	if (input == 255){
		wind_dir_degrees = 356/2;
		}
	else{
		wind_dir_degrees = (float)input * 1.430894 - 9;
		}
	return wind_dir_degrees/2;
}

/*Returns a WS value in m/s * 100*/
uint16_t WS_converter(uint16_t input){

	if(input == 0){
		return 0;
	}
	return (77700/input)+19;

}


/*When called by in main loop (in FR mode), increments. If .1 seconds has passed: returns 1. If not: returns 0.*/
uint8_t FR_Counter_10thsec(void){
	if (cnt_FR10thsec < COUNT_FR10TH_SEC){
		cnt_FR10thsec ++;
		return 0;
	}
	else{
		cnt_FR10thsec = 0;
		return 1;
	}
}

/*When called by in main loop (in FR mode), increments. If 1 second has passed: returns 1. If not: returns 0.*/
uint8_t FR_Counter_1sec(void){
	if (cnt_FR1sec < COUNT_1_FRSEC){
		cnt_FR1sec ++;
		return 0;
	}
	else{
		cnt_FR1sec = 0;
		return 1;
	}
}

/*When called in main loop (in FR mode), increments. If 10 seconds has passed: returns 1. If not: returns 0.*/
uint8_t FR_Counter_10secs(void){
	if (cnt_FR10secs < COUNT_10_FRSECS){
		cnt_FR10secs ++;
		return 0;
	}
	else{
		cnt_FR10secs = 0;
		return 1;
	}
}

/*When called in main loop (in FR mode), increments. If 1 minute has passed: returns 1. If not: returns 0.*/
uint8_t FR_Counter_1min(void){
	if (cnt_FR1min < COUNT_1_FRMIN){
		cnt_FR1min ++;
		return 0;
	}
	else{
		cnt_FR1min = 0;
		return 1;
	}
}

/*When called by RTC 1s interrupt, increments. If 15 minutes has passed: returns 1. If not: returns 0. Counts up to 900*/
uint8_t BATT_Counter_15mins(void){
	if(cnt_BATT15mins < COUNT_15_BATTMINS){
		cnt_BATT15mins ++;
		return 0;
	}
	else{
		cnt_BATT15mins = 0;
		return 1;
	}
}

/*When called by RTC 1s interrupt, increments. If 1 minute has passed: returns 1. If not: returns 0.*/
uint8_t BATT_Counter_1min(void){
	if(cnt_BATT1min < COUNT_1_BATTMIN){
		cnt_BATT1min ++;
		return 0;
	}
	else{
		cnt_BATT1min = 0;
		return 1;
	}
}

/*When called by RTC 1s interrupt, increments. If 15 seconds has passed: returns 1. If not: returns 0. -- set to 15 later.*/
uint8_t BATT_Counter_15secs(void){
	if(cnt_BATT15secs < COUNT_15_BATTSECS){
		cnt_BATT15secs ++;
		return 0;
	}
	else{
		cnt_BATT15secs = 0;
		return 1;
	}
}

/*When called by RTC 1s interrupt, increments. If 2 seconds has passed: returns 1. If not: returns 0.*/
uint8_t BATT_Counter_2secs(void){
	if(cnt_BATT2secs < COUNT_2_BATTSECS){
		cnt_BATT2secs ++;
		return 0;
	}
	else{
		cnt_BATT2secs = 0;
		return 1;
	}
}

/*Sets RTC wakeup interrupt to fire at 120Hz for FR mode.*/
void RTC_Wakeup_120Hz_Init(void){
	/* Disable the write protection for RTC registers */
	__HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);

	/* Disable the Wake-up Timer */
	__HAL_RTC_WAKEUPTIMER_DISABLE(&hrtc);

	/* In case of interrupt mode is used, the interrupt source must disabled */
	__HAL_RTC_WAKEUPTIMER_DISABLE_IT(&hrtc,RTC_IT_WUT);

	/* Wait till RTC WUTWF flag is set  */
	while(__HAL_RTC_WAKEUPTIMER_GET_FLAG(&hrtc, RTC_FLAG_WUTWF) == RESET){}

	/* Set wake up timer to trigger every 1/120th s. */
	HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 132, RTC_WAKEUPCLOCK_RTCCLK_DIV2);

	/*Enable IRQ*/
	HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
}

/*Sets RTC wakeup interrupt to fire at 1Hz for BATT mode.*/
void RTC_Wakeup_1Hz_Init(void){
	/* Disable the write protection for RTC registers */
	__HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);

	/* Disable the Wake-up Timer */
	__HAL_RTC_WAKEUPTIMER_DISABLE(&hrtc);

	/* In case of interrupt mode is used, the interrupt source must disabled */
	__HAL_RTC_WAKEUPTIMER_DISABLE_IT(&hrtc,RTC_IT_WUT);

	/* Wait till RTC WUTWF flag is set */
	while(__HAL_RTC_WAKEUPTIMER_GET_FLAG(&hrtc, RTC_FLAG_WUTWF) == RESET){}

	/* Set wake up timer to trigger every 1/120th s. */
	HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 15350, RTC_WAKEUPCLOCK_RTCCLK_DIV2);

	/*Enable IRQ*/
	HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
}


/*Fifo read function for accelerometer*/
uint8_t A_Fifo_read(struct A_Fifo *f){

	f->emptyifone = 0;

	if(f->tail != f->head){

		f->output = f->buffer[f->tail];

		if (f->tail != A_BUFFER_SIZE - 1){
			f->tail++;
		}
		else{
			f->tail = 0;
		}
		return 0;
	}

	else{
		f->emptyifone = 1;
		return 1;
	}
}
/*Fifo write function for accelerometer.*/
void A_Fifo_write(struct A_Fifo *f, uint16_t input){

	f->fullifone = 0;

	if (f->head == f->tail - 1){

		f->fullifone = 1;

	}

	else if ((f->head == A_BUFFER_SIZE - 1) && (f->tail == 0)){

		f->fullifone = 1;
	}

	else{

		f->buffer[f->head] = input;

		if (f->head != A_BUFFER_SIZE - 1){
				f->head ++;
			}
		else{
				f->head = 0;
			}
		}
}

/*Fifo read function for Rotor Speed*/
uint8_t RS_Fifo_read(struct RS_Fifo *f){

	f->emptyifone = 0;

	if(f->tail != f->head){

		f->output = f->buffer[f->tail];

		if (f->tail != RS_BUFFER_SIZE - 1){
			f->tail++;
		}
		else{
			f->tail = 0;
		}
		return 0;
	}

	else{
		f->emptyifone = 1;
		return 1;
	}
}

/*Fifo write function for Rotor Speed*/
void RS_Fifo_write(struct RS_Fifo *f, uint16_t input){

	f->fullifone = 0;

	if (f->head == f->tail - 1){

		f->fullifone = 1;

	}

	else if ((f->head == RS_BUFFER_SIZE - 1) && (f->tail == 0)){

		f->fullifone = 1;
	}

	else{

		f->buffer[f->head] = input;

		if (f->head != RS_BUFFER_SIZE - 1){
				f->head ++;
			}
		else{
				f->head = 0;
			}
		}
}

/*Fifo write function for TPH sensor*/
void TPH_Fifo_write(struct TPH_Fifo *f, int32_t input){

	f->fullifone = 0;

	if (f->head == f->tail - 1){

		f->fullifone = 1;

	}

	else if ((f->head == TPH_BUFFER_SIZE - 1) && (f->tail == 0)){

		f->fullifone = 1;
	}

	else{

		f->buffer[f->head] = input;

		if (f->head != TPH_BUFFER_SIZE - 1){
				f->head ++;
			}
		else{
				f->head = 0;
			}
		}
}

/*Fifo read function for TPH sensor*/
uint8_t TPH_Fifo_read(struct TPH_Fifo *f){

	f->emptyifone = 0;

	if(f->tail != f->head){

		f->output = f->buffer[f->tail];

		if (f->tail != TPH_BUFFER_SIZE - 1){
			f->tail++;
		}
		else{
			f->tail = 0;
		}
		return 0;
	}

	else{
		f->emptyifone = 1;
		return 1;
	}
}

/*Fifo write function for Stator Temp*/
void ST_Fifo_write(struct ST_Fifo *f, int8_t input){

	f->fullifone = 0;

	if (f->head == f->tail - 1){

		f->fullifone = 1;

	}

	else if ((f->head == ST_BUFFER_SIZE - 1) && (f->tail == 0)){

		f->fullifone = 1;
	}

	else{

		f->buffer[f->head] = input;

		if (f->head != ST_BUFFER_SIZE - 1){
				f->head ++;
			}
		else{
				f->head = 0;
			}
		}
}

/*Fifo read function for stator Temp*/
uint8_t ST_Fifo_read(struct ST_Fifo *f){

	f->emptyifone = 0;

	if(f->tail != f->head){

		f->output = f->buffer[f->tail];

		if (f->tail != ST_BUFFER_SIZE - 1){
			f->tail++;
		}
		else{
			f->tail = 0;
		}
		return 0;
	}

	else{
		f->emptyifone = 1;
		return 1;
	}
}

/*Fifo write function for Rotor Temp*/
void RT_Fifo_write(struct RT_Fifo *f, uint16_t input){

	f->fullifone = 0;

	if (f->head == f->tail - 1){

		f->fullifone = 1;

	}

	else if ((f->head == RT_BUFFER_SIZE - 1) && (f->tail == 0)){

		f->fullifone = 1;
	}

	else{

		f->buffer[f->head] = input;

		if (f->head != RT_BUFFER_SIZE - 1){
				f->head ++;
			}
		else{
				f->head = 0;
			}
		}
}

/*Fifo read function for Rotor Temp*/
uint8_t RT_Fifo_read(struct RT_Fifo *f){

	f->emptyifone = 0;

	if(f->tail != f->head){

		f->output = f->buffer[f->tail];

		if (f->tail != RT_BUFFER_SIZE - 1){
			f->tail++;
		}
		else{
			f->tail = 0;
		}
		return 0;
	}

	else{

		f->emptyifone = 1;
		return 1;
	}
}

/*Fifo read function for Wind Direction*/
uint8_t WD_Fifo_read(struct WD_Fifo *f){

	f->emptyifone = 0;

	if(f->tail != f->head){

		f->output = f->buffer[f->tail];

		if (f->tail != WD_BUFFER_SIZE - 1){
			f->tail++;
		}
		else{
			f->tail = 0;
		}
		return 0;
	}

	else{
		f->emptyifone = 1;
		return 1;
	}
}

/*Fifo write function for Wind Direction*/
void WD_Fifo_write(struct WD_Fifo *f, uint8_t input){

	f->fullifone = 0;

	if (f->head == f->tail - 1){

		f->fullifone = 1;

	}

	else if ((f->head == WD_BUFFER_SIZE - 1) && (f->tail == 0)){

		f->fullifone = 1;
	}

	else{

		f->buffer[f->head] = input;

		if (f->head != WD_BUFFER_SIZE - 1){
				f->head ++;
			}
		else{
				f->head = 0;
			}
		}

}
/*Initiates a read of the accelerometer.
 * Data is burst read and saved upon read complete interrupt.
 */
void GetAccel(void){
	uint8_t register_address = 0x28 | 0x80;
	RxCpltType = ACCEL_RX_CPLT;
	HAL_I2C_Mem_Read_IT(&hi2c1, A_DEVICE_ADDRESS<<1, register_address, 1, i2c_storage_buffer, 6);
}

/*Writes value of RS_Time_Elapsed is to Fifo*/
void GetRS(void){

	RS_Fifo_write(&RS, RS_Time_Elapsed);
}

/*Writes current value of WS_Time_Elapsed to Fifo*/
void GetWS(void){

	RS_Fifo_write(&WS, WS_Time_Elapsed);
}

/*Takes single reading from both TPH sensors.
 * Combines read bytes and saves to FIFOs.
 */
void GetTPH(void){

	//config device 1:

	uint8_t buffer[2];
	buffer[0] = 0xF2;
	buffer[1] = 0x1;
	HAL_I2C_Master_Transmit(&hi2c1,TPH_1_DEVICE_ADDRESS<<1,buffer,2,100);

	buffer[0] = 0xF4;
	buffer[1] = 0x25;
	HAL_I2C_Master_Transmit(&hi2c1,TPH_1_DEVICE_ADDRESS<<1,buffer,2,100);

	//read out values from device 1:
	HAL_I2C_Mem_Read(&hi2c1,TPH_1_DEVICE_ADDRESS<<1, 0xFE , 1, ph_storage, 1, 100);
	uint8_t hum_lsb_1 = ph_storage[0];
	HAL_I2C_Mem_Read(&hi2c1,TPH_1_DEVICE_ADDRESS<<1, 0xFD , 1, ph_storage, 1, 100);
	uint8_t hum_msb_1 = ph_storage[0];
	HAL_I2C_Mem_Read(&hi2c1,TPH_1_DEVICE_ADDRESS<<1, 0xFC , 1, ph_storage, 1, 100);
	uint8_t temp_xlsb_1 = ph_storage[0];
	HAL_I2C_Mem_Read(&hi2c1,TPH_1_DEVICE_ADDRESS<<1, 0xFB , 1, ph_storage, 1, 100);
	uint8_t temp_lsb_1 = ph_storage[0];
	HAL_I2C_Mem_Read(&hi2c1,TPH_1_DEVICE_ADDRESS<<1, 0xFA , 1, ph_storage, 1, 100);
	uint8_t temp_msb_1 = ph_storage[0];
	HAL_I2C_Mem_Read(&hi2c1,TPH_1_DEVICE_ADDRESS<<1, 0xF9 , 1, ph_storage, 1, 100);
	uint8_t press_xlsb_1 = ph_storage[0];
	HAL_I2C_Mem_Read(&hi2c1,TPH_1_DEVICE_ADDRESS<<1, 0xF8 , 1, ph_storage, 1, 100);
	uint8_t press_lsb_1 = ph_storage[0];
	HAL_I2C_Mem_Read(&hi2c1,TPH_1_DEVICE_ADDRESS<<1, 0xF7 , 1, ph_storage, 1, 100);
	uint8_t press_msb_1 = ph_storage[0];

	//put bytes together for device 1 and write to FIfo.
	int32_t ut_1 = ((temp_xlsb_1| (temp_lsb_1<<4))|(temp_msb_1<<12));
	uint16_t t1_con = T1_converter(ut_1);
	TPH_Fifo_write(&T1,t1_con);
	int32_t up_1 = (press_xlsb_1|(press_lsb_1<<4)|(press_msb_1<<12));
	uint16_t p1_con = P1_converter(up_1);
	TPH_Fifo_write(&P1,p1_con);
	int32_t uh_1 = hum_lsb_1|(hum_msb_1<<8);
	uint16_t h1_con = H1_converter(uh_1);
	TPH_Fifo_write(&H1,h1_con);

	//config device 2:
	buffer[0] = 0xF2;
	buffer[1] = 0x1;
	HAL_I2C_Master_Transmit(&hi2c1,TPH_2_DEVICE_ADDRESS<<1,buffer,2,100);

	buffer[0] = 0xF4;
	buffer[1] = 0x25;
	HAL_I2C_Master_Transmit(&hi2c1,TPH_2_DEVICE_ADDRESS<<1,buffer,2,100);

	//read out values from device 2:
	HAL_I2C_Mem_Read(&hi2c1,TPH_2_DEVICE_ADDRESS<<1, 0xFE , 1, ph_storage, 1, 100);
	uint8_t hum_lsb_2 = ph_storage[0];
	HAL_I2C_Mem_Read(&hi2c1,TPH_2_DEVICE_ADDRESS<<1, 0xFD , 1, ph_storage, 1, 100);
	uint8_t hum_msb_2 = ph_storage[0];
	HAL_I2C_Mem_Read(&hi2c1,TPH_2_DEVICE_ADDRESS<<1, 0xFC , 1, ph_storage, 1, 100);
	uint8_t temp_xlsb_2 = ph_storage[0];
	HAL_I2C_Mem_Read(&hi2c1,TPH_2_DEVICE_ADDRESS<<1, 0xFB , 1, ph_storage, 1, 100);
	uint8_t temp_lsb_2 = ph_storage[0];
	HAL_I2C_Mem_Read(&hi2c1,TPH_2_DEVICE_ADDRESS<<1, 0xFA , 1, ph_storage, 1, 100);
	uint8_t temp_msb_2 = ph_storage[0];
	HAL_I2C_Mem_Read(&hi2c1,TPH_2_DEVICE_ADDRESS<<1, 0xF9 , 1, ph_storage, 1, 100);
	uint8_t press_xlsb_2 = ph_storage[0];
	HAL_I2C_Mem_Read(&hi2c1,TPH_2_DEVICE_ADDRESS<<1, 0xF8 , 1, ph_storage, 1, 100);
	uint8_t press_lsb_2 = ph_storage[0];
	HAL_I2C_Mem_Read(&hi2c1,TPH_2_DEVICE_ADDRESS<<1, 0xF7 , 1, ph_storage, 1, 100);
	uint8_t press_msb_2 = ph_storage[0];

	//put bytes together for device 2 and write to Fifo:
	int32_t ut_2 = ((temp_xlsb_2| (temp_lsb_2<<4))|(temp_msb_2<<12));
	uint16_t t2_con = T2_converter(ut_2);
	TPH_Fifo_write(&T2,t2_con);
	int32_t up_2 = (press_xlsb_2|(press_lsb_2<<4)|(press_msb_2<<12));
	uint16_t p2_con = P2_converter(up_2);
	TPH_Fifo_write(&P2,p2_con);
	int32_t uh_2 = hum_lsb_2|(hum_msb_2<<8);
	uint16_t h2_con = H2_converter(uh_2);
	TPH_Fifo_write(&H2,h2_con);
}

/*Captures Stator Temp Data
 * Saves to ST FIFO
 */
void GetST(void){
	/*Variables used by ST1 and ST2*/
		uint8_t buffer[2];
		uint8_t st_storage[1];

	/*For ST1: Process is repeated below for ST2*/
		/*Write 01 to control register to enter normal mode*/
		buffer[0] = 0x01;
		buffer[1] = 0x01;
		HAL_I2C_Master_Transmit(&hi2c1,ST_1_DEVICE_ADDRESS<<1,buffer,2,100);

		/*Read the TEMP register and store in st_storage*/
		HAL_I2C_Mem_Read(&hi2c1,ST_1_DEVICE_ADDRESS<<1, 0x00 , 1, st_storage, 1, 100);

		/*Write it to Fifo*/
		ST_Fifo_write(&ST1, st_storage[0]);

		/*write 00 to control register to enter standby mode*/
		buffer[0] = 0x01;
		buffer[1] = 0x00;
		HAL_I2C_Master_Transmit(&hi2c1,ST_1_DEVICE_ADDRESS<<1,buffer,2,100);

	/*For ST2*/
		buffer[0] = 0x01;
		buffer[1] = 0x01;
		HAL_I2C_Master_Transmit(&hi2c1,ST_2_DEVICE_ADDRESS<<1,buffer,2,100);

		HAL_I2C_Mem_Read(&hi2c1,ST_2_DEVICE_ADDRESS<<1, 0x00 , 1, st_storage, 1, 100);

		ST_Fifo_write(&ST2, st_storage[0]);

		buffer[0] = 0x01;
		buffer[1] = 0x00;
		HAL_I2C_Master_Transmit(&hi2c1,ST_2_DEVICE_ADDRESS<<1,buffer,2,100);
}

/*Captures data from Rotor Temp sensors
 * Compares the two readings
 * Saves the highest one to the RT FIFO
 */
void GetRT(void){

	/*shared pass through array*/
	uint8_t RT_1_storage[2];
	uint8_t RT_2_storage[2];
	uint16_t raw_rt;

	/*Read object temperature from RT 1:*/
	HAL_I2C_Mem_Read(&hi2c1,RT_1_DEVICE_ADDRESS<<1, 0x007 , 1, RT_1_storage, 2, 100);
	uint16_t raw_rt1= RT_1_storage[0] | (RT_1_storage[1]<<8);

	/*Read object temperature from RT 2:*/
	HAL_I2C_Mem_Read(&hi2c1,RT_2_DEVICE_ADDRESS<<1, 0x007 , 1, RT_2_storage, 2, 100);
	uint16_t raw_rt2 = RT_2_storage[0] | (RT_2_storage[1]<<8);

	/*Choose the hotter one*/
	if(raw_rt1 > raw_rt2){
		raw_rt = raw_rt1;
		}
	else{
		raw_rt = raw_rt2;
	}

	/*convert into units to save*/
	raw_rt = RT_converter(raw_rt);

	/*save it to fifo*/
	RT_Fifo_write(&RT, raw_rt);

	}

/*Triggers Wind Direction capture*/
void GetWD(void){
	HAL_ADC_Start_IT(&hadc1);
}

/*Sets up accel
* See Accel data sheet for setup information*/
void InitAccel(void)
{
	uint8_t buffer[2];
	buffer[0] = 0x20;
	buffer[1] = 0x97;
	HAL_I2C_Master_Transmit(&hi2c1,A_DEVICE_ADDRESS<<1,buffer,2,100);

	buffer[0] = 0x23;
	buffer[1] = 0x8;
	HAL_I2C_Master_Transmit(&hi2c1,A_DEVICE_ADDRESS<<1,buffer,2,100);
}

/*Configures TPH and gets trim variables
 * Consult BME280 datasheet for configuration information*/
void InitTPH(void){

	uint8_t buffer[5];
	buffer[0] = 0xF4;
	buffer[1] = 0x85;
	HAL_I2C_Master_Transmit(&hi2c1,TPH_1_DEVICE_ADDRESS<<1,buffer,2,100);

	buffer[0] = 0xF2;
	buffer[1] = 0x01;
	HAL_I2C_Master_Transmit(&hi2c1,TPH_1_DEVICE_ADDRESS<<1,buffer,2,100);

	//Get temparature trim variables:
	//dig T1:
	HAL_I2C_Mem_Read(&hi2c1,TPH_1_DEVICE_ADDRESS<<1, 0x88 , 1, ph_storage, 1, 100);
	dig_T1_1 = ph_storage[0];
	HAL_I2C_Mem_Read(&hi2c1,TPH_1_DEVICE_ADDRESS<<1, 0x89 , 1, ph_storage, 1, 100);
	dig_T1_1 = dig_T1_1|(ph_storage[0]<<8);

	//dig T2:
	HAL_I2C_Mem_Read(&hi2c1,TPH_1_DEVICE_ADDRESS<<1, 0x8A , 1, ph_storage, 1, 100);
	dig_T2_1 = ph_storage[0];
	HAL_I2C_Mem_Read(&hi2c1,TPH_1_DEVICE_ADDRESS<<1, 0x8B , 1, ph_storage, 1, 100);
	dig_T2_1 = dig_T2_1|(ph_storage[0]<<8);

	//dig T3:
	HAL_I2C_Mem_Read(&hi2c1,TPH_1_DEVICE_ADDRESS<<1, 0x8C , 1, ph_storage, 1, 100);
	dig_T3_1 = ph_storage[0];
	HAL_I2C_Mem_Read(&hi2c1,TPH_1_DEVICE_ADDRESS<<1, 0x8D , 1, ph_storage, 1, 100);
	dig_T3_1 = dig_T3_1|(ph_storage[0]<<8);

	//dig_P1:
	HAL_I2C_Mem_Read(&hi2c1,TPH_1_DEVICE_ADDRESS<<1, 0x8E , 1, ph_storage, 1, 100);
	dig_P1_1 = ph_storage[0];
	HAL_I2C_Mem_Read(&hi2c1,TPH_1_DEVICE_ADDRESS<<1, 0x8F , 1, ph_storage, 1, 100);
	dig_P1_1 = dig_P1_1|(ph_storage[0]<<8);

	//dig_P2:
	HAL_I2C_Mem_Read(&hi2c1,TPH_1_DEVICE_ADDRESS<<1, 0x90 , 1, ph_storage, 1, 100);
	dig_P2_1 = ph_storage[0];
	HAL_I2C_Mem_Read(&hi2c1,TPH_1_DEVICE_ADDRESS<<1, 0x91 , 1, ph_storage, 1, 100);
	dig_P2_1 = dig_P2_1|(ph_storage[0]<<8);

	//dig_P3:
	HAL_I2C_Mem_Read(&hi2c1,TPH_1_DEVICE_ADDRESS<<1, 0x92 , 1, ph_storage, 1, 100);
	dig_P3_1 = ph_storage[0];
	HAL_I2C_Mem_Read(&hi2c1,TPH_1_DEVICE_ADDRESS<<1, 0x93 , 1, ph_storage, 1, 100);
	dig_P3_1 = dig_P3_1|(ph_storage[0]<<8);

	//dig_P4:
	HAL_I2C_Mem_Read(&hi2c1,TPH_1_DEVICE_ADDRESS<<1, 0x94 , 1, ph_storage, 1, 100);
	dig_P4_1 = ph_storage[0];
	HAL_I2C_Mem_Read(&hi2c1,TPH_1_DEVICE_ADDRESS<<1, 0x95 , 1, ph_storage, 1, 100);
	dig_P4_1 = dig_P4_1|(ph_storage[0]<<8);

	//dig_P5:
	HAL_I2C_Mem_Read(&hi2c1,TPH_1_DEVICE_ADDRESS<<1, 0x96 , 1, ph_storage, 1, 100);
	dig_P5_1 = ph_storage[0];
	HAL_I2C_Mem_Read(&hi2c1,TPH_1_DEVICE_ADDRESS<<1, 0x97 , 1, ph_storage, 1, 100);
	dig_P5_1 = dig_P5_1|(ph_storage[0]<<8);

	//dig_P6:
	HAL_I2C_Mem_Read(&hi2c1,TPH_1_DEVICE_ADDRESS<<1, 0x98 , 1, ph_storage, 1, 100);
	dig_P6_1 = ph_storage[0];
	HAL_I2C_Mem_Read(&hi2c1,TPH_1_DEVICE_ADDRESS<<1, 0x99 , 1, ph_storage, 1, 100);
	dig_P6_1 = dig_P6_1|(ph_storage[0]<<8);

	//dig_P7:
	HAL_I2C_Mem_Read(&hi2c1,TPH_1_DEVICE_ADDRESS<<1, 0x9A , 1, ph_storage, 1, 100);
	dig_P7_1 = ph_storage[0];
	HAL_I2C_Mem_Read(&hi2c1,TPH_1_DEVICE_ADDRESS<<1, 0x9B , 1, ph_storage, 1, 100);
	dig_P7_1 = dig_P7_1|(ph_storage[0]<<8);

	//dig_P8:
	HAL_I2C_Mem_Read(&hi2c1,TPH_1_DEVICE_ADDRESS<<1, 0x9C , 1, ph_storage, 1, 100);
	dig_P8_1 = ph_storage[0];
	HAL_I2C_Mem_Read(&hi2c1,TPH_1_DEVICE_ADDRESS<<1, 0x9D , 1, ph_storage, 1, 100);
	dig_P8_1 = dig_P8_1|(ph_storage[0]<<8);

	//dig_H1:
	HAL_I2C_Mem_Read(&hi2c1,TPH_1_DEVICE_ADDRESS<<1, 0xA1 , 1, ph_storage, 1, 100);
	dig_H1_1 = ph_storage[0];

	//dig_H2:
	HAL_I2C_Mem_Read(&hi2c1,TPH_1_DEVICE_ADDRESS<<1, 0xE1 , 1, ph_storage, 1, 100);
	dig_H2_1 = ph_storage[0];
	HAL_I2C_Mem_Read(&hi2c1,TPH_1_DEVICE_ADDRESS<<1, 0xE2 , 1, ph_storage, 1, 100);
	dig_H2_1 = dig_H2_1|(ph_storage[0]<<8);

	//dig_H3:
	HAL_I2C_Mem_Read(&hi2c1,TPH_1_DEVICE_ADDRESS<<1, 0xE3 , 1, ph_storage, 1, 100);
	dig_H3_1 = ph_storage[0];

	//dig_H4:
	HAL_I2C_Mem_Read(&hi2c1,TPH_1_DEVICE_ADDRESS<<1, 0xE5 , 1, ph_storage, 1, 100);
	uint8_t pass_through = (ph_storage[0] & 0xF);
	HAL_I2C_Mem_Read(&hi2c1,TPH_1_DEVICE_ADDRESS<<1, 0xE4 , 1, ph_storage, 1, 100);
	dig_H4_1 = (ph_storage[0]<<4)|pass_through;

	//dig_H5:
	HAL_I2C_Mem_Read(&hi2c1,TPH_1_DEVICE_ADDRESS<<1, 0xE5 , 1, ph_storage, 1, 100);
	pass_through = (ph_storage[0] & 0xF0);
	HAL_I2C_Mem_Read(&hi2c1,TPH_1_DEVICE_ADDRESS<<1, 0xE6, 1, ph_storage, 1, 100);
	dig_H5_1 = (ph_storage[0]<<4)|pass_through;

	//dig_H6:
	HAL_I2C_Mem_Read(&hi2c1,TPH_1_DEVICE_ADDRESS<<1, 0xE7 , 1, ph_storage, 1, 100);
	dig_H6_1 = ph_storage[0];


	buffer[0] = 0xF4;
	buffer[1] = 0x85;
	HAL_I2C_Master_Transmit(&hi2c1,TPH_2_DEVICE_ADDRESS<<1,buffer,2,100);

	buffer[0] = 0xF2;
	buffer[1] = 0x01;
	HAL_I2C_Master_Transmit(&hi2c1,TPH_2_DEVICE_ADDRESS<<1,buffer,2,100);

	//Get temparature trim variables (device 2):
	//dig T1:
	HAL_I2C_Mem_Read(&hi2c1,TPH_2_DEVICE_ADDRESS<<1, 0x88 , 1, ph_storage, 1, 100);
	dig_T1_2 = ph_storage[0];
	HAL_I2C_Mem_Read(&hi2c1,TPH_2_DEVICE_ADDRESS<<1, 0x89 , 1, ph_storage, 1, 100);
	dig_T1_2 = dig_T1_2|(ph_storage[0]<<8);

	//dig T2:
	HAL_I2C_Mem_Read(&hi2c1,TPH_2_DEVICE_ADDRESS<<1, 0x8A , 1, ph_storage, 1, 100);
	dig_T2_2 = ph_storage[0];
	HAL_I2C_Mem_Read(&hi2c1,TPH_2_DEVICE_ADDRESS<<1, 0x8B , 1, ph_storage, 1, 100);
	dig_T2_2 = dig_T2_2|(ph_storage[0]<<8);

	//dig T3:
	HAL_I2C_Mem_Read(&hi2c1,TPH_2_DEVICE_ADDRESS<<1, 0x8C , 1, ph_storage, 1, 100);
	dig_T3_2 = ph_storage[0];
	HAL_I2C_Mem_Read(&hi2c1,TPH_2_DEVICE_ADDRESS<<1, 0x8D , 1, ph_storage, 1, 100);
	dig_T3_2 = dig_T3_2|(ph_storage[0]<<8);

	//dig_P1:
	HAL_I2C_Mem_Read(&hi2c1,TPH_2_DEVICE_ADDRESS<<1, 0x8E , 1, ph_storage, 1, 100);
	dig_P1_2 = ph_storage[0];
	HAL_I2C_Mem_Read(&hi2c1,TPH_2_DEVICE_ADDRESS<<1, 0x8F , 1, ph_storage, 1, 100);
	dig_P1_2 = dig_P1_2|(ph_storage[0]<<8);

	//dig_P2:
	HAL_I2C_Mem_Read(&hi2c1,TPH_2_DEVICE_ADDRESS<<1, 0x90 , 1, ph_storage, 1, 100);
	dig_P2_2 = ph_storage[0];
	HAL_I2C_Mem_Read(&hi2c1,TPH_2_DEVICE_ADDRESS<<1, 0x91 , 1, ph_storage, 1, 100);
	dig_P2_2 = dig_P2_2|(ph_storage[0]<<8);

	//dig_P3:
	HAL_I2C_Mem_Read(&hi2c1,TPH_2_DEVICE_ADDRESS<<1, 0x92 , 1, ph_storage, 1, 100);
	dig_P3_2 = ph_storage[0];
	HAL_I2C_Mem_Read(&hi2c1,TPH_2_DEVICE_ADDRESS<<1, 0x93 , 1, ph_storage, 1, 100);
	dig_P3_2 = dig_P3_2|(ph_storage[0]<<8);

	//dig_P4:
	HAL_I2C_Mem_Read(&hi2c1,TPH_2_DEVICE_ADDRESS<<1, 0x94 , 1, ph_storage, 1, 100);
	dig_P4_2 = ph_storage[0];
	HAL_I2C_Mem_Read(&hi2c1,TPH_2_DEVICE_ADDRESS<<1, 0x95 , 1, ph_storage, 1, 100);
	dig_P4_2 = dig_P4_2|(ph_storage[0]<<8);

	//dig_P5:
	HAL_I2C_Mem_Read(&hi2c1,TPH_2_DEVICE_ADDRESS<<1, 0x96 , 1, ph_storage, 1, 100);
	dig_P5_2 = ph_storage[0];
	HAL_I2C_Mem_Read(&hi2c1,TPH_2_DEVICE_ADDRESS<<1, 0x97 , 1, ph_storage, 1, 100);
	dig_P5_2 = dig_P5_2|(ph_storage[0]<<8);

	//dig_P6:
	HAL_I2C_Mem_Read(&hi2c1,TPH_2_DEVICE_ADDRESS<<1, 0x98 , 1, ph_storage, 1, 100);
	dig_P6_2 = ph_storage[0];
	HAL_I2C_Mem_Read(&hi2c1,TPH_2_DEVICE_ADDRESS<<1, 0x99 , 1, ph_storage, 1, 100);
	dig_P6_2 = dig_P6_2|(ph_storage[0]<<8);

	//dig_P7:
	HAL_I2C_Mem_Read(&hi2c1,TPH_2_DEVICE_ADDRESS<<1, 0x9A , 1, ph_storage, 1, 100);
	dig_P7_2 = ph_storage[0];
	HAL_I2C_Mem_Read(&hi2c1,TPH_2_DEVICE_ADDRESS<<1, 0x9B , 1, ph_storage, 1, 100);
	dig_P7_2 = dig_P7_2|(ph_storage[0]<<8);

	//dig_P8:
	HAL_I2C_Mem_Read(&hi2c1,TPH_2_DEVICE_ADDRESS<<1, 0x9C , 1, ph_storage, 1, 100);
	dig_P8_2 = ph_storage[0];
	HAL_I2C_Mem_Read(&hi2c1,TPH_2_DEVICE_ADDRESS<<1, 0x9D , 1, ph_storage, 1, 100);
	dig_P8_2 = dig_P8_2|(ph_storage[0]<<8);

	//dig_H1:
	HAL_I2C_Mem_Read(&hi2c1,TPH_2_DEVICE_ADDRESS<<1, 0xA1 , 1, ph_storage, 1, 100);
	dig_H1_2 = ph_storage[0];

	//dig_H2:
	HAL_I2C_Mem_Read(&hi2c1,TPH_2_DEVICE_ADDRESS<<1, 0xE1 , 1, ph_storage, 1, 100);
	dig_H2_2 = ph_storage[0];
	HAL_I2C_Mem_Read(&hi2c1,TPH_2_DEVICE_ADDRESS<<1, 0xE2 , 1, ph_storage, 1, 100);
	dig_H2_2 = dig_H2_2|(ph_storage[0]<<8);

	//dig_H3:
	HAL_I2C_Mem_Read(&hi2c1,TPH_2_DEVICE_ADDRESS<<1, 0xE3 , 1, ph_storage, 1, 100);
	dig_H3_2 = ph_storage[0];

	//dig_H4:
	HAL_I2C_Mem_Read(&hi2c1,TPH_2_DEVICE_ADDRESS<<1, 0xE5 , 1, ph_storage, 1, 100);
	pass_through = (ph_storage[0] & 0xF);
	HAL_I2C_Mem_Read(&hi2c1,TPH_2_DEVICE_ADDRESS<<1, 0xE4 , 1, ph_storage, 1, 100);
	dig_H4_2 = (ph_storage[0]<<4)|pass_through;

	//dig_H5:
	HAL_I2C_Mem_Read(&hi2c1,TPH_2_DEVICE_ADDRESS<<1, 0xE5 , 1, ph_storage, 1, 100);
	pass_through = (ph_storage[0] & 0xF0);
	HAL_I2C_Mem_Read(&hi2c1,TPH_2_DEVICE_ADDRESS<<1, 0xE6, 1, ph_storage, 1, 100);
	dig_H5_2 = (ph_storage[0]<<4)|pass_through;

	//dig_H6:
	HAL_I2C_Mem_Read(&hi2c1,TPH_2_DEVICE_ADDRESS<<1, 0xE7 , 1, ph_storage, 1, 100);
	dig_H6_2 = ph_storage[0];
}

/*Converts from raw values to thousandths of Gs + 2000.
 *
 * 0000 = -2.000 Gs
 * 1000 = -1.000 Gs
 * 2000 = 0.000 Gs
 * 3000 = 1.000 Gs
 * 4000 = 2.000 Gs
 *
 */
uint16_t A_converter(uint16_t input){

	float x = input/16780.0;
	x = x - .38379;
	x = x * 1000;
	x = (uint16_t)x;
	return x;
}

/*Erases block of flash starting at given address*/
void EraseFlash(uint32_t BlockAddress){

	uint8_t Block_Erase = 0x50;

	uint8_t Addr1 = BlockAddress >> 16;
	uint8_t Addr2 = BlockAddress >> 8;
	uint8_t Addr3 = BlockAddress;

	//Write CS Pin Low:
  	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_1, GPIO_PIN_RESET);

  	uint8_t TxBuffer[4] = {Block_Erase, Addr1, Addr2, Addr3};

  	HAL_SPI_Transmit(&hspi1,TxBuffer, 4,100);

  	//Write CS Pin High:
  	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_1, GPIO_PIN_SET);

}

/*Writes contents of packet buffer to into flash:
 * 1) Writes whatever is in the packet buffer into the head pointer of the flash buffer.
 * 2) Increments the head pointer.
 * 3) Checks if the head pointer (after increment) is on the first address of a block.
 * 4) Erases one block ahead of pointer if 3 is true.
 * 5) If the tail pointer is in the range to be erased, it is moved to the start of the next block.
 * */
void WriteFlash(struct Flash_Fifo *f){

	/*WRITE THE BUFFER*/
	uint8_t Write_Command = 0x02;

	uint8_t Addr1 = f->head >> 16;
	uint8_t Addr2 = f->head >> 8;
	uint8_t Addr3 = f->head;

	//Write CS Pin Low:
	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_1, GPIO_PIN_RESET);

	uint8_t TxBuffer[18] = {Write_Command, Addr1, Addr2, Addr3, Packet_Buffer[0],Packet_Buffer[1],Packet_Buffer[2],Packet_Buffer[3],Packet_Buffer[4],Packet_Buffer[5],
			Packet_Buffer[6],Packet_Buffer[7],Packet_Buffer[8],Packet_Buffer[9],Packet_Buffer[10],Packet_Buffer[11],Packet_Buffer[12],Packet_Buffer[13]};

	HAL_SPI_Transmit(&hspi1,TxBuffer, 18,100);


	//Write CS Pin High:
	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_1, GPIO_PIN_SET);

	//Need to give the chip a moment tow write following CS Deassert.
	HAL_Delay(1);

	/*INCREMENT THE HEAD POINTER*/

	//Decompose head address into page and byte addresses to handle incrementation:
	head_page_address = f->head >>10;
	head_byte_address = f->head & 0b0000001111111111;

	//The page address ranges from 0 to 4095
	//The byte address ranges from 0 to 518
	//Increment seperately, then re-join into head.

	//If the last starting memory location has not been written to,
	if(head_byte_address < 518 - 14){
		//Increment by the length of one packet.
		head_byte_address = head_byte_address + 14;
		}

	//If the last starting memory location has been written to,
	else{
		//go back down to the first address
		head_byte_address = 0;
		//and go to the next page.

		//if the last written to location was not in the last page,
		if(head_page_address < 4095){
			//increment to the next page.
			head_page_address ++;
			}

		//if the last written to location was on the last page,
		else{
			//go back to the first page.
			head_page_address = 0;
			}

		}
	//Reassemble the head:
	f->head = (head_page_address << 10) | head_byte_address;

	/*ERASE ONE BLOCK AHEAD AND MANAGE TAIL POINTER*/
	//if the new head is on the beginning of a block,
	if(f->head << 19 == 0){

		//Get the page address.
		head_page_address = f->head >>10;

		//if the head pointer (after incrementation) is not at the last block,
				if(head_page_address < 4088){
					//Erase the next block.

					uint16_t page_address_to_erase;
					uint32_t Address_to_erase;

					page_address_to_erase = head_page_address + 8;
					Address_to_erase = (page_address_to_erase << 10);

					//check to see if the tail is in the region to be erased. If it is, move it to the start of the following block.
					tail_page_address = f->tail >>10;

					if((tail_page_address >> 2 ) == (page_address_to_erase >> 2)){


						//Following two statements prevent error that occurs on wrap around
						if(page_address_to_erase == 4088){
							tail_page_address = 0;
							tail_byte_address = 0;
							}

						else{
							tail_page_address = page_address_to_erase + 8;
							tail_byte_address = 0;
							}

						}

					//reconsitute the tail:
					f->tail = (tail_page_address << 10) | tail_byte_address;

					EraseFlash(Address_to_erase);
					}

				//if the head pointer (after incrementation) is at the last block,
				else{
					//Clear the first block:
					uint32_t Address_to_erase = 0;
					uint16_t page_address_to_erase = 0;

					//check to see if the tail is in the region to be erased. If it is, move it to the start of the following block.
					tail_page_address = f->tail >> 10;

					if((tail_page_address >> 2 ) == (page_address_to_erase >> 2)){
						tail_page_address = page_address_to_erase + 8;
						//tail_byte_address = 0;
						}

					//reconstitute the tail:
					f->tail = (tail_page_address << 10) | tail_byte_address;

					EraseFlash(Address_to_erase);
					}

	}
}

/*Reads data out of flash into UART output buffer until there is nothing new in the flash or the buffer is full*/

uint8_t FillUARTBuffer(struct Flash_Fifo *f){

	UART_bytes_in_send = 0;
	uint8_t low_speed_read = 0x01;
	uint8_t rxbuffer[14];

	uint16_t UART_Tx_Buffer_Index = 0;
	while(f->tail != f->head){

		//if the tail has not caught the head and the UART_Tx buffer isn't full, read a packet of data into the UART Tx buffer
		if(UART_Tx_Buffer_Index < UART_TX_BUFFER_LENGTH){

			//Create address bytes out of flash fifo tail:
			uint8_t addr1 = f->tail >> 16;
			uint8_t addr2 = f->tail >> 8;
			uint8_t addr3 = f->tail;

		    uint8_t srbuffer[4] = {low_speed_read, addr1, addr2, addr3};

		  	//Write CS Pin Low:
			HAL_GPIO_WritePin(GPIOH, GPIO_PIN_1, GPIO_PIN_RESET);

		  	HAL_SPI_Transmit(&hspi1, srbuffer,4,100);
		  	HAL_SPI_Receive(&hspi1, rxbuffer, 14, 100);

		  	//Write CS Pin High:
		  	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_1, GPIO_PIN_SET);

		  	//Move bytes from rx buffer into UART_Tx_Buffer:
		  	for(uint8_t i = 0; i <14; i ++){
		  		UART_Tx_Buffer[UART_Tx_Buffer_Index] = rxbuffer[i];
		  		UART_Tx_Backup[UART_Tx_Buffer_Index] = rxbuffer[i];
		  		UART_Tx_Buffer_Index ++;
		  		UART_bytes_in_send ++;
		  	}

		  	//Increment tail:


		  	//Decompose tail address into page and byte addresses to handle incrementation:
		  		tail_page_address = f->tail >>10;
		  		tail_byte_address = f->tail & 0b0000001111111111;


		  		//If the last starting memory location has not been read from,
		  		if(tail_byte_address < 518 - 14){
		  			//Increment by the length of one packet.
		  			tail_byte_address = tail_byte_address + 14;
		  			}

		  		//If the last starting memory location has been read from,
		  		else{
		  			//go back down to the first address
		  			tail_byte_address = 0;
		  			//and go to the next page.

		  			//if the last read location was not in the last page,
		  			if(tail_page_address < 4095){
		  				//increment to the next page.
		  				tail_page_address ++;
		  				}

		  			//if the last read location was on the last page,
		  			else{
		  				//go back to the first page.
		  				tail_page_address = 0;
		  				}

		  			}
		  		//Reassemble the tail:
		  		f->tail = (tail_page_address << 10) | tail_byte_address;

		}

		//If the Tx buffer is full, return 0 to exit function.
		else if(UART_Tx_Buffer_Index >= UART_TX_BUFFER_LENGTH){
			return 0;
			}


	}
	return 0;
}


/*Generates moving average given input, past average, and alpha weighting value.
 * Higher alpha = more smoothing, slower rise time
 */
int16_t dsp_ema_i32(int32_t in, int32_t average, uint16_t alpha){
  if ( alpha == 65535 ) return in;
  int64_t tmp0;
  tmp0 = (int64_t)in * (alpha+1) + average * (65536 - alpha);
  return (int16_t)((tmp0 + 32768) / 65536);
}

/* Callback function for i2c read complete interrupt:
 * Moves data from i2c_storage_buffer into appropriate fifo based on the value of the RxCpltType flag.
 * Resets RxCpltType flag.
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){

switch(RxCpltType){

	case ACCEL_RX_CPLT:
			A_Fifo_write(&AX, ((i2c_storage_buffer[1]<<8 | i2c_storage_buffer[0]) + 40000));
			A_Fifo_write(&AY, ((i2c_storage_buffer[3]<<8 | i2c_storage_buffer[2]) + 40000));
			A_Fifo_write(&AZ, ((i2c_storage_buffer[5]<<8 | i2c_storage_buffer[4]) + 40000));
			RxCpltType = 0;

			break;
	}

}

/*Sets parameters to switch into full run mode*/
void SwitchToFRMode(void){
	  RS_Instance_Switch = 2;
	  Running_Mode = 1;
	  RTC_Wakeup_120Hz_Init();
}

/*Sets parameters to switch into battery mode*/
void SwitchToBATTMode(void){
	  RS_Instance_Switch = 0;
	  Running_Mode = 0;
	  RTC_Wakeup_1Hz_Init();
}


/*Responds to conversion complete interrupt triggered by adc wind direction conversion*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){

	uint8_t WD_storage = HAL_ADC_GetValue(&hadc1);

	WD_Fifo_write(&WD, WD_storage);

	HAL_ADC_Stop(&hadc1);
}

/*Transmits the contents of the backup uart array via UART*/
void send_UART_Backup_Fifo(void){

	HAL_UART_Transmit_IT(&huart2,(uint8_t*)UART_Tx_Backup,UART_bytes_in_send);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line){ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
