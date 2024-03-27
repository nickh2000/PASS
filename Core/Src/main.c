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
#include "math.h"
#include "arm_math.h"
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

I2C_HandleTypeDef hi2c2;

SAI_HandleTypeDef hsai_BlockA1;
SAI_HandleTypeDef hsai_BlockB1;
DMA_HandleTypeDef hdma_sai1_a;
DMA_HandleTypeDef hdma_sai1_b;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SAI1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t fftSize = 1024;
uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;
arm_rfft_instance_q15 fft_instance;
arm_rfft_instance_q15 ifft_instance;
q15_t fft_in_buf[300];
q15_t fft_out_buf[300*2];
q15_t fft_out_buf_mag[300*2];
q15_t ifft_out_buf[300];
float audio_buf_high_f[300];

/* Reference index at which max energy of bin ocuurs */
uint32_t refIndex = 213, testIndex = 0;
/* --------------------------------------
 *
 *
 */
#define AUDIO_LOW_BUF_SIZE 180
#define AUDIO_HIGH_BUF_SIZE 300
#define AUDIO_LOW_BUF_SIZE_HALF 90
#define AUDIO_HIGH_BUF_SIZE_HALF 150
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
uint16_t adc_get[2];
uint32_t sai_fifo_a[16];
uint32_t sai_fifo_b[16];
uint16_t audio_buf_low[AUDIO_LOW_BUF_SIZE];
uint16_t audio_buf_high[AUDIO_HIGH_BUF_SIZE];
uint8_t delay_denom = 8;


static inline int16_t interpolate(uint16_t x, uint8_t delay_n, uint16_t* array, int dir, uint16_t size) {
	return ((array[x % size] * (delay_denom - delay_n)+ array[(x + dir + size) % size] * delay_n) >> 3) - 32768;
}

uint16_t read_ADC_Channel(ADC_HandleTypeDef* hadc, int channel) {
	ADC_ChannelConfTypeDef chConfig = { 0 };
	chConfig.Channel = channel;
	chConfig.Rank = 1;
	chConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	HAL_ADC_ConfigChannel(hadc, &chConfig);
	HAL_ADC_Start(hadc);
	int status = HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
		  if (status != HAL_OK) {
			Error_Handler();
		  }
	uint16_t val = HAL_ADC_GetValue(hadc);
//	HAL_ADC_Stop(hadc);

	return val;
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai) {

	static int circ_offset_low = 0;
	static int circ_offset_high = 0;
	static int delay_nom = 0;
	static int counter = 0;


//	if (counter % 300 == 0) {
//
//
//	for(int i = 0; i < 300; i ++)
//			audio_buf_high_f[i] = audio_buf_high[i];
//	arm_float_to_q15(audio_buf_high_f, fft_in_buf, AUDIO_HIGH_BUF_SIZE);
//
//	arm_rfft_init_q15(&fft_instance, 300, 0, 1);
//	arm_rfft_q15(&fft_instance, fft_in_buf, fft_out_buf);
//
//	arm_cmplx_mag_q15(fft_out_buf + 2, fft_out_buf_mag + 1, 300/2-1);
//
//	arm_rfft_init_q15(&ifft_instance, 300, 1, 1);
//	arm_rfft_q15(&ifft_instance, fft_out_buf, ifft_out_buf);
//	arm_shift_q15(ifft_out_buf, 7, ifft_out_buf, 300);
//	arm_q15_to_float(ifft_out_buf, audio_buf_high_f, 300);
//
//	for(int i = 0; i < 300; i ++)
//		audio_buf_high[i] = audio_buf_high_f[i];
//	}
//
//	counter ++;

	if(hsai == &hsai_BlockB1) return;

	__disable_irq();
	adc_get[0] = read_ADC_Channel(&hadc1, 0);
	adc_get[1] = read_ADC_Channel(&hadc1, 3);
	delay_nom = (read_ADC_Channel(&hadc1, 4) - 2000) * 64 / 2048;



	audio_buf_high[circ_offset_high] = (adc_get[0] << 4);// should be 0
	audio_buf_low[circ_offset_low] = (adc_get[1] << 4);// - (1 << 15);

	circ_offset_high = (circ_offset_high + 1) % AUDIO_HIGH_BUF_SIZE;
	circ_offset_low = (circ_offset_low + 1) % AUDIO_LOW_BUF_SIZE;


	int dir = 1;
	if (delay_nom < 0) dir = -1;
	//Fill the 12 subwoofers
	uint16_t buff_offset = ((AUDIO_LOW_BUF_SIZE >> 1) + AUDIO_LOW_BUF_SIZE + circ_offset_low);
	uint16_t buff_index = buff_offset;
	uint8_t i = 0;
	int int_delay = 4 * delay_nom;
	while (i < 12) {
		buff_index = (int_delay / delay_denom + buff_offset + AUDIO_LOW_BUF_SIZE) % AUDIO_LOW_BUF_SIZE;
		sai_fifo_a[(6 + i) % 12] = interpolate(buff_index,
				(int_delay * dir) % delay_denom,
				audio_buf_low, dir,
				AUDIO_LOW_BUF_SIZE);
		int_delay += delay_nom;
		i ++;
	}
//	Fill the first 4 tweeters into the remaining space in DAC 1
	buff_offset = ((AUDIO_HIGH_BUF_SIZE >> 1) + AUDIO_HIGH_BUF_SIZE + circ_offset_high);
	int_delay = 0;
	while (i < 16) {
		buff_index = (int_delay / delay_denom + buff_offset + AUDIO_HIGH_BUF_SIZE) % AUDIO_HIGH_BUF_SIZE;
		sai_fifo_a[i] = interpolate(buff_index,
				(int_delay * dir) % delay_denom,
				audio_buf_high, dir,
				AUDIO_HIGH_BUF_SIZE);
		int_delay += delay_nom;
		i++;
	}
////////	//Fill the remaining 16 tweeters into DAC 2
	while (i < 32) {
		buff_index = (int_delay / delay_denom + buff_offset + AUDIO_HIGH_BUF_SIZE) % AUDIO_HIGH_BUF_SIZE;
		sai_fifo_b[i - 16] = interpolate(buff_index,
				(int_delay * dir) % delay_denom,
				audio_buf_high, dir,
				AUDIO_HIGH_BUF_SIZE);
		int_delay += delay_nom;
		i++;
	}
	__enable_irq();
////	char msg[100];
////	sprintf(msg, "adc1 is %d; ", adc_get[0]);
////
//	char msg1[100];
//	sprintf(msg1, "adc2 is %d\r\n", delay_nom);
//	HAL_UART_Transmit(&huart3, msg1, strlen((char*)msg1), HAL_MAX_DELAY);
////	HAL_UART_Transmit(&huart3, msg1, strlen((char*)msg1), HAL_MAX_DELAY);


}

#define DAC1_ADDR 0x04
#define DAC2_ADDR 0x24
#define DAC_MUTE1 0x09
#define DAC_MUTE2 0x0A
#define PLL_CLK_CTRL0 0x00
#define DAC_CTRL0 0x06
#define DAC_CTRL1 0x07
#define DAC_CTRL2 0x08


uint8_t mute1_data_DAC1 = 0x00;	//0 is normal operation, 1 is muted
uint8_t mute2_data_DAC1 = 0x00;	//0 is normal operation, 1 is muted
uint8_t mute1_data_DAC2 = 0x00;
uint8_t mute2_data_DAC2 = 0x00;	//0 is normal operation, 1 is muted
uint8_t pll_clk_data = 0b01000001; //assert reset and pllin = 01 to use DLRCLK reference
uint8_t dac_ctrl0 = 0b01100000;
uint8_t dac_ctrl1 = 0b10000100;
uint8_t dac_ctrl2 = 0b10000;



void write_DAC1(uint8_t reg, uint8_t* data) {
	HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(&hi2c2, DAC1_ADDR << 1, reg,
            1, data, 1, HAL_MAX_DELAY);
	char str_fail[100];
	char str_success[100] = "Success on DAC1!\r\n";
	if (ret != HAL_OK) {
		sprintf(str_fail, "Error is %d on DAC1; Register=0x%02X, Data=0x%02x\r\n", HAL_I2C_GetError(&hi2c2), reg, *data);
		HAL_UART_Transmit(&huart3, str_fail, strlen((char*)str_fail), HAL_MAX_DELAY);
	} else {
		HAL_UART_Transmit(&huart3, str_success, strlen((char*)str_success), HAL_MAX_DELAY);
	}
}

void write_DAC2(uint8_t reg, uint8_t* data) {
	HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(&hi2c2, DAC2_ADDR << 1, reg,
            1, data, 1, HAL_MAX_DELAY);
	char str_fail[100];
	char str_success[100] = "Success occured!\r\n";
	if (ret != HAL_OK) {
		sprintf(str_fail, "Error is %d on DAC2; Register=0x%02X, Data=0x%02x\r\n", HAL_I2C_GetError(&hi2c2), reg, *data);
		HAL_UART_Transmit(&huart3, str_fail, strlen((char*)str_fail), HAL_MAX_DELAY);
	} else {
		HAL_UART_Transmit(&huart3, str_success, strlen((char*)str_success), HAL_MAX_DELAY);
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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C2_Init();
  MX_USART3_UART_Init();
  MX_SAI1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  __disable_irq();
//  HAL_ADC_Start_DMA(&hadc1, &adc_get, 2);
  sai_fifo_a[0] = 0x7000U;
  sai_fifo_b[0] = 0x7000U;
  int error;
  error = HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint32_t * )&sai_fifo_a, 16);
  char error_msg[50];
  sprintf(error_msg, "Error is %d on DMA1\r\n", error);
  HAL_UART_Transmit(&huart3, error_msg, strlen((char*)error_msg), HAL_MAX_DELAY);


  error = HAL_SAI_Transmit_DMA(&hsai_BlockB1, (uint32_t * )&sai_fifo_b, 16);
  sprintf(error_msg, "Error is %d on DMA2\r\n", error);
  HAL_UART_Transmit(&huart3, error_msg, strlen((char*)error_msg), HAL_MAX_DELAY);

  //HAL_Delay(1000);



  sprintf(error_msg, "UART good!\r\n");

  HAL_UART_Transmit(&huart3, error_msg, strlen((char*)error_msg), HAL_MAX_DELAY);


  write_DAC1(PLL_CLK_CTRL0, &pll_clk_data);
  write_DAC1(DAC_MUTE1, &mute1_data_DAC1);
  write_DAC1(DAC_MUTE2, &mute2_data_DAC1);
  write_DAC1(DAC_CTRL0, &dac_ctrl0);
  write_DAC1(DAC_CTRL1, &dac_ctrl1);
  write_DAC1(DAC_CTRL2, &dac_ctrl2);

  write_DAC2(PLL_CLK_CTRL0, &pll_clk_data);
  write_DAC2(DAC_MUTE1, &mute1_data_DAC2);
    write_DAC2(DAC_MUTE2, &mute2_data_DAC2);
write_DAC2(DAC_CTRL0, &dac_ctrl0);
write_DAC2(DAC_CTRL1, &dac_ctrl1);
write_DAC2(DAC_CTRL2, &dac_ctrl2);


  __enable_irq();



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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 50;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV2;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI;
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c2.Init.Timing = 0x80102AFF;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI1_Init(void)
{

  /* USER CODE BEGIN SAI1_Init 0 */

  /* USER CODE END SAI1_Init 0 */

  /* USER CODE BEGIN SAI1_Init 1 */

  /* USER CODE END SAI1_Init 1 */
  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockA1.Init.DataSize = SAI_DATASIZE_16;
  hsai_BlockA1.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockA1.Init.ClockStrobing = SAI_CLOCKSTROBING_RISINGEDGE;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_MCKDIV;
  hsai_BlockA1.Init.Mckdiv = 2;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockA1.FrameInit.FrameLength = 256;
  hsai_BlockA1.FrameInit.ActiveFrameLength = 1;
  hsai_BlockA1.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockA1.FrameInit.FSPolarity = SAI_FS_ACTIVE_HIGH;
  hsai_BlockA1.FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;
  hsai_BlockA1.SlotInit.FirstBitOffset = 0;
  hsai_BlockA1.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockA1.SlotInit.SlotNumber = 16;
  hsai_BlockA1.SlotInit.SlotActive = 0x0000FFFF;
  if (HAL_SAI_Init(&hsai_BlockA1) != HAL_OK)
  {
    Error_Handler();
  }
  hsai_BlockB1.Instance = SAI1_Block_B;
  hsai_BlockB1.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockB1.Init.AudioMode = SAI_MODESLAVE_TX;
  hsai_BlockB1.Init.DataSize = SAI_DATASIZE_16;
  hsai_BlockB1.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockB1.Init.ClockStrobing = SAI_CLOCKSTROBING_RISINGEDGE;
  hsai_BlockB1.Init.Synchro = SAI_SYNCHRONOUS;
  hsai_BlockB1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockB1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockB1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockB1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockB1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockB1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockB1.FrameInit.FrameLength = 256;
  hsai_BlockB1.FrameInit.ActiveFrameLength = 1;
  hsai_BlockB1.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockB1.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockB1.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
  hsai_BlockB1.SlotInit.FirstBitOffset = 0;
  hsai_BlockB1.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockB1.SlotInit.SlotNumber = 16;
  hsai_BlockB1.SlotInit.SlotActive = 0x0000FFFF;
  if (HAL_SAI_Init(&hsai_BlockB1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI1_Init 2 */
  /* USER CODE END SAI1_Init 2 */

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
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

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
  char str[100] = "Error handler!\r\n";
  HAL_UART_Transmit(&huart3, str, strlen((char*)str), HAL_MAX_DELAY);

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
