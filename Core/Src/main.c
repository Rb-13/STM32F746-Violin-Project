/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"
#include "stm32746g_discovery_audio.h"
#include "stdio.h"
#include "math.h"
#include <string.h>
#include <stdlib.h>

#include "intro_new.h"
#include "ode_a_la_joie.h"
#include "le_printemps.h"
#include "bella_ciao.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ARBG8888_BYTE_PER_PIXEL   4

/**
 * @brief  SDRAM Write read buffer start address after CAM Frame buffer
 * Assuming Camera frame buffer is of size 640x480 and format RGB565 (16 bits per pixel).
 */
#define SDRAM_WRITE_READ_ADDR        ((uint32_t)(LCD_FB_START_ADDRESS + (RK043FN48H_WIDTH * RK043FN48H_HEIGHT * ARBG8888_BYTE_PER_PIXEL)))

#define SDRAM_WRITE_READ_ADDR_OFFSET ((uint32_t)0x0800)
#define SRAM_WRITE_READ_ADDR_OFFSET  SDRAM_WRITE_READ_ADDR_OFFSET

#define AUDIO_REC_START_ADDR         SDRAM_WRITE_READ_ADDR

#define AUDIO_BLOCK_SIZE   	((uint32_t)512)
#define AUDIO_BUFFER_IN    	AUDIO_REC_START_ADDR     /* In SDRAM */
#define AUDIO_BUFFER_OUT   	(AUDIO_REC_START_ADDR + (AUDIO_BLOCK_SIZE*2)) /* In SDRAM */
#define AUDIO_BUFFER_READ  	(AUDIO_REC_START_ADDR + (AUDIO_BLOCK_SIZE*4))
#define AUDIO_BUFFER_POST  	(AUDIO_REC_START_ADDR + (AUDIO_BLOCK_SIZE*6))

#define Audio_freq 			AUDIO_FREQUENCY_22K
#define Audio_bit_res 		DEFAULT_AUDIO_IN_BIT_RESOLUTION	//16
#define Audio_chan 			DEFAULT_AUDIO_IN_CHANNEL_NBR	//2
#define BytePerBloc			((uint16_t)Audio_bit_res*Audio_chan/8)
#define BytePerSec			((uint32_t)BytePerBloc*Audio_freq)

#define MASK_32_TO_8_0		0x000000FF
#define MASK_32_TO_8_1		0x0000FF00
#define MASK_32_TO_8_2		0x00FF0000
#define MASK_32_TO_8_3		0xFF000000

#define REC_BUTTON_X 		BSP_LCD_GetXSize() / 3
#define REC_BUTTON_Y 		BSP_LCD_GetYSize() / 2-40
#define PLAY_BUTTON_X 		2 * BSP_LCD_GetXSize() / 3
#define PLAY_BUTTON_Y 		REC_BUTTON_Y
#define OFF_X				BSP_LCD_GetXSize() - 30
#define OFF_Y				BSP_LCD_GetYSize() - 30
#define ON_X				BSP_LCD_GetXSize() / 2
#define ON_Y				BSP_LCD_GetYSize() / 2
#define R					40


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

DMA2D_HandleTypeDef hdma2d;

LTDC_HandleTypeDef hltdc;

SAI_HandleTypeDef hsai_BlockA2;
SAI_HandleTypeDef hsai_BlockB2;
DMA_HandleTypeDef hdma_sai2_a;
DMA_HandleTypeDef hdma_sai2_b;

SD_HandleTypeDef hsd1;
DMA_HandleTypeDef hdma_sdmmc1_rx;
DMA_HandleTypeDef hdma_sdmmc1_tx;

UART_HandleTypeDef huart1;

SDRAM_HandleTypeDef hsdram1;
SDRAM_HandleTypeDef hsdram2;

osThreadId defaultTaskHandle;
osThreadId SDHandle;
osThreadId RecordHandle;
osMessageQId WakeUpHandle;
/* USER CODE BEGIN PV */
uint8_t workBuffer[2 * _MAX_SS], enable = 0, fin_record = 0,etat = 0;
uint32_t NB_Bloc=0,Bloc_Cursor=0;
unsigned char text[32];
uint8_t record_OK=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_FMC_Init(void);
static void MX_LTDC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DMA2D_Init(void);
static void MX_SAI2_Init(void);
void StartDefaultTask(void const * argument);
void StartSD(void const * argument);
void StartRecord(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t InCircle(Point xy, Point Circle, uint8_t Radius) {
	return ((xy.X - Circle.X) * (xy.X - Circle.X)
			+ (xy.Y - Circle.Y) * (xy.Y - Circle.Y)) <= Radius*Radius;
}
void Addition(char Offset){
	if (Offset){
		for (uint16_t i = 0;i<AUDIO_BLOCK_SIZE/2;i++){
			((int16_t*)AUDIO_BUFFER_OUT + AUDIO_BLOCK_SIZE/2)[i] = (((int16_t*)AUDIO_BUFFER_IN + AUDIO_BLOCK_SIZE/2)[i]+((int16_t*)AUDIO_BUFFER_READ + AUDIO_BLOCK_SIZE/2)[i])/2;
		}
	}else{
		for (uint16_t i = 0;i<AUDIO_BLOCK_SIZE/2;i++){
			((int16_t*)AUDIO_BUFFER_OUT)[i] = (((int16_t*)AUDIO_BUFFER_IN)[i]+((int16_t*)AUDIO_BUFFER_READ)[i])/2;
		}
	}
}
void DrawPlayButton(uint32_t Color){
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_SetTextColor(Color);
	BSP_LCD_FillCircle( PLAY_BUTTON_X, PLAY_BUTTON_Y, R);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_SetBackColor(Color);
	BSP_LCD_DisplayChar(PLAY_BUTTON_X-6, PLAY_BUTTON_Y-11, (char)'P');
}
void DrawRecButton(uint32_t Color){
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_SetTextColor(Color);
	BSP_LCD_FillCircle( REC_BUTTON_X, REC_BUTTON_Y, R);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_SetBackColor(Color);
	BSP_LCD_DisplayChar(REC_BUTTON_X-6, REC_BUTTON_Y-11,(char)'R');
}
void DrawONButton(){
	BSP_LCD_Clear(LCD_COLOR_BLACK);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_FillCircle(ON_X, ON_Y, R);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_DisplayChar(ON_X-14, ON_Y-11,(char)'O');
	BSP_LCD_DisplayChar(ON_X+1, ON_Y-11,(char)'N');
}
void DrawOFFutton(){
	BSP_LCD_Clear(LCD_COLOR_BLACK);
	BSP_LCD_SetTextColor(LCD_COLOR_ORANGE);
	BSP_LCD_FillCircle(OFF_X, OFF_Y, R/2);
	BSP_LCD_SetBackColor(LCD_COLOR_ORANGE);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_SetFont(&Font16);
	BSP_LCD_DisplayChar(OFF_X-14, OFF_Y-7,(char)'O');
	BSP_LCD_DisplayChar(OFF_X-4, OFF_Y-7,(char)'F');
	BSP_LCD_DisplayChar(OFF_X+6, OFF_Y-7,(char)'F');
}

void DrawSOONButton(){
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHTRED);
	BSP_LCD_FillRect(330,155,125,110);
	BSP_LCD_SetBackColor(LCD_COLOR_LIGHTRED);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_DisplayChar(335,180,(char)'C');
	BSP_LCD_DisplayChar(350,180,(char)'O');
	BSP_LCD_DisplayChar(365,180,(char)'M');
	BSP_LCD_DisplayChar(380,180,(char)'I');
	BSP_LCD_DisplayChar(395,180,(char)'N');
	BSP_LCD_DisplayChar(410,180,(char)'G');
	BSP_LCD_DisplayChar(380,210,(char)'S');
	BSP_LCD_DisplayChar(395,210,(char)'O');
	BSP_LCD_DisplayChar(410,210,(char)'O');
	BSP_LCD_DisplayChar(425,210,(char)'N');
}

void SD_Init() {
	uint16_t rtext[_MAX_SS];/* File read buffer */
	FRESULT Res;
	if (f_mount(&SDFatFS, (TCHAR const*) SDPath, 0) != FR_OK) {
		Error_Handler();
	} else {
		BSP_LCD_DisplayStringAt(0, 40, (uint8_t*) "SD - Mount Ok", CENTER_MODE);
//		Res = f_mkfs((TCHAR const*) SDPath, FM_ANY, 0, rtext, sizeof(rtext));
//		if (Res != FR_OK) {
//			Error_Handler();
//		} else {
//			BSP_LCD_DisplayStringAt(0, 50, (uint8_t*) "SD - Formatage Ok",
//					CENTER_MODE);
//			//Open file for writing (Create)
//		}
	}
}
void Audio_Init() {
	if (BSP_AUDIO_IN_OUT_Init(INPUT_DEVICE_INPUT_LINE_1,
	OUTPUT_DEVICE_HEADPHONE, Audio_freq,
	Audio_bit_res,
	Audio_chan) == AUDIO_OK) {
		BSP_LCD_DisplayStringAt(0, 20, (uint8_t*) "Init Audio - OK",
				CENTER_MODE);
	}

	/* Initialize SDRAM buffers */
	memset((uint16_t*) AUDIO_BUFFER_IN, 0, AUDIO_BLOCK_SIZE * 2);
	memset((uint16_t*) AUDIO_BUFFER_OUT, 0, AUDIO_BLOCK_SIZE * 2);
	memset((uint16_t*) AUDIO_BUFFER_READ, 0, AUDIO_BLOCK_SIZE * 2);

	/* Start Recording */
	BSP_AUDIO_IN_Record((uint16_t*) AUDIO_BUFFER_IN, AUDIO_BLOCK_SIZE);
	BSP_AUDIO_IN_SetVolume(191);
	BSP_AUDIO_OUT_SetVolume(75);
	/* Start Playback */
	BSP_AUDIO_OUT_SetAudioFrameSlot(CODEC_AUDIOFRAME_SLOT_02);
	if (BSP_AUDIO_OUT_Play((uint16_t*) AUDIO_BUFFER_OUT,
	AUDIO_BLOCK_SIZE * 2) == AUDIO_OK) {
		BSP_LCD_DisplayStringAt(0, 30, (uint8_t*) "Play Audio - OK",
				CENTER_MODE);
	}

}
void LCD_Init() {
	BSP_LCD_Init();
	BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
	BSP_LCD_LayerDefaultInit(1,
	LCD_FB_START_ADDRESS + BSP_LCD_GetXSize() * BSP_LCD_GetYSize() * 4);
	BSP_LCD_DisplayOn();
	BSP_LCD_SelectLayer(1);
	BSP_LCD_Clear(LCD_COLOR_BLACK);
	BSP_LCD_SetFont(&Font12);
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHTBLUE);
	if (BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize()) != TS_OK) {
		BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
		BSP_LCD_SetTextColor(LCD_COLOR_RED);
		BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 95, (uint8_t*) "ERROR",
				CENTER_MODE);
		BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 80,
				(uint8_t*) "Touchscreen cannot be initialized", CENTER_MODE);
	} else {
		BSP_LCD_DisplayStringAt(0, 10, (uint8_t*) "Init Ecran - OK",
				CENTER_MODE);
	}
}
void write_header(uint32_t N_Bytes_Data) {
	uint32_t byteswritten;
	uint8_t entete[] = {
			0x52, 0x49, 0x46,0x46, //'RIFF' : identification du format
			(uint8_t) ((N_Bytes_Data + 36) & MASK_32_TO_8_0),
			(uint8_t) (((N_Bytes_Data + 36) & MASK_32_TO_8_1) >> 8),
			(uint8_t) (((N_Bytes_Data + 36) & MASK_32_TO_8_2) >> 16),
			(uint8_t) (((N_Bytes_Data + 36) & MASK_32_TO_8_3) >> 24), //Taille du fichier - 8 octets (en octets)

			0x57, 0x41, 0x56,0x45, //'WAVE'
			0x66, 0x6D, 0x74,0x20, //'fmt ' identifiant le format WAV
			0x10, 0x00, 0x00,0x00, //Nombre d'octets utilisés pour définir le format (16)

			0x01,0x00, //Pas de compression, format PCM entier
			Audio_chan,0x00, //Nombre de cannaux
			(uint8_t) ( Audio_freq & MASK_32_TO_8_0),
			(uint8_t) ((Audio_freq & MASK_32_TO_8_1) >> 8),
			(uint8_t) ((Audio_freq & MASK_32_TO_8_2) >> 16),
			(uint8_t) ((Audio_freq & MASK_32_TO_8_3) >> 24), //Fréquence d'échantillonnage

			(uint8_t) ( BytePerSec & MASK_32_TO_8_0),
			(uint8_t) ((BytePerSec & MASK_32_TO_8_1) >> 8),
			(uint8_t) ((BytePerSec & MASK_32_TO_8_2) >> 16),
			(uint8_t) ((BytePerSec & MASK_32_TO_8_3) >> 24), //Nombre d'octets par seconde

			(uint8_t)( BytePerBloc & MASK_32_TO_8_0),
			(uint8_t)((BytePerBloc & MASK_32_TO_8_1) >> 8), //BytePerBloc

			(Audio_bit_res),0x00, //Bits par échantillons
			0x64, 0x61, 0x74,0x61, //'DATA'
			(uint8_t) ( (N_Bytes_Data) & MASK_32_TO_8_0),
			(uint8_t) (((N_Bytes_Data) & MASK_32_TO_8_1) >> 8),
			(uint8_t) (((N_Bytes_Data) & MASK_32_TO_8_2) >> 16),
			(uint8_t) (((N_Bytes_Data) & MASK_32_TO_8_3) >> 24),	//DataSize
			};
	//f_lseek(&SDFile, 0);
	//f_write(&SDFile, entete, 44, (void*) &byteswritten);
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_DisplayStringAt(0, 190, (uint8_t*) "En tete - OK", CENTER_MODE);
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

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  MX_FMC_Init();
  MX_LTDC_Init();
  MX_USART1_UART_Init();
  MX_DMA2D_Init();
  MX_SAI2_Init();
  /* USER CODE BEGIN 2 */
	LCD_Init();
	Audio_Init();
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

  /* Create the queue(s) */
  /* definition and creation of WakeUp */
  osMessageQDef(WakeUp, 1, uint8_t);
  WakeUpHandle = osMessageCreate(osMessageQ(WakeUp), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of SD */
  osThreadDef(SD, StartSD, osPriorityHigh, 0, 512);
  SDHandle = osThreadCreate(osThread(SD), NULL);

  /* definition and creation of Record */
  osThreadDef(Record, StartRecord, osPriorityHigh, 0, 256);
  RecordHandle = osThreadCreate(osThread(Record), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_SAI2
                              |RCC_PERIPHCLK_SDMMC1|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 5;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_8;
  PeriphClkInitStruct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLSAI;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
  PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 40;
  hltdc.Init.VerticalSync = 9;
  hltdc.Init.AccumulatedHBP = 53;
  hltdc.Init.AccumulatedVBP = 11;
  hltdc.Init.AccumulatedActiveW = 533;
  hltdc.Init.AccumulatedActiveH = 283;
  hltdc.Init.TotalWidth = 565;
  hltdc.Init.TotalHeigh = 285;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 480;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 272;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  pLayerCfg.FBStartAdress = 0xC0000000;
  pLayerCfg.ImageWidth = 480;
  pLayerCfg.ImageHeight = 272;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief SAI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI2_Init(void)
{

  /* USER CODE BEGIN SAI2_Init 0 */

  /* USER CODE END SAI2_Init 0 */

  /* USER CODE BEGIN SAI2_Init 1 */

  /* USER CODE END SAI2_Init 1 */
  hsai_BlockA2.Instance = SAI2_Block_A;
  hsai_BlockA2.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockA2.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockA2.Init.DataSize = SAI_DATASIZE_8;
  hsai_BlockA2.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockA2.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockA2.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA2.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA2.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA2.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_48K;
  hsai_BlockA2.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA2.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA2.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA2.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockA2.FrameInit.FrameLength = 8;
  hsai_BlockA2.FrameInit.ActiveFrameLength = 1;
  hsai_BlockA2.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockA2.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockA2.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
  hsai_BlockA2.SlotInit.FirstBitOffset = 0;
  hsai_BlockA2.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockA2.SlotInit.SlotNumber = 1;
  hsai_BlockA2.SlotInit.SlotActive = 0x00000000;
  if (HAL_SAI_Init(&hsai_BlockA2) != HAL_OK)
  {
    Error_Handler();
  }
  hsai_BlockB2.Instance = SAI2_Block_B;
  hsai_BlockB2.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockB2.Init.AudioMode = SAI_MODESLAVE_RX;
  hsai_BlockB2.Init.DataSize = SAI_DATASIZE_8;
  hsai_BlockB2.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockB2.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockB2.Init.Synchro = SAI_SYNCHRONOUS;
  hsai_BlockB2.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockB2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockB2.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockB2.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockB2.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockB2.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockB2.FrameInit.FrameLength = 8;
  hsai_BlockB2.FrameInit.ActiveFrameLength = 1;
  hsai_BlockB2.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockB2.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockB2.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
  hsai_BlockB2.SlotInit.FirstBitOffset = 0;
  hsai_BlockB2.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockB2.SlotInit.SlotNumber = 1;
  hsai_BlockB2.SlotInit.SlotActive = 0x00000000;
  if (HAL_SAI_Init(&hsai_BlockB2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI2_Init 2 */

  /* USER CODE END SAI2_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK1;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_1;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_DISABLE;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 16;
  SdramTiming.ExitSelfRefreshDelay = 16;
  SdramTiming.SelfRefreshTime = 16;
  SdramTiming.RowCycleDelay = 16;
  SdramTiming.WriteRecoveryTime = 16;
  SdramTiming.RPDelay = 16;
  SdramTiming.RCDDelay = 16;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /** Perform the SDRAM2 memory initialization sequence
  */
  hsdram2.Instance = FMC_SDRAM_DEVICE;
  /* hsdram2.Init */
  hsdram2.Init.SDBank = FMC_SDRAM_BANK2;
  hsdram2.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram2.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram2.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram2.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram2.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_1;
  hsdram2.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram2.Init.SDClockPeriod = FMC_SDRAM_CLOCK_DISABLE;
  hsdram2.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
  hsdram2.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 16;
  SdramTiming.ExitSelfRefreshDelay = 16;
  SdramTiming.SelfRefreshTime = 16;
  SdramTiming.RowCycleDelay = 16;
  SdramTiming.WriteRecoveryTime = 16;
  SdramTiming.RPDelay = 16;
  SdramTiming.RCDDelay = 16;

  if (HAL_SDRAM_Init(&hsdram2, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
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
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, ARDUINO_D7_Pin|ARDUINO_D8_Pin|LCD_DISP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, DCMI_PWR_EN_Pin|LED2_Pin|LED1_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, ARDUINO_D4_Pin|ARDUINO_D2_Pin|EXT_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : OTG_HS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_HS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_HS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : QSPI_D2_Pin */
  GPIO_InitStruct.Pin = QSPI_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(QSPI_D2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TXD1_Pin RMII_TXD0_Pin RMII_TX_EN_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin|RMII_TXD0_Pin|RMII_TX_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_SCL_D15_Pin ARDUINO_SDA_D14_Pin */
  GPIO_InitStruct.Pin = ARDUINO_SCL_D15_Pin|ARDUINO_SDA_D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_PWM_D3_Pin */
  GPIO_InitStruct.Pin = ARDUINO_PWM_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(ARDUINO_PWM_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPDIF_RX0_Pin */
  GPIO_InitStruct.Pin = SPDIF_RX0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF8_SPDIFRX;
  HAL_GPIO_Init(SPDIF_RX0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_PWM_D9_Pin */
  GPIO_InitStruct.Pin = ARDUINO_PWM_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(ARDUINO_PWM_D9_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DCMI_D6_Pin DCMI_D7_Pin */
  GPIO_InitStruct.Pin = DCMI_D6_Pin|DCMI_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_VBUS_Pin */
  GPIO_InitStruct.Pin = OTG_FS_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Audio_INT_Pin */
  GPIO_InitStruct.Pin = Audio_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Audio_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_P_Pin OTG_FS_N_Pin OTG_FS_ID_Pin */
  GPIO_InitStruct.Pin = OTG_FS_P_Pin|OTG_FS_N_Pin|OTG_FS_ID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DCMI_D5_Pin */
  GPIO_InitStruct.Pin = DCMI_D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(DCMI_D5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_D7_Pin ARDUINO_D8_Pin LCD_DISP_Pin */
  GPIO_InitStruct.Pin = ARDUINO_D7_Pin|ARDUINO_D8_Pin|LCD_DISP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : uSD_Detect_Pin */
  GPIO_InitStruct.Pin = uSD_Detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(uSD_Detect_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_BL_CTRL_Pin */
  GPIO_InitStruct.Pin = LCD_BL_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_BL_CTRL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DCMI_VSYNC_Pin */
  GPIO_InitStruct.Pin = DCMI_VSYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(DCMI_VSYNC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TP3_Pin NC2_Pin */
  GPIO_InitStruct.Pin = TP3_Pin|NC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_SCK_D13_Pin */
  GPIO_InitStruct.Pin = ARDUINO_SCK_D13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(ARDUINO_SCK_D13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DCMI_PWR_EN_Pin LED2_Pin LED1_Pin LED3_Pin */
  GPIO_InitStruct.Pin = DCMI_PWR_EN_Pin|LED2_Pin|LED1_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : DCMI_D4_Pin DCMI_D0_Pin */
  GPIO_InitStruct.Pin = DCMI_D4_Pin|DCMI_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_PWM_CS_D5_Pin */
  GPIO_InitStruct.Pin = ARDUINO_PWM_CS_D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
  HAL_GPIO_Init(ARDUINO_PWM_CS_D5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_PWM_D10_Pin */
  GPIO_InitStruct.Pin = ARDUINO_PWM_D10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(ARDUINO_PWM_D10_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_INT_Pin */
  GPIO_InitStruct.Pin = LCD_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LCD_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_RX_D0_Pin ARDUINO_TX_D1_Pin */
  GPIO_InitStruct.Pin = ARDUINO_RX_D0_Pin|ARDUINO_TX_D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ULPI_NXT_Pin */
  GPIO_InitStruct.Pin = ULPI_NXT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(ULPI_NXT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_D4_Pin ARDUINO_D2_Pin EXT_RST_Pin */
  GPIO_InitStruct.Pin = ARDUINO_D4_Pin|ARDUINO_D2_Pin|EXT_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_A4_Pin ARDUINO_A5_Pin ARDUINO_A1_Pin ARDUINO_A2_Pin
                           ARDUINO_A3_Pin */
  GPIO_InitStruct.Pin = ARDUINO_A4_Pin|ARDUINO_A5_Pin|ARDUINO_A1_Pin|ARDUINO_A2_Pin
                          |ARDUINO_A3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_D6_Pin ULPI_D5_Pin ULPI_D3_Pin ULPI_D2_Pin
                           ULPI_D1_Pin ULPI_D4_Pin */
  GPIO_InitStruct.Pin = ULPI_D6_Pin|ULPI_D5_Pin|ULPI_D3_Pin|ULPI_D2_Pin
                          |ULPI_D1_Pin|ULPI_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_STP_Pin ULPI_DIR_Pin */
  GPIO_InitStruct.Pin = ULPI_STP_Pin|ULPI_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : QSPI_D1_Pin QSPI_D3_Pin QSPI_D0_Pin */
  GPIO_InitStruct.Pin = QSPI_D1_Pin|QSPI_D3_Pin|QSPI_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_RXER_Pin */
  GPIO_InitStruct.Pin = RMII_RXER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RMII_RXER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DCMI_HSYNC_Pin PA6 */
  GPIO_InitStruct.Pin = DCMI_HSYNC_Pin|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_CLK_Pin ULPI_D0_Pin */
  GPIO_InitStruct.Pin = ULPI_CLK_Pin|ULPI_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_SDA_Pin */
  GPIO_InitStruct.Pin = LCD_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(LCD_SDA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_MISO_D12_Pin ARDUINO_MOSI_PWM_D11_Pin */
  GPIO_InitStruct.Pin = ARDUINO_MISO_D12_Pin|ARDUINO_MOSI_PWM_D11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void BSP_AUDIO_IN_TransferComplete_CallBack(void) {
	char a=1,b=3;
	if (enable) {
		switch(etat){
			//HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		case 1:
			memcpy((uint16_t*) (AUDIO_BUFFER_OUT + AUDIO_BLOCK_SIZE),
				(uint16_t*) (AUDIO_BUFFER_IN + AUDIO_BLOCK_SIZE),
				AUDIO_BLOCK_SIZE);
			break;
		case 2:
			memcpy((uint16_t*) (AUDIO_BUFFER_OUT + AUDIO_BLOCK_SIZE),
				(uint16_t*) (AUDIO_BUFFER_IN + AUDIO_BLOCK_SIZE),
				AUDIO_BLOCK_SIZE);
			xQueueSendFromISR(WakeUpHandle,&a,0);
			break;
		case 3:
			xQueueSendFromISR(WakeUpHandle,&b,0);
			break;
		}
	}
	return;
}

void BSP_AUDIO_IN_HalfTransfer_CallBack(void) {
	char a=0,b=2;
	if (enable) {
		switch(etat){
			//HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		case 1:
			memcpy((uint16_t*) (AUDIO_BUFFER_OUT),
				(uint16_t*) (AUDIO_BUFFER_IN), AUDIO_BLOCK_SIZE);
			break;
		case 2:
			memcpy((uint16_t*) (AUDIO_BUFFER_OUT),
				(uint16_t*) (AUDIO_BUFFER_IN), AUDIO_BLOCK_SIZE);
			xQueueSendFromISR(WakeUpHandle,&a,0);
			break;
		case 3:
			xQueueSendFromISR(WakeUpHandle,&b,0);
			break;
		}
	}
	return;
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
	static TS_StateTypeDef TS_State;
	Point REC_BUTTON,ON_BUTTON,OFF_BUTTON,PLAY_BUTTON,Touch;
	//uint8_t record_OK;
	REC_BUTTON.X = REC_BUTTON_X;
	REC_BUTTON.Y = REC_BUTTON_Y;
	ON_BUTTON.X = ON_X;
	ON_BUTTON.Y = ON_Y;
	OFF_BUTTON.X = OFF_X;
	OFF_BUTTON.Y = OFF_Y;
	PLAY_BUTTON.X = PLAY_BUTTON_X;
	PLAY_BUTTON.Y = PLAY_BUTTON_Y;
	while (enable==0);
	osDelay(500);
	BSP_LCD_DrawBitmap(0,0,(uint8_t*)intro_new_bmp);

	int musique =1 ;

	for (;;) {
		BSP_TS_GetState(&TS_State);
		if (TS_State.touchDetected) {
			Touch.X = TS_State.touchX[0];
			Touch.Y = TS_State.touchY[0];
		}else{
			Touch.X = 0;
			Touch.Y = 0;
		}
		switch (etat) {
			case 0:
				record_OK=0;
				if ((Touch.Y>120) && (Touch.Y<272) && (Touch.X>10) && ((Touch.X<230))){
					NB_Bloc=3312;
					etat = 101; //Ode à la joie

				}

				else if ((Touch.Y>120) && (Touch.Y<272) && (Touch.X>240) && ((Touch.X<470))){
					NB_Bloc=2181;
					etat = 102; //Bella Ciao

				}

				break;

			/////////////////// IMAGE 1 - ODE A LA JOIE
			case 101:
				BSP_LCD_DrawBitmap(0,0,(uint8_t*)ode_a_la_joie_bmp);
				if (TS_State.touchDetected==0){
					f_open(&SDFile, "ODE.WAV", FA_READ);
					musique=1;
					etat=1;
				}
				break;

			case 1:
				if ((Touch.Y>250) && (Touch.Y<272) && (Touch.X>445) && ((Touch.X<480))){ //Back to menu
					etat=10;
					BSP_LCD_DrawBitmap(0,0,(uint8_t*)intro_new_bmp); //Retour intro

				}
				if ((Touch.Y>250) && (Touch.Y<272) && (Touch.X>350) && ((Touch.X<444))){ //Read sample from SD file

					f_close(&SDFile);
					if (musique==1){
						NB_Bloc=3312;
						f_open(&SDFile, "ODE.WAV", FA_READ);//Sample wav file
					}
					else{
						NB_Bloc=2181;
						f_open(&SDFile, "BELLA.WAV", FA_READ);//Sample wav file
					}

					etat=13;
				}

				//Musical notes exisiting in a true violin

				if ((Touch.Y>154) && (Touch.Y<188) && (Touch.X>80) && ((Touch.X<123))){
					NB_Bloc=44;
					f_close(&SDFile);
					f_open(&SDFile, "1-4SOL.WAV", FA_READ);
					etat=13;
				}

				if ((Touch.Y>160) && (Touch.Y<192) && (Touch.X>157) && ((Touch.X<198))){
					NB_Bloc=41;
					f_close(&SDFile);
					f_open(&SDFile, "2-4LA.WAV", FA_READ);
					etat=13;
				}

				if ((Touch.Y>160) && (Touch.Y<192) && (Touch.X>234) && ((Touch.X<272))){
					NB_Bloc=43;
					f_close(&SDFile);
					f_open(&SDFile, "3-4SI.WAV", FA_READ);
					etat=13;
				}

				if ((Touch.Y>160) && (Touch.Y<192) && (Touch.X>273) && ((Touch.X<312))){
					NB_Bloc=35;
					f_close(&SDFile);
					f_open(&SDFile, "4-4DO.WAV", FA_READ);
					etat=13;
				}

				if ((Touch.Y>126) && (Touch.Y<154) && (Touch.X>80) && ((Touch.X<123))) {
					NB_Bloc=45;
					f_close(&SDFile);
					f_open(&SDFile, "5-34RE.WAV", FA_READ);
					etat=13;
				}

				if ((Touch.Y>161) && (Touch.Y<193) && (Touch.X>343) && ((Touch.X<380))) {
					NB_Bloc=45;
					f_close(&SDFile);
					f_open(&SDFile, "5-34RE.WAV", FA_READ);
					etat=13;
				}

				if ((Touch.Y>126) && (Touch.Y<159) && (Touch.X>157) && ((Touch.X<198))){
					NB_Bloc=43;
					f_close(&SDFile);
					f_open(&SDFile, "6-3MI.WAV", FA_READ);
					etat=13;
				}

				if ((Touch.Y>129) && (Touch.Y<161) && (Touch.X>199) && ((Touch.X<237))){
					NB_Bloc=44;
					f_close(&SDFile);
					f_open(&SDFile, "7-3FA.WAV", FA_READ);
					etat=13;
				}

				if ((Touch.Y>130) && (Touch.Y<162) && (Touch.X>273) && ((Touch.X<312))){
					NB_Bloc=45;
					f_close(&SDFile);
					f_open(&SDFile, "8-3SOL.WAV", FA_READ);
					etat=13;
				}

				if ((Touch.Y>126) && (Touch.Y<160) && (Touch.X>343) && ((Touch.X<380))){
					NB_Bloc=43;
					f_close(&SDFile);
					f_open(&SDFile, "9-23LA.WAV", FA_READ);
					etat=13;
				}

				if ((Touch.Y>96) && (Touch.Y<126) && (Touch.X>80) && ((Touch.X<123))){
					NB_Bloc=43;
					f_close(&SDFile);
					f_open(&SDFile, "9-23LA.WAV", FA_READ);
					etat=13;
				}

				if ((Touch.Y>95) && (Touch.Y<127) && (Touch.X>157) && ((Touch.X<198))){
					NB_Bloc=44;
					f_close(&SDFile);
					f_open(&SDFile, "10-2SI.WAV", FA_READ);
					etat=13;
				}

				if ((Touch.Y>98) && (Touch.Y<131) && (Touch.X>199) && ((Touch.X<237))){
					NB_Bloc=44;
					f_close(&SDFile);
					f_open(&SDFile, "11-2DO.WAV", FA_READ);
					etat=13;
				}

				if ((Touch.Y>98) && (Touch.Y<130) && (Touch.X>273) && ((Touch.X<312))){
					NB_Bloc=45;
					f_close(&SDFile);
					f_open(&SDFile, "12-2RE.WAV", FA_READ);
					etat=13;
				}

				if ((Touch.Y>66) && (Touch.Y<96) && (Touch.X>80) && ((Touch.X<123))){
					NB_Bloc=48;
					f_close(&SDFile);
					f_open(&SDFile, "13-12MI.WAV", FA_READ);
					etat=13;
				}

				if ((Touch.Y>94) && (Touch.Y<125) && (Touch.X>343) && ((Touch.X<380))){
					NB_Bloc=48;
					f_close(&SDFile);
					f_open(&SDFile, "13-12MI.WAV", FA_READ);
					etat=13;
				}

				if ((Touch.Y>68) && (Touch.Y<101) && (Touch.X>124) && ((Touch.X<162))){
					NB_Bloc=44;
					f_close(&SDFile);
					f_open(&SDFile, "14-1FA.WAV", FA_READ);
					etat=13;
				}

				if ((Touch.Y>66) && (Touch.Y<98) && (Touch.X>199) && ((Touch.X<237))){
					NB_Bloc=45;
					f_close(&SDFile);
					f_open(&SDFile, "15-1SOL.WAV", FA_READ);
					etat=13;
				}

				if ((Touch.Y>66) && (Touch.Y<98) && (Touch.X>273) && ((Touch.X<312))){
					NB_Bloc=43;
					f_close(&SDFile);
					f_open(&SDFile, "16-1LA.WAV", FA_READ);
					etat=13;
				}

				if ((Touch.Y>60) && (Touch.Y<95) && (Touch.X>343) && ((Touch.X<380))){
					NB_Bloc=44;
					f_close(&SDFile);
					f_open(&SDFile, "17-1SI.WAV", FA_READ);
					etat=13;
				}

				break;

			case 10: //Return to the introduction picture
				if (TS_State.touchDetected==0){
					etat=0;
					if (record_OK){
						f_close(&SDFile);
					}
				}
				break;

			case 13:
				if (TS_State.touchDetected==0){
					f_lseek(&SDFile, 44);
					Bloc_Cursor=0;
					etat=3;
				}
				break;


			case 3:
				if(record_OK==1){
					etat=31;
					record_OK=0;
				}
				break;

			case 30: //Return to the introduction picture
				if (TS_State.touchDetected==0){
					etat=0;
					if (record_OK){
						f_close(&SDFile);
					}
				}
				break;
			case 31:
				if (TS_State.touchDetected==0)
					etat=1;
				break;


			/////////////////// IMAGE 2 - BELLA CIAO
			case 102:
				BSP_LCD_DrawBitmap(0,0,(uint8_t*)bella_ciao_bmp);
				if (TS_State.touchDetected==0){
					f_open(&SDFile, "BELLA.WAV", FA_READ);
					musique=2;
					etat=1;
				}
				break;

			}
		vTaskDelay(20);
	}
  /* USER CODE END 5 */
}



/* USER CODE BEGIN Header_StartSD */
/**
* @brief Function implementing the SD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSD */
void StartSD(void const * argument)
{
  /* USER CODE BEGIN StartSD */
	SD_Init();
	enable = 1;
	vTaskDelete(SDHandle);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartSD */
}

/* USER CODE BEGIN Header_StartRecord */
/**
* @brief Function implementing the Record thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRecord */
void StartRecord(void const * argument)
{
  /* USER CODE BEGIN StartRecord */
	char i;
	uint32_t byteswritten,bytesread;
  /* Infinite loop */
  for(;;)
  {
	  xQueueReceive(WakeUpHandle, &i, portMAX_DELAY);
	  if (i==0){
		  //f_write(&SDFile,(uint8_t*) (AUDIO_BUFFER_OUT) , AUDIO_BLOCK_SIZE,(void*) &byteswritten);
		  //NB_Bloc=3312;
	  }
	  if (i==1){
		  //f_write(&SDFile,(uint8_t*) (AUDIO_BUFFER_OUT + AUDIO_BLOCK_SIZE), AUDIO_BLOCK_SIZE,(void*) &byteswritten);
		  //NB_Bloc=3312;
	  }
	  if (i==2){
		  if (Bloc_Cursor++==NB_Bloc-1){
			  //f_lseek(&SDFile, 44);
			  record_OK=1;
			  f_close(&SDFile);
//			  Bloc_Cursor=0;
		  }
		  f_read(&SDFile, ((uint8_t*)AUDIO_BUFFER_READ), AUDIO_BLOCK_SIZE,(void*) &bytesread);
		  //memcpy((uint16_t*) (AUDIO_BUFFER_OUT),(uint16_t*) (AUDIO_BUFFER_READ),AUDIO_BLOCK_SIZE);
		  Addition(0);
	  }
	  if (i==3){
		  if (Bloc_Cursor++==NB_Bloc-1){
//			  f_lseek(&SDFile, 44);
//			  Bloc_Cursor=0;
			  record_OK=1;
			  f_close(&SDFile);
		  }
		  f_read(&SDFile, ((uint8_t*)AUDIO_BUFFER_READ+AUDIO_BLOCK_SIZE), AUDIO_BLOCK_SIZE,(void*) &bytesread);
		  //memcpy((uint16_t*) (AUDIO_BUFFER_OUT+AUDIO_BLOCK_SIZE),(uint16_t*) (AUDIO_BUFFER_READ+AUDIO_BLOCK_SIZE),AUDIO_BLOCK_SIZE);
		  Addition(1);
	  }
  }
  /* USER CODE END StartRecord */
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
