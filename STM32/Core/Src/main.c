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
#include "usb_device.h"
#include "stdio.h"
#include "../../STM32CubeIDE/Middlewares/USB_Device_Library/usbd_hid_ext.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define KEYBOARD //also need to define in usbd_hid.c!

#define MOD_ALT_LEFT 0x04
#define WINKEY_LEFT 0xE3
#define ENTER 0x28
#define SPACE 0x2C
#define COMMA 0x36
#define DOT 0x37
#define K_A 0x04
#define K_B 0x05
#define K_C 0x06
#define K_D 0x07
#define K_E 0x08
#define K_G 0x0A
#define K_H 0x0B
#define K_I 0x0C
#define K_L 0x0F
#define K_M 0x10
#define K_N 0x11
#define K_O 0x12
#define K_P 0x13
#define K_R 0x15
#define K_S 0x16
#define K_T 0x17
#define K_U 0x18
#define K_V 0x19
#define K_W 0x1A

#define K_F8 0x41
#define CAPSLOCK 0x39
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

/* USER CODE BEGIN PV */
volatile uint8_t capslock_;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
/* USER CODE BEGIN PFP */
extern USBD_HandleTypeDef hUsbDeviceFS;
void Keyboard_SendChars(uint8_t mod, uint8_t *buf);
void Keyboard_SendSingleChar(uint8_t mod, uint8_t val);
void Keyboard_SendHelloWorld();
void Keyboard_WaitCapsLock();
void Keyboard_ShowPassword();
void Keyboard_TestADC();

typedef struct
{
	uint8_t MODIFIER;
	uint8_t RESERVED;
	uint8_t KEYCODE1;
	uint8_t KEYCODE2;
	uint8_t KEYCODE3;
	uint8_t KEYCODE4;
	uint8_t KEYCODE5;
	uint8_t KEYCODE6;
} kbdHID;

kbdHID keyboardHID = {0,0,0,0,0,0,0,0};
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Keyboard_SendChars(uint8_t mod, uint8_t *buf)
	{
		keyboardHID.MODIFIER = mod;
		keyboardHID.KEYCODE1 = buf[0];
		keyboardHID.KEYCODE2 = buf[1];
		keyboardHID.KEYCODE3 = buf[2];
		keyboardHID.KEYCODE4 = buf[3];
		keyboardHID.KEYCODE5 = buf[4];
		keyboardHID.KEYCODE6 = buf[5];
		USBD_HID_SendReport(&hUsbDeviceFS, &keyboardHID, sizeof (keyboardHID));
		HAL_Delay (50);

		//release
		keyboardHID.MODIFIER = 0x00;
		keyboardHID.KEYCODE1 = 0x00;
		keyboardHID.KEYCODE2 = 0x00;
		keyboardHID.KEYCODE3 = 0x00;
		keyboardHID.KEYCODE4 = 0x00;
		keyboardHID.KEYCODE5 = 0x00;
		keyboardHID.KEYCODE6 = 0x00;
		USBD_HID_SendReport(&hUsbDeviceFS, &keyboardHID, sizeof (keyboardHID));
		HAL_Delay (50);
	}

	void Keyboard_SendSingleChar(uint8_t mod, uint8_t val)
	{
		keyboardHID.MODIFIER = mod;
		keyboardHID.KEYCODE1 = val;
		USBD_HID_SendReport(&hUsbDeviceFS, &keyboardHID, sizeof (keyboardHID));
		HAL_Delay (50);

		keyboardHID.MODIFIER = 0;
		keyboardHID.KEYCODE1 = 0;
		USBD_HID_SendReport(&hUsbDeviceFS, &keyboardHID, sizeof (keyboardHID));
		HAL_Delay(50);
	}

	//open terminal and send hello world
	void Keyboard_SendHelloWorld()
	{
	    uint8_t buf[6] = {0,0,0,0,0,0};
		//winkey+r to open run utility
	    buf[0] = WINKEY_LEFT; buf[1] = K_R;
		Keyboard_SendChars(0,buf);
		HAL_Delay(100);
		//open command prompt
		buf[0] = K_C; buf[1] = K_M; buf[2] = K_D; buf[3] = ENTER;
		Keyboard_SendChars(0,buf);
		HAL_Delay(600);
		buf[0] = K_E; buf[1] = K_C; buf[2] = K_H; buf[3] = K_O; buf[4] = SPACE;
		Keyboard_SendChars(0,buf);
		buf[0] = K_H; buf[1] = K_E; buf[2] = K_L; buf[3] = 0x00; buf[4] = 0x00;
		Keyboard_SendChars(0,buf);
		buf[0] = K_L; buf[1] = K_O; buf[2] = SPACE;
		Keyboard_SendChars(0,buf);
		buf[0] = K_W; buf[1] = K_O; buf[2] = K_R; buf[3] = K_L; buf[4] = K_D; buf[5] = ENTER;
		Keyboard_SendChars(0,buf);
	}

	//check that PC is unlocked before executing by setting capslock and waiting for user to turn it off
	void Keyboard_WaitCapsLock()
	{
		//try to set capslock until it succeeds
		while(capslock_==0) {
		  Keyboard_SendSingleChar(0x00, CAPSLOCK);
		  HAL_Delay(1000);
		}
		while(capslock_==1); //wait here for user to turn off capslock
		HAL_Delay(2000);
	}

	//send ALT+F8 shortcut to display passwords being typed in
	void Keyboard_ShowPassword()
	{
		keyboardHID.MODIFIER = MOD_ALT_LEFT;
		keyboardHID.KEYCODE1 = K_F8;
		USBD_HID_SendReport(&hUsbDeviceFS, &keyboardHID, sizeof (keyboardHID));
		HAL_Delay (250);

		keyboardHID.MODIFIER = 0;
		keyboardHID.KEYCODE1 = 0;
		USBD_HID_SendReport(&hUsbDeviceFS, &keyboardHID, sizeof (keyboardHID));
		HAL_Delay(50);
	}

	//send current ADC value over USB
	void Keyboard_TestADC()
	{
		uint8_t buf[6] = {0,0,0,0,0,0};
		uint16_t adc_val = 0;
		char adc_str[6];
		HAL_ADC_Start(&hadc);
		HAL_ADC_PollForConversion(&hadc,100);
		adc_val = HAL_ADC_GetValue(&hadc);
		HAL_ADC_Stop(&hadc);
		sprintf(adc_str, "%04d", adc_val);
		for(uint8_t i=0;i<4;i++){
		  if(adc_str[i]==0x30) buf[i]=0x27; //zero case
		  else buf[i] = adc_str[i] - 0x13; //translate other digits from ASCII to HID
		  Keyboard_SendSingleChar(0,buf[i]);
		}
		Keyboard_SendSingleChar(0,ENTER);
		HAL_Delay(1000);
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
  capslock_ = 0;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(3000);
  Keyboard_WaitCapsLock();
  //HAL_Delay(10000);

  //get reference ADC value without IR by averaging 16 ADC readings over first 8 seconds
  uint16_t adc_val_ini = 0;
  for(uint8_t i = 0; i<16; i++) {
	HAL_Delay(500);
	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc,100);
	adc_val_ini += HAL_ADC_GetValue(&hadc) >> 4;
	HAL_ADC_Stop(&hadc);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_ADC_Start(&hadc);
	  HAL_ADC_PollForConversion(&hadc,100);
	  //default ADC value seems to sink quite a bit from initial during use, presumably
	  //due to temperature dependence of the photodiodes - so subtract 200 from initial value
	  //to use as reference
	  if(HAL_ADC_GetValue(&hadc) < (adc_val_ini - 200)) {
		  Keyboard_SendHelloWorld();
		  //Keyboard_ShowPassword();
	  }
	  HAL_Delay(500);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
