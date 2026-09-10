/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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
#include "spi.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "string.h"
#include "usbd_cdc_if.h"
#include "ism330dhcx_reg.h"
#include "NanoEdgeAI.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* Step #2 Definition of enumerated type of FSM states*/
typedef enum
{
  LOG,
  LEARN,
  INFERENCE,
} States_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define VERSION                (uint32_t)(260822)

#define SENSOR_BUS            hspi3  /* Sensor SPI bus */
#define SENSOR_BOOT_TIME      (uint32_t)(100)  /* Boot time of the sensor [ms] */
#define NUMBER_OF_VALUES      (uint32_t)(DATA_INPUT_USER)
#define NUMBER_OF_AXIS        (uint32_t)(AXIS_NUMBER)
#define SIGNAL_SIZE           (uint32_t)(NUMBER_OF_VALUES * NUMBER_OF_AXIS)  /* Runtime signal size */
#define LEARNING_ITERATIONS   (uint32_t)(10)
#define LED_LOG_PERIOD        (uint32_t)(75)  /* LEDs toggling half period [ms] following the app state */
#define LED_LEARN_PERIOD      (uint32_t)(1000)
#define LED_INFERENCE_PERIOD  (uint32_t)(250)
#define LED_ANOMALY_PERIOD    (uint32_t)(20)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

stmdev_ctx_t dev_ctx;
volatile States_t appState = LOG;  /* FSM state */
volatile uint32_t sensorDataReady = RESET;
volatile uint32_t led1PeriodCnt = 0;  /* LED1 toggling counter incremented by SysTick */
volatile uint32_t led1Period = 0;  /* LED1 toggle timing to be set by user */
volatile uint32_t led1Enable = SET;
volatile uint32_t led2PeriodCnt = 0;  /* LED1 toggling counter incremented by SysTick */
volatile uint32_t led2Period = 0;  /* LED1 toggle timing to be set by user */
volatile uint32_t led2Enable = RESET;
float input_user_buffer[SIGNAL_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void CHARGER_Init(void);

int32_t platform_write(void *handle,uint8_t reg,const uint8_t *bufp,uint16_t len);
int32_t platform_read(void *handle,uint8_t reg,uint8_t *bufp,uint16_t len);

void MEMS_Init(void);
void FillBuffer(float *buffer, uint32_t size);
void PrintBuffer(float *buffer, uint32_t size);
void Log(void);
void Learn(void);
void Inference(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int fd, char * ptr, int len)
{
  CDC_Transmit_FS((uint8_t *)ptr,len);
  HAL_Delay((uint32_t)((len / 8) + 1));
  return len;
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

  HAL_PWREx_EnableVddUSB();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI3_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(500);  /* Delay to init VCP on host side */
  CHARGER_Init();
  MEMS_Init();
  printf("\r\nVer: %u.%u\r\n",(uint)VERSION,(uint)SIGNAL_SIZE);
  enum neai_state error_code = neai_anomalydetection_init();
  if (error_code!=NEAI_OK)
  {
    printf("Knowledge initialization ERROR: %d\r\n",error_code);
    Error_Handler();
  }
  else
  {
    printf("Knowledge initialization done\r\n");
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	switch (appState)
	{
	  case LOG:
	    Log();
	  	break;
	  case LEARN:
	    Learn();
	  	appState = INFERENCE;
	  	break;
	  case INFERENCE:
	    Inference();
	  	break;
	  default:
	    appState = LOG;
	  	break;
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 30;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
  /** Enable the SYSCFG APB clock
  */
  __HAL_RCC_CRS_CLK_ENABLE();
  /** Configures CRS
  */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,1000);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
}

/* USER CODE BEGIN 4 */

void Delay_us_Init(void)
{
  CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0;
}

void Delay_us(volatile uint32_t us)
{
  uint32_t t0 = DWT->CYCCNT;
  us = us * (HAL_RCC_GetHCLKFreq() / 1000000);
  while ((DWT->CYCCNT - t0) < us);
}

void SWIRE_Cmd(uint8_t cmd)
{
  Delay_us_Init();

  HAL_GPIO_WritePin(SW_SEL_GPIO_Port,SW_SEL_Pin,GPIO_PIN_SET);  // tstart
  Delay_us(375);

  for(uint8_t i=0;i<cmd;i++)
  {
    HAL_GPIO_WritePin(SW_SEL_GPIO_Port,SW_SEL_Pin,GPIO_PIN_RESET);  // tl
	Delay_us(110);
	HAL_GPIO_WritePin(SW_SEL_GPIO_Port,SW_SEL_Pin,GPIO_PIN_SET);  // th
	Delay_us(110);
  }
  HAL_GPIO_WritePin(SW_SEL_GPIO_Port,SW_SEL_Pin,GPIO_PIN_RESET);  // tlstop
  Delay_us(110);
  HAL_GPIO_WritePin(SW_SEL_GPIO_Port,SW_SEL_Pin,GPIO_PIN_SET);  // thstop
  Delay_us(550);
  HAL_GPIO_WritePin(SW_SEL_GPIO_Port,SW_SEL_Pin,GPIO_PIN_RESET);  // IDLE
}

void CHARGER_Init(void)
{
  HAL_GPIO_WritePin(C_EN_GPIO_Port,C_EN_Pin,GPIO_PIN_SET);  // Charging enable
  SWIRE_Cmd(23); // CHARGER SHUTDOWN when no USB cable connected (SWIRE command)
}


int32_t platform_write(void *handle,uint8_t reg,const uint8_t *bufp,uint16_t len)
{
  HAL_GPIO_WritePin(CS_DHC_GPIO_Port, CS_DHC_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(CS_DHC_GPIO_Port, CS_DHC_Pin, GPIO_PIN_SET);

  return 0;
}


int32_t platform_read(void *handle,uint8_t reg,uint8_t *bufp,uint16_t len)
{
  /* Read command */
  reg |= 0x80;
  HAL_GPIO_WritePin(CS_DHC_GPIO_Port, CS_DHC_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_DHC_GPIO_Port, CS_DHC_Pin, GPIO_PIN_SET);

  return 0;
}

void MEMS_Init(void)
{
  uint8_t whoamI;
  ism330dhcx_int1_ctrl_t int1Cfg = {0};
  int16_t acc_axes_raw[3], gy_axes_raw[3];

  HAL_GPIO_WritePin(CS_DH_GPIO_Port, CS_DH_Pin, GPIO_PIN_SET);      /* Deselect IIS2DH CS */
  HAL_GPIO_WritePin(CS_DHC_GPIO_Port, CS_ADWB_Pin, GPIO_PIN_SET);   /* Deselect IIS3DWB CS */
  HAL_GPIO_WritePin(CS_DHC_GPIO_Port, CS_DHC_Pin, GPIO_PIN_SET);    /* Deselect ISM330DHCX CS */


  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &SENSOR_BUS;
  HAL_Delay(SENSOR_BOOT_TIME);
  ism330dhcx_device_id_get(&dev_ctx, &whoamI);
  //printf("\r\nSENSOR WHOAMI=0x%02X\r\n",(int)whoamI);
  if (whoamI!=ISM330DHCX_ID)
  {
    printf("ISM330DHCX not detected !\r\n");
  	Error_Handler();
  }
  else
  {
	ism330dhcx_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
	ism330dhcx_gy_data_rate_set(&dev_ctx, ISM330DHCX_GY_ODR_52Hz);
	ism330dhcx_xl_data_rate_set(&dev_ctx, ISM330DHCX_XL_ODR_52Hz);
	ism330dhcx_gy_full_scale_set(&dev_ctx, ISM330DHCX_250dps);
	ism330dhcx_xl_full_scale_set(&dev_ctx, ISM330DHCX_2g);
	ism330dhcx_xl_hp_path_on_out_set(&dev_ctx, ISM330DHCX_LP_ODR_DIV_100);
	ism330dhcx_xl_filter_lp2_set(&dev_ctx, PROPERTY_ENABLE);
	int1Cfg.int1_drdy_xl = 1;
	ism330dhcx_write_reg(&dev_ctx, ISM330DHCX_INT1_CTRL,(uint8_t*)&int1Cfg,1);
	ism330dhcx_acceleration_raw_get(&dev_ctx,acc_axes_raw);
	ism330dhcx_angular_rate_raw_get(&dev_ctx,gy_axes_raw);
  }
}

void FillBuffer(float* buffer, uint32_t size)
{
  int16_t acc_axes_raw[3], gy_axes_raw[3];
  uint32_t i = 0;

  while (i < size)
  {
    if (sensorDataReady!=RESET)
    {
      sensorDataReady = RESET;
      for(uint8_t j=0;j<3;j++) { acc_axes_raw[j] = 0; gy_axes_raw[j] = 0;}
      ism330dhcx_acceleration_raw_get(&dev_ctx,acc_axes_raw);
      ism330dhcx_angular_rate_raw_get(&dev_ctx,gy_axes_raw);
      buffer[i] = ism330dhcx_from_fs2g_to_mg(acc_axes_raw[0]);
      buffer[i+1] = ism330dhcx_from_fs2g_to_mg(acc_axes_raw[1]);
      buffer[i+2] = ism330dhcx_from_fs2g_to_mg(acc_axes_raw[2]);
      buffer[i+3] = ism330dhcx_from_fs250dps_to_mdps(gy_axes_raw[0]) / 1000; /* [mdps] -> [dps] */
      buffer[i+4] = ism330dhcx_from_fs250dps_to_mdps(gy_axes_raw[1]) / 1000;
      buffer[i+5] = ism330dhcx_from_fs250dps_to_mdps(gy_axes_raw[2]) / 1000;
      i += NUMBER_OF_AXIS;
    }
  }
}

void PrintBuffer(float *buffer, uint32_t size)
{
  uint32_t i = 0;
  uint32_t j;

  while (i < size)
  {
	for (j=0;j<NUMBER_OF_AXIS;j++)
	{
	  printf("%d ",(int)buffer[i+j]);
	}
    i += NUMBER_OF_AXIS;
  }
  printf("\n");
}

void Log(void)
{
  led1Enable = SET;
  led2Enable = RESET;
  led1Period = LED_LOG_PERIOD;
  FillBuffer(input_user_buffer,SIGNAL_SIZE);
  PrintBuffer(input_user_buffer,SIGNAL_SIZE);
}

void Learn(void)
{
  enum neai_state learnStatus;
  uint32_t learnIteration = 0;

  led1Enable = SET;
  led2Enable = RESET;
  led1Period = LED_LEARN_PERIOD;
  printf("\r\nStart of learning\r\n");
  HAL_Delay(2000);  /* Avoid parasitic vibrations of the mechanical setup*/
  while (learnIteration < LEARNING_ITERATIONS)
  {
    printf("Learning... %d/%d",(int)learnIteration+1,(int)LEARNING_ITERATIONS);
    FillBuffer(input_user_buffer,SIGNAL_SIZE);
    learnStatus = neai_anomalydetection_learn(input_user_buffer);
    if ((learnStatus==NEAI_OK) || (learnStatus==NEAI_MINIMAL_RECOMMENDED_LEARNING_DONE)) { printf(" -> OK\r\n"); }
    else if (learnStatus==NEAI_NOT_ENOUGH_CALL_TO_LEARNING) { printf(" -> .\r\n"); }
    else { printf(" -> ERR: %d\r\n",(int)learnStatus); }
    learnIteration++;
  }
}

void Inference(void)
{
  enum neai_state detectionStatus;
  uint8_t similarity = 0;

  FillBuffer(input_user_buffer,SIGNAL_SIZE);
  detectionStatus = neai_anomalydetection_detect(input_user_buffer,&similarity);
  if (detectionStatus==NEAI_OK)
  {
    printf("Similarity = %d%%",(int)similarity);
    if (similarity<90)
    {
      led1Enable = RESET;
      led2Enable = SET;
      led2Period = LED_ANOMALY_PERIOD;
      printf(" -> ANOMALY\r\n");
    }
    else
    {
      led2Enable = RESET;
      led1Enable = SET;
      led1Period = LED_INFERENCE_PERIOD;
      printf(" -> OK\r\n");
    }
  }
  else
  {
	printf("Detection error\r\n");
  }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
    case GPIO_PIN_8:
      sensorDataReady = SET;
      break;
    case GPIO_PIN_0:
      if (appState==LOG) { appState = LEARN; }
      else if (appState==INFERENCE) { appState = LOG; }
      break;
    default:
      break;
  }
}

void HAL_IncTick(void)
{
  uwTick += (uint32_t)uwTickFreq;

  if(led1PeriodCnt>0)
  {
    led1PeriodCnt--;
  }
  else
  {
    led1PeriodCnt = led1Period;
    if (led1Enable) { HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_1); }
    else { HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1,GPIO_PIN_RESET); }
  }

  if(led2PeriodCnt>0)
  {
    led2PeriodCnt--;
  }
  else
  {
    led2PeriodCnt = led2Period;
    if (led2Enable) { HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_0); }
    else { HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,GPIO_PIN_RESET); }
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

