/*
	纯雷达显示器代码
	Version：V1.00
	Date：04/04/2018
*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "WTN6.h"
#include "delay.h"
#include "DWINTFT_Radar.h"
#include "InternalFlash.h"

#define MAX_PROBE_NUM 8
#define PROBE_VERSION PROBE_VERSION_8
#define PROBE_VERSION_8	0x01
#define PROBE_VERSION_10 0x03
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t RadarRxComplete=0;				//雷达串口收到数据标志位
uint8_t TFTRxComplete=0;					//触摸屏收到数据标志位
uint8_t RadarRxBuf[32];						//雷达接收缓存
uint8_t TFTRxBuf[32];							//触摸屏接收缓存
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint8_t Radar_8Probe[MAX_PROBE_NUM] = {0};
uint8_t Radar_10Probe[MAX_PROBE_NUM] = {0};
uint8_t Radar_checksum = 0;

struct 
{
  uint8_t fornt_door;
  uint8_t rear_door;
  uint8_t vehicle_back;
  uint8_t vehicle_speed;
}Radar_State;
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t i;
	uint32_t RadarLimitDist=0x96;					//默认是1.5米
	uint32_t WTN6_Volume=0x03;						//默认最大音量
	uint8_t AlarmOn=0;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_CAN_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
	delay_init(72);
	TFT_SetProbeVersion(&huart2, PROBE_VERSION);
	
	RadarLimitDist=FlashRead32bit(FLASH_USER_START_ADDR+RADAR_LIMIT_OFFSET_ADDR);
	if(RadarLimitDist&0xFF==0xFF)		//如果此地址为空
	{
		RadarLimitDist=0x96;
		FlashWrite_SingleUint32(FLASH_USER_START_ADDR+RADAR_LIMIT_OFFSET_ADDR, RadarLimitDist);
	}
	WTN6_Volume=FlashRead32bit(FLASH_USER_START_ADDR+WTN6_VOLUME_OFFSET_ADDR);
	if(WTN6_Volume&0xFF==0xFF)
	{
		WTN6_Volume=0x03;
		FlashWrite_SingleUint32(FLASH_USER_START_ADDR+WTN6_VOLUME_OFFSET_ADDR, WTN6_Volume);
	}
	WTN6_SetVolume((uint8_t)WTN6_Volume);
	
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);//radar interrupt enable
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);//TFT interrupt enable
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		HAL_UART_Receive_DMA(&huart1, RadarRxBuf, 16);
		HAL_UART_Receive_DMA(&huart2, TFTRxBuf, 16);
		if(RadarRxComplete == 1)//雷达接收完成，进行解析
		{
			HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);//灯闪烁 提示接收到雷达数据
			uint8_t sum = 0;
      Radar_checksum = RadarRxBuf[14];//校验和赋值
      for(i = 1;i < 14;i++)//计算收到的数据的和
      {
        sum += RadarRxBuf[i];
      }
      if(Radar_checksum == sum)//校验和判断
      {
				if(0x55 == RadarRxBuf[0])	//帧开始字节
  			{
          if(8 == MAX_PROBE_NUM)    //如果是8探头
          {
            //给探头数据赋值
            //顺序为5#7#8#6#1#2#4#****3#
            //      1 2 3 4 5 6 7     12
            Radar_8Probe[5 - 1] = RadarRxBuf[1];//给探头数组赋值雷达数据，数组编号为逻辑编号-1
            Radar_8Probe[7 - 1] = RadarRxBuf[2];
            Radar_8Probe[8 - 1] = RadarRxBuf[3];
            Radar_8Probe[6 - 1] = RadarRxBuf[4];
            Radar_8Probe[1 - 1] = RadarRxBuf[5];
            Radar_8Probe[2 - 1] = RadarRxBuf[6];
            Radar_8Probe[4 - 1] = RadarRxBuf[7];
            Radar_8Probe[3 - 1] = RadarRxBuf[12];
            //显示屏显示0#探头数据
            TFT_DispRadarDist(&huart2, Radar_8Probe, 0);
						//显示屏显示颜色（波形）表示探头距离
						TFT_DispRadarColor(&huart2, Radar_8Probe, MAX_PROBE_NUM);
					}
          else                      //10探头
          {
            //给探头数据赋值
            //顺序为8#9#10#*1#2#3#4#5#6#7#*
            //      1 2 3   5 6 7 8 9 10 11
            for(i = 1; i < 4; i++)//赋值后三个探头，编号为8.9.10
            {
              Radar_10Probe[i + 6] = RadarRxBuf[i];
            }
            for(i = 1; i < 8; i++)//赋值前3和两侧各两个探头，编号为1~7
            {
              Radar_10Probe[i - 1] = RadarRxBuf[i + 4];
            }
            //显示屏显示0#探头数据
            TFT_DispRadarDist(&huart2, Radar_10Probe, 0);
						//显示屏显示颜色（波形）表示探头距离
						TFT_DispRadarColor(&huart2, Radar_10Probe, MAX_PROBE_NUM);
					}
				}
				//状态信号
  			Radar_State.fornt_door = RadarRxBuf[13] & 0x01;//bit0表示前门，0开1关
        Radar_State.rear_door = (RadarRxBuf[13] & 0x02) >> 1; //bit1表示后门，0开1关
        Radar_State.vehicle_back = (RadarRxBuf[13] & 0x04) >> 2;  //bit2表示倒车状态，0前1倒
        Radar_State.vehicle_speed = (RadarRxBuf[13] & 0x08) >> 3; //bit3表示车速，0高1低
			}
		}

		if(TFTRxComplete == 1)//屏幕信息接收完成，进行解析
		{
			switch(TFTRxBuf[5])
			{
				case 0x04:				//探头界面-探头按下
					break;
				case 0x06:				//探头界面-确认按下
					break;
				case 0x00:				//音量界面-确认按下
					WTN6_SetVolume(TFTRxBuf[10]);
				  FlashWrite_SingleUint32(FLASH_USER_START_ADDR+WTN6_VOLUME_OFFSET_ADDR, WTN6_Volume);
					break;
				case 0x02:				//距离界面-确认按下
					RadarLimitDist=TFTRxBuf[10];
					FlashWrite_SingleUint32(FLASH_USER_START_ADDR+RADAR_LIMIT_OFFSET_ADDR, RadarLimitDist);
					break;
			}
		}
			
		if(AlarmOn==1)
		{
		
		}
		//WTN6_Broadcast(0x00);		//播放第一段语音
		//for(i=0;i<10;i++)
		//Delay_ms(1000);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

/* CAN init function */
static void MX_CAN_Init(void)
{

  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SJW = CAN_SJW_1TQ;
  hcan.Init.BS1 = CAN_BS1_7TQ;
  hcan.Init.BS2 = CAN_BS2_8TQ;
  hcan.Init.TTCM = DISABLE;
  hcan.Init.ABOM = DISABLE;
  hcan.Init.AWUM = DISABLE;
  hcan.Init.NART = DISABLE;
  hcan.Init.RFLM = DISABLE;
  hcan.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
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
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 19200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BELL_DATA_GPIO_Port, BELL_DATA_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BELL_DATA_Pin */
  GPIO_InitStruct.Pin = BELL_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(BELL_DATA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BELL_BUSY_Pin */
  GPIO_InitStruct.Pin = BELL_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BELL_BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY4_Pin */
  GPIO_InitStruct.Pin = KEY4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEY4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY3_Pin KEY2_Pin KEY1_Pin */
  GPIO_InitStruct.Pin = KEY3_Pin|KEY2_Pin|KEY1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED0_Pin BUZZER_Pin */
  GPIO_InitStruct.Pin = LED0_Pin|BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
