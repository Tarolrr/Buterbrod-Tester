//TODO implement oscillator test

#include "stm32f1xx_hal.h"
#include "SX1278Drv.h"
#include <string.h>
#include "main.h"
#include "cmsis_os.h"

#define MySTM
//#define FreeRTOS
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim1;
UART_HandleTypeDef huart1;
SX1278Drv_LoRaConfiguration cfg;
osThreadId hMainTask;
uint8_t UART_RX[6];
uint8_t UART_TX[6];
uint8_t state =State_Idle;
uint8_t stateArg = 0;
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void	MainTaskFxn(void const * argument);

int main(void){

	HAL_Init();

	SystemClock_Config();
	MX_GPIO_Init();
	MX_SPI1_Init();
	MX_USART1_UART_Init();

	cfg.bw = SX1278Drv_RegLoRaModemConfig1_BW_125;
	cfg.cr = SX1278Drv_RegLoRaModemConfig1_CR_4_8;
	cfg.crc = SX1278Drv_RegLoRaModemConfig2_PayloadCrc_ON;
	cfg.hdrMode = SX1278Drv_RegLoRaModemConfig1_HdrMode_Explicit;
	cfg.power = 17;
	cfg.preambleLength = 20;
	cfg.sf = SX1278Drv_RegLoRaModemConfig2_SF_12;
	cfg.spi = &hspi1;
	cfg.sleepInIdle = true;
#ifdef MySTM
	cfg.frequency = 434e6;
	cfg.spi_css_pin = &SPICSMyPin;
	cfg.tx_led = &LoRaTxRxPin;
#else
	cfg.frequency = 868e6;
	cfg.spi_css_pin = &SPICSPin;
	cfg.rx_en = &LoRaRxEnPin;
	cfg.tx_en = &LoRaTxEnPin;
#endif

	SX1278Drv_Init(&cfg);


	osThreadDef(MainTask, MainTaskFxn, osPriorityNormal, 0, 512);
	hMainTask = osThreadCreate(osThread(MainTask), NULL);

	HAL_UART_Receive_IT(&huart1,UART_RX,6);
	osKernelStart();
	return 0;
}

void SystemClock_Config(void){
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
		Error_Handler();

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
							  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
		Error_Handler();


	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
	HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void){
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
		Error_Handler();
}

static void MX_USART1_UART_Init(void){
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
    Error_Handler();
}

static void MX_GPIO_Init(void){
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	uint8_t i;
	for(i = 0; i < GPIOPinNum; i++){
		GPIO_PIN_SET(GPIOPins+i);
		GPIO_InitStruct.Pin = GPIOPins[i].pin;
		HAL_GPIO_Init(GPIOPins[i].port, &GPIO_InitStruct);
		GPIO_PIN_RESET(GPIOPins+i);
	}
}

void Error_Handler(void){
  while(1);
}

static void	MainTaskFxn(void const * argument){
	while(1){
		switch(state){
		case State_Idle:
			break;
		case State_Test_Pin:
			if(stateArg < GPIOPinNum){
				GPIO_PIN_TOGGLE(GPIOPins+stateArg);
				osDelay(1);
				GPIO_PIN_TOGGLE(GPIOPins+stateArg);
				osDelay(1);
			}
			state = State_Idle;
			break;
		case State_Test_Osc:
			state = State_Idle;
			break;
		case State_Test_LoRa:
			break;
		}
	}
}

void SX1278Drv_LoRaRxCallback(LoRa_Message *msg){}

void SX1278Drv_LoRaRxError(){}

void SX1278Drv_LoRaTxCallback(LoRa_Message *msg){}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if (htim->Instance == TIM1)
    HAL_IncTick();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	switch(UART_RX[0]){
	case Test_Commands_Pin:
		state = State_Test_Pin;
		stateArg = UART_RX[1];
		break;
	case Test_Commands_Osc:

	}

}
