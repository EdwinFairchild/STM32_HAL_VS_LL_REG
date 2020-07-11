#define USE_HAL
//#define USE_LL
//#define USE_REG

#ifdef USE_HAL

#include <stm32f1xx_hal.h>
#include <stm32_hal_legacy.h>
#include "CL_CONFIG.h"

#include "CL_printMsg.h"
#define LED0_Pin GPIO_PIN_13
#define LED0_GPIO_Port GPIOC

SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
void Error_Handler(void);
void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

int main(void)
{
	HAL_Init();

	
	SystemClock_Config();

	MX_GPIO_Init();
	MX_SPI1_Init();
	MX_USART1_UART_Init();
	
	CL_printMsg_init_Default(false);
	__GPIOC_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.Pin = GPIO_PIN_13;

	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

	for (;;)
	{
		
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_SPI_Transmit(&hspi1, 0x65, 1, HAL_MAX_DELAY);
		CL_printMsg("hello!");
		HAL_Delay(500);
		
		
	}
}
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };


	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
	                            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}


static void MX_SPI1_Init(void)
{

	
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		Error_Handler();
	}
	

}


static void MX_USART1_UART_Init(void)
{


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


}


static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();


	HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);


	GPIO_InitStruct.Pin = LED0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED0_GPIO_Port, &GPIO_InitStruct);

}
void Error_Handler(void)
{
	
}

#endif
 
#ifdef USE_LL

#include <stm32f1xx_ll_bus.h>
#include <stm32f1xx_ll_gpio.h>
#include <stm32f1xx_ll_utils.h>
#include <stm32f1xx_ll_spi.h>
#include <stm32f1xx_ll_exti.h>
#include <stm32f1xx_ll_system.h>
//------------| COMM LIBS |----------
#include "CL_CONFIG.h"
#include "CL_delay.h"
#include "CL_systemClockUpdate.h"
#include "CL_printMsg.h"







#define NRF_PINS_CLOCK_ENABLE() (RCC->APB2ENR |= RCC_APB2ENR_IOPAEN )  //given the pins are on GPIOA

uint8_t NRFSTATUS = 0x00;
uint8_t tx_data_buff[32];
uint8_t rx_data_buff[32];

uint8_t multibyte_buff[10] = { 0 };
uint8_t flag = 0x55;


void init_pins(void);
void init_spi1(void);


void spiSend(uint8_t  data);




void init_debug_led(void);
void blinkLed(void);
void printRegister(uint8_t reg);

int main(void)
{

	
	setSysClockTo72();
	CL_delay_init();
	CL_printMsg_init_Default(false); //this is in place of init UART
	init_debug_led();
	init_pins();
	init_spi1();	
	
	for(;  ;)
	{
		LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);
		LL_SPI_TransmitData8(SPI1, 0x65);
		CL_printMsg("hello!");
		delayMS(500);
	}
	
}

void init_pins(void)
{
	//clock enable GPIOA
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	
	LL_GPIO_InitTypeDef nrfPins;	
	LL_GPIO_StructInit(&nrfPins);	
	
	// CE pin as output : active high/ normally low
	// CSN chip select for spi active low / normally high
	nrfPins.Pin			= LL_GPIO_PIN_3 | LL_GPIO_PIN_2;
	nrfPins.Mode		= LL_GPIO_MODE_OUTPUT;
	nrfPins.OutputType	= LL_GPIO_OUTPUT_PUSHPULL;	
	nrfPins.Speed		= LL_GPIO_SPEED_FREQ_HIGH;
	LL_GPIO_Init(GPIOA, &nrfPins);  // CE & CSN on same port only need this once

	
	// IRQ pin as input with interrupt enabled
	nrfPins.Pin			= LL_GPIO_PIN_4;
	nrfPins.Mode = LL_GPIO_MODE_INPUT;
	nrfPins.Pull = LL_GPIO_PULL_UP;
	LL_GPIO_Init(GPIOA, &nrfPins);
		
	LL_EXTI_InitTypeDef myEXTI = { 0 };
	LL_EXTI_StructInit(&myEXTI);
	myEXTI.Line_0_31		= LL_EXTI_LINE_4;
	myEXTI.LineCommand		= ENABLE;
	myEXTI.Mode				= LL_EXTI_MODE_IT;
	myEXTI.Trigger			= LL_EXTI_TRIGGER_FALLING;
	LL_EXTI_Init(&myEXTI);	
	
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_3);
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_2);
	NVIC_EnableIRQ(EXTI4_IRQn); 	//enable IRQ on Pin 4
}

void init_spi1(void)
{
	// CLOCK  [ Alt Function ] [ GPIOA ] [ SPI1 ]
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_SPI1EN; 	
	
	// GPIO [ PA5:SCK:output:push ] [ PA6:MISO:input:float/pullup ] [ PA7:MOSI:output:push ]
	LL_GPIO_InitTypeDef spiGPIO;
	LL_GPIO_StructInit(&spiGPIO);
	
	spiGPIO.Pin			= LL_GPIO_PIN_5 | LL_GPIO_PIN_7;
	spiGPIO.Mode		= LL_GPIO_MODE_ALTERNATE;
	spiGPIO.OutputType	= LL_GPIO_OUTPUT_PUSHPULL;
	spiGPIO.Speed		= LL_GPIO_SPEED_FREQ_MEDIUM;
	
	LL_GPIO_Init(GPIOA, &spiGPIO);
	
	spiGPIO.Pin		= LL_GPIO_PIN_6;
	spiGPIO.Mode	= LL_GPIO_MODE_FLOATING;
	spiGPIO.Pull	= LL_GPIO_PULL_UP;
	LL_GPIO_Init(GPIOA, &spiGPIO);
		
	// SPI
	LL_SPI_InitTypeDef mySPI;
	LL_SPI_StructInit(&mySPI);
	
	mySPI.Mode		= LL_SPI_MODE_MASTER;
	mySPI.NSS		= LL_SPI_NSS_SOFT;
	mySPI.BaudRate	= LL_SPI_BAUDRATEPRESCALER_DIV32;

	LL_SPI_Init(SPI1, &mySPI);
	
	LL_SPI_Enable(SPI1);	
	
}

void init_debug_led(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	LL_GPIO_InitTypeDef debugLED;
	LL_GPIO_StructInit(&debugLED);
	debugLED.Pin        = LL_GPIO_PIN_13;
	debugLED.Mode       = LL_GPIO_MODE_OUTPUT;
	debugLED.Speed      = LL_GPIO_SPEED_FREQ_LOW;
	debugLED.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	LL_GPIO_Init(GPIOC, &debugLED);
	GPIOC->BSRR = LL_GPIO_PIN_13;
	
}

#endif

#ifdef USE_REG
#include "stm32f1xx.h"
//------------| COMM LIBS |----------
#include "CL_CONFIG.h"
#include "CL_delay.h"
#include "CL_systemClockUpdate.h"
#include "CL_printMsg.h"

void initSPI(void);
void initDebugLed(void);
uint8_t spiSend(uint8_t data);
int main(void)
{
	setSysClockTo72();
	CL_delay_init();
	CL_printMsg_init_Default(false);
	initDebugLed();
	 initSPI();
	
	for (;;)
	{
		
		GPIOC->ODR ^= GPIO_ODR_ODR13;
		spiSend(0x65);
		CL_printMsg("hello");
		delayMS(500);
		
		
	}
	
}

void initSPI(void)
{
	//clocks and pin configs related to spi
	RCC->APB2ENR 	|= RCC_APB2ENR_IOPAEN  | RCC_APB2ENR_SPI1EN | RCC_APB2ENR_AFIOEN;
	GPIOA->CRL 		&= ~(GPIO_CRL_MODE5 | GPIO_CRL_MODE7 | GPIO_CRL_CNF5 | GPIO_CRL_CNF7 | GPIO_CRL_CNF6 | GPIO_CRL_MODE6);
	GPIOA->CRL 		|= (GPIO_CRL_CNF5_1 | GPIO_CRL_MODE5 | GPIO_CRL_CNF7_1 | GPIO_CRL_MODE7 |  GPIO_CRL_CNF6_0);
	
	//setup spi
	SPI1->CR1 |= (SPI_CR1_SSM | SPI_CR1_BR_2 | SPI_CR1_MSTR);
	SPI1->CR2 |= (SPI_CR2_SSOE);
	SPI1->CR1 |= SPI_CR1_SPE;	
	
	
}

void initDebugLed(void)
{
	RCC->APB2ENR 	|= RCC_APB2ENR_IOPCEN;
	GPIOC->CRH 		&= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);
	GPIOC->CRH 		|= (GPIO_CRH_MODE13);
	
	
}

uint8_t spiSend(uint8_t data)
{
	uint8_t volatile i;
	while ((SPI1->SR & SPI_SR_TXE) != SPI_SR_TXE) 
	{
		i++;
	}
	SPI1->DR  = data;
	return SPI1->DR ;
	
}

#endif