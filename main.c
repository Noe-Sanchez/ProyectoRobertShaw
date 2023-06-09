#include "main.h"
#include "myprintf.h"
#include "math.h"
#include "lcd.h"

void SystemClock_Config(void);
static void MX_GPIO_Init(void);

void USER_RCC_Init(void);
void USER_GPIO_Init(void);
void USER_TIM2_Capture_Init(void);
uint16_t USER_TIM2_Capture_Event(void);

void USER_USART2_Transmit(uint8_t *pData, uint16_t size);
int USER_USART2_Available(void);
uint8_t USER_USART2_Read(void);
void USER_USART2_Init(void);

void USER_ADC_Init(void);
void USER_ADC_Calibration(void);
uint16_t USER_ADC_Read( void );


void USER_TIM2_Capture_Init(void);
uint16_t USER_TIM2_Capture_Event(void);

int main(void){
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  USER_RCC_Init();
  USER_GPIO_Init();

  USER_USART2_Init();

  USER_ADC_Init();
  USER_ADC_Calibration();

  USER_TIM2_Capture_Init();


  uint16_t event_val1, event_val2, event_diff,
  pressed_t, dataADC;


  LCD_Init();
  LCD_Cursor_ON();
  LCD_Clear();
  LCD_Set_Cursor(1,0);
  LCD_Put_Str("ROBERTSHAW");
  LCD_Set_Cursor(2,0);
  LCD_Put_Str("TE2003B");

  HAL_Delay(2000);
  LCD_Clear();

  //LCD_Set_Cursor(1,0);
  //LCD_Put_Str("Lectura ADC:");


  uint8_t msg[] = "\r\n";
  USER_USART2_Transmit(msg, 28);
  ADC1->CR2	|=	 ADC_CR2_ADON;
  char lastReceived;
  while (1){
	char received = 'x';
	received = USER_USART2_Read();
	if(received != 'x'){
		LCD_Clear();
		lastReceived = received;
		int intpart = -1;
		int intpart2 = -1;
		int floatpart = 0;
		int floatpart2 = 0;
		if(received == 'A' || received == 'P'){
			//----------------------PWM
			dataADC = USER_ADC_Read();
			float converted = 3.3*(dataADC/((pow(2,12)-1)));
			intpart = floor(converted);
			floatpart = (converted - intpart) * 100;
			//-------------------------

			float psi = (converted * 25) - 12;

			LCD_Set_Cursor(2,0);
			LCD_Put_Num(floor(psi));
			LCD_Put_Str(".");
			LCD_Put_Num((psi - floor(psi))*100);
			LCD_Put_Str(" psi");

	   }
	   if(received == 'A' || received == 'F'){
		    //--------------------ADC
		   	event_val1 = USER_TIM2_Capture_Event();//	capture the 1st event
		   	event_val2 = USER_TIM2_Capture_Event();//	capture the 2nd event
		   	event_diff = event_val2 - event_val1;//		2nd event - 1st event
		   	//Calculating time according to the timer ticks difference5
		   	pressed_t = 1/(( 1.0 / 64000000.0 ) * event_diff * (TIM2->PSC + 1));
		   	intpart2 = floor(pressed_t);
		   	floatpart2 = (pressed_t - intpart2) * 100;
		   	//-----------------------

		   	float lm = 0.12121 * pressed_t + 0.05349;

		   	LCD_Set_Cursor(1,0);
		   	LCD_Put_Num(floor(lm));
		   	LCD_Put_Str(".");
		   	LCD_Put_Num((lm - floor(lm))*100);
		   	LCD_Put_Str(" l/m");

	 }
	 char str[30];

	 sprintf(str, "%d", intpart2);
	 printf(str);
	 printf(".");
	 sprintf(str, "%d", floatpart2);
	 printf(str);

	 sprintf(str, "%c", ' ');
	 printf(str);

	 sprintf(str, "%c", ' ');

	 sprintf(str, "%d", intpart);
	 printf(str);
	 printf(".");
	 sprintf(str, "%d", floatpart);
	 printf(str);

	 printf("\r\n");
	}
  }

}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void){
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

/* USER CODE BEGIN 4 */
void USER_RCC_Init(void){
	//I/O port A clock enable
	RCC->APB2ENR	|=	 RCC_APB2ENR_IOPAEN | RCC_APB2ENR_ADC1EN;
	RCC->CFGR	|=	 RCC_CFGR_ADCPRE;
	//Timer 2 clock enable
	RCC->APB1ENR	|=	 RCC_APB1ENR_TIM2EN;
	RCC->APB1ENR	|=	 RCC_APB1ENR_TIM2EN;
	RCC->APB1ENR	|=	 RCC_APB1ENR_USART2EN;//  	USART2 clock enable
}
void USER_GPIO_Init(void){
	//Input ADC
	GPIOA->CRL	&=	~GPIO_CRL_CNF0 & ~GPIO_CRL_MODE0;
	//Input frec
	GPIOA->CRL	&=	~GPIO_CRL_CNF1_1 & ~GPIO_CRL_MODE1;
	GPIOA->CRL	|=	 GPIO_CRL_CNF1_0;


	GPIOA->CRL	&=	~GPIO_CRL_CNF2_0 & ~GPIO_CRL_MODE2_1;
	GPIOA->CRL	|=	 GPIO_CRL_CNF2_1 | GPIO_CRL_MODE2_0;

	GPIOA->CRL	&=	~GPIO_CRL_CNF3_1 & ~GPIO_CRL_MODE3;
	GPIOA->CRL	|=	GPIO_CRL_CNF3_0;

	//pin A5 as output push-pull max speed 10MHz
	GPIOA->CRL &= ~GPIO_CRL_CNF5 & ~GPIO_CRL_MODE5_1;
	GPIOA->CRL |= GPIO_CRL_MODE5_0;



	USART2->CR1	|=	 USART_CR1_UE;//		USART enabled
	USART2->CR1	&=	~USART_CR1_M//		  	1 start bit, 8 data bits
				&	~USART_CR1_PCE;//		parity control disabled
	USART2->CR2	&=	~USART_CR2_STOP;//  		1 stop bit
	USART2->BRR	 =	 0x116;//			115200 bps -> 17.36,
	USART2->CR1	|=	 USART_CR1_TE;//	        transmitter enabled
	USART2->CR1	|=	 USART_CR1_RE;// receiver enabled
}
void USER_ADC_Init(void){
	ADC1->CR1	&=	~ADC_CR1_DUALMOD;
	ADC1->CR2	&=	~ADC_CR2_ALIGN;
	ADC1->CR2	|=	 ADC_CR2_CONT;
	ADC1->SMPR2	&=	~ADC_SMPR2_SMP0;
	ADC1->SQR1	&=	~ADC_SQR1_L;
	ADC1->SQR3 	&=	~ADC_SQR3_SQ1;
	ADC1->CR2	|=	 ADC_CR2_ADON;
	HAL_Delay(1);
}
void USER_ADC_Calibration(void){
	ADC1->CR2	|=	 ADC_CR2_CAL;
	while( ADC1->CR2 & ADC_CR2_CAL );
}
uint16_t USER_ADC_Read( void ){
	while( !( ADC1->SR & ADC_SR_EOC ) );
	return (uint16_t)ADC1->DR;
}


void USER_USART2_Transmit(uint8_t *pData, uint16_t size ){
	for( int i = 0; i < size; i++ ){
		while( ( USART2->SR & USART_SR_TXE ) == 0 ){}//	wait until transmit reg is empty
		USART2->DR = *pData++;//			transmit data
	}
}

uint8_t USER_USART2_Read(){
	uint8_t charReceived = 'x';
	if( ( USART2->SR & USART_SR_RXNE ) != 0 ){charReceived = USART2->DR;}//	wait until transmit reg is empty
	//printf("Received data \r\n");
	//charReceived = USART2->DR;//			transmit data
	return charReceived;
}

void USER_USART2_Init(void){
	USART2->CR1	|=	 USART_CR1_UE;//		USART enabled
	USART2->CR1	&=	~USART_CR1_M//		  	1 start bit, 8 data bits
			&	~USART_CR1_PCE;//		parity control disabled
	USART2->CR2	&=	~USART_CR2_STOP;//  		1 stop bit
	USART2->BRR	 =	 0x116;//			115200 bps -> 17.36,
	USART2->CR1	|=	 USART_CR1_TE;//	        transmitter enabled
	USART2->CR1	|=	 USART_CR1_RE;// receiver enabled
}

void USER_TIM2_Capture_Init(void){
	TIM2->CR1	&=	~TIM_CR1_CKD_0;
	TIM2->CR1	|=	 TIM_CR1_CKD_1;//	sampling (DTS) = TIM_CLK/4
	TIM2->CCMR1 	&=	~TIM_CCMR1_CC1S_0;
	TIM2->CCMR1 	|=	 TIM_CCMR1_CC1S_1;//	CC1 channel as input, mapped on TI1
	TIM2->CCMR1 	|=	 TIM_CCMR1_IC1F;//	filter -> DTS/32, N=8
	TIM2->CCER	|=	 TIM_CCER_CC1P;//	capture is done on falling edge
	TIM2->CCMR1 	&=	~TIM_CCMR1_IC1PSC;//	no prescaler
	TIM2->CCER	|=	 TIM_CCER_CC1E;//	capture enabled
	TIM2->PSC	 =	 977;//		maximum prescaler
	TIM2->CR1	|=	 TIM_CR1_CEN;//		counter enabled
}
uint16_t USER_TIM2_Capture_Event(void){
	while( !(TIM2->SR & TIM_SR_CC1IF) );//		wait until a capture occurrs
	return TIM2->CCR1;//				return the captured value
}

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
