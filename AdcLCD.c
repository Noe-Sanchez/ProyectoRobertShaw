#include "main.h"
#include "lcd.h"
#include "math.h"

void SystemClock_Config(void);
static void MX_GPIO_Init(void);

void USER_RCC_Init(void);
void USER_GPIO_Init(void);
void USER_ADC_Init(void);
void USER_ADC_Calibration(void);

uint16_t USER_ADC_Read( void );

int main(void){
  uint16_t dataADC;

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  USER_RCC_Init();
  USER_GPIO_Init();
  USER_ADC_Init();
  USER_ADC_Calibration();

  LCD_Init();
  LCD_Cursor_ON();
  LCD_Clear();
  LCD_Set_Cursor(1,0);
  LCD_Put_Str("DEMO ADC");
  LCD_Set_Cursor(2,0);
  LCD_Put_Str("TE2003B");

  HAL_Delay(2000);
  LCD_Clear();

  LCD_Set_Cursor(1,0);
  LCD_Put_Str("Lectura ADC:");

  ADC1->CR2	|=	 ADC_CR2_ADON;
  while (1){

      dataADC = USER_ADC_Read();

      float converted = 3.3*(dataADC/((pow(2,12)-1)));
      int intpart = floor(converted);
      int floatpart = (converted - intpart) * 100;

      LCD_Set_Cursor(2,0);
      LCD_Put_Str("       ");
      LCD_Set_Cursor(2,0);
      LCD_Put_Num(intpart);
      LCD_Put_Str(".");
      LCD_Put_Num(floatpart);
      LCD_Put_Str(" V");
  }
}

void SystemClock_Config(void){
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){Error_Handler();}

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK){Error_Handler();}
}

static void MX_GPIO_Init(void){
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

void USER_RCC_Init(void){
	RCC->APB2ENR	|=	 RCC_APB2ENR_IOPAEN
		      	|	 RCC_APB2ENR_ADC1EN;
	RCC->CFGR	|=	 RCC_CFGR_ADCPRE;
}
void USER_GPIO_Init(void){
	GPIOA->CRL	&=	~GPIO_CRL_CNF0 & ~GPIO_CRL_MODE0;
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

void Error_Handler(void){
  __disable_irq();
  while (1){}
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line){}
#endif
