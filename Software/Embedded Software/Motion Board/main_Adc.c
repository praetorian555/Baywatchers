#include "stm32F10x.h"
#include "STM32vldiscovery.h"

uint32_t readADCValue(void);


  //Deklaracija struktura za razne periferije
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
  TIM_OCInitTypeDef TIM_OCInitStruct;
  NVIC_InitTypeDef NVIC_InitStructure;  
  EXTI_InitTypeDef EXTI_InitStructure;
  ADC_InitTypeDef ADC_InitStructure;

    
      __IO uint32_t ADCValue;
 
int main(void)
{
    //Dovodimo klok signal periferijama
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7|RCC_APB1Periph_TIM2,ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div2);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOC, ENABLE);
    //RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE ); //ovo ne treba za ADC
     
    GPIO_InitTypeDef GPIO_InitStructure;//inicijalizacija porta - izbor analognog ulaza
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;//analogni ulaz je kanal CH14 koji se nalazi na pinu PC.04
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//inicijalizacija ADC-a
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 1, ADC_SampleTime_1Cycles5);//inicijalizacija kanala
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);//kalibracija
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));  
    ADC_ExternalTrigConvCmd(ADC1,ENABLE);  

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = ADC1_IRQn;//jedini dozvoljeni prekid je prekid tajmera TIM7
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    TIM_DeInit(TIM2);
    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
    TIM_TimeBaseInitStruct.TIM_Period = 150 - 1;
    TIM_TimeBaseInitStruct.TIM_Prescaler = 99;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    //TIM_Cmd(TIM2,ENABLE);
    
     
    //Inicijalizacija OC strukture tajmera TIM2
    TIM_OCStructInit( &TIM_OCInitStruct );
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    //Initial duty cycle equals 0%. Value can range from zero to 1000.
    TIM_OCInitStruct.TIM_Pulse = 0; // 0 .. 1000 (0=Always Off, 1000=Always On) 
    TIM_OC2Init( TIM2, &TIM_OCInitStruct ); // Channel 3 Blue LED
    TIM_Cmd( TIM2, ENABLE );//startovanje tajmera
    
    /*TIM_DeInit(TIM7);//inicijalizacija BASIC TIMER TIM7
    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
    TIM_TimeBaseInitStruct.TIM_Period = 150 - 1;
    TIM_TimeBaseInitStruct.TIM_Prescaler = 0;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseInitStruct);
    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM7,ENABLE);*/
    
    TIM2->CCR2=0x44;
    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
   
    while(1)
    {
    }
}



uint32_t readADCValue(void)
{
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  /* Wait until conversion completion*/
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
  return ADC_GetConversionValue(ADC1);
}




