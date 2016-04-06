/**
  ******************************************************************************
  * @file    Examples/GPIOToggle/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    09/13/2010
  * @brief   Main program body.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_conf.h"
#include "EUROBOT_Init.h"

int main(void)
{

    /* Inicijalizacija trigger-a. */
  
  InitGPIOPin(GPIOB, GPIO_Pin_10, GPIO_Mode_AF_PP, GPIO_Speed_50MHz); // channel 3 tim2
  InitGPIOPin(GPIOB, GPIO_Pin_11, GPIO_Mode_AF_PP, GPIO_Speed_50MHz); // channel 4 tim2
  GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM2, ENABLE);
  
  
  /* Postavljanje u INfloating pinova koje ne koristim, a koji mogu da generisu prekide slucajno */
  
  InitGPIOPin(GPIOA, GPIO_Pin_0, GPIO_Mode_AIN, GPIO_Speed_50MHz); // channel 1 tim2
  InitGPIOPin(GPIOA, GPIO_Pin_1, GPIO_Mode_AIN, GPIO_Speed_50MHz); // channel 2 tim2
  InitGPIOPin(GPIOA, GPIO_Pin_6, GPIO_Mode_AIN, GPIO_Speed_50MHz); // channel 1 tim3
  InitGPIOPin(GPIOA, GPIO_Pin_7, GPIO_Mode_AIN, GPIO_Speed_50MHz); // channel 2 tim3
  InitGPIOPin(GPIOB, GPIO_Pin_6, GPIO_Mode_AIN, GPIO_Speed_50MHz); // channel 1 tim4
  InitGPIOPin(GPIOB, GPIO_Pin_7, GPIO_Mode_AIN, GPIO_Speed_50MHz); // channel 2 tim4
  
  
   //Perioda izlaznog signala je 75 ms sa korakom 5us.   -- pwm u razlicitim trenucima
  InitTIM_TimeBase(TIM2, 120 - 1, 15000 - 1, TIM_CounterMode_Up, TIM_CKD_DIV1, 0x00);
  InitTIM_OC(TIM2, TIM_Channel_3, TIM_OutputState_Enable, TIM_OCMode_PWM1, 2, TIM_OCPolarity_High);  //edge-aligned
  InitTIM_TimeBase(TIM2, 120 - 1, 15000 - 1, TIM_CounterMode_Up, TIM_CKD_DIV1, 0x00);
  InitTIM_OC(TIM2, TIM_Channel_4, TIM_OutputState_Enable, TIM_OCMode_PWM2, 2, TIM_OCPolarity_High); // centre-aligned 
  
  /* Inicijalizacija za prijem echo-a. */
 
  InitGPIOPin(GPIOB, GPIO_Pin_0, GPIO_Mode_IN_FLOATING, GPIO_Speed_50MHz); // channel 3 tim3
  InitGPIOPin(GPIOB, GPIO_Pin_1, GPIO_Mode_IN_FLOATING, GPIO_Speed_50MHz); // channel 4 tim3
  InitGPIOPin(GPIOB, GPIO_Pin_8, GPIO_Mode_IN_FLOATING, GPIO_Speed_50MHz); // channel 3 tim4
  InitGPIOPin(GPIOB, GPIO_Pin_9, GPIO_Mode_IN_FLOATING, GPIO_Speed_50MHz); // channel 4 tim4
  
  // Perioda je 5 ms sa korakom 1 us. 
  
  //TIM3
  InitTIM_TimeBase(TIM3, 24 - 1, 5000 - 1, TIM_CounterMode_Up, TIM_CKD_DIV1, 0xFF);
  InitTIM_IC(TIM3, TIM_Channel_3, TIM_ICSelection_DirectTI, TIM_ICPSC_DIV1, 0x0, TIM_ICPolarity_Rising); // bilo je 0x000A
  InitTIM_IC(TIM3, TIM_Channel_4, TIM_ICSelection_DirectTI, TIM_ICPSC_DIV1, 0x0, TIM_ICPolarity_Rising);
  
  //TIM4
  InitTIM_TimeBase(TIM4, 24 - 1, 5000 - 1, TIM_CounterMode_Up, TIM_CKD_DIV1, 0xFF);
  InitTIM_IC(TIM4, TIM_Channel_3, TIM_ICSelection_DirectTI, TIM_ICPSC_DIV1, 0x0, TIM_ICPolarity_Rising);
  InitTIM_IC(TIM4, TIM_Channel_4, TIM_ICSelection_DirectTI, TIM_ICPSC_DIV1, 0x0, TIM_ICPolarity_Rising);
  
  
  // Trigger za interrupt nam dolazi od ulaza na prvi kanal a gore smo vec definisali da se gleda samo
  // uzlazna ivica ovog signala pa ce ona generisati interrupt.
  TIM_SelectInputTrigger(TIM4, TIM_TS_TI1FP1); // TIM_TS_TI1FP1 znaci Filtered Timer Input 1
  TIM_SelectInputTrigger(TIM3, TIM_TS_TI1FP1);
  
  // Definise sta se desava posle detekcije interrupt-a. U ovom slucaju brojac se resetuje.
  TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset); 
  TIM_SelectMasterSlaveMode(TIM4,TIM_MasterSlaveMode_Enable);
  
  TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset); 
  TIM_SelectMasterSlaveMode(TIM3,TIM_MasterSlaveMode_Enable);

  
  // Osposobljava se generisanje prekida od strane kanala tim2, tim3, tim4 i njihovih kanala 3 i 4, sto se desava na obe ivice
  TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);
  TIM_ITConfig(TIM3, TIM_IT_CC4, ENABLE);
  TIM_ITConfig(TIM4, TIM_IT_CC3, ENABLE);
  TIM_ITConfig(TIM4, TIM_IT_CC4, ENABLE);
  
  // Podesavanje NVIC-a.
  InitNVICChannel(TIM3_IRQn, 0, 0, ENABLE);
  InitNVICChannel(TIM4_IRQn, 0, 0, ENABLE);
  
 while (1) {
  }
}

