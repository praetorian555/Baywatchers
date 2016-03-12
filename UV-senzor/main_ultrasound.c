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
  
  InitGPIOPin(GPIOB, GPIO_Pin_9, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
  
  // Perioda izlaznog signala je 75 ms sa korakom 5us.
  InitTIM_TimeBase(TIM4, 120 - 1, 15000 - 1, TIM_CounterMode_Up, TIM_CKD_DIV1, 0x00);
  InitTIM_OC(TIM4, TIM_Channel_4, TIM_OutputState_Enable, TIM_OCMode_PWM1, 2, TIM_OCPolarity_High);
  
  /* Inicijalizacija za prijem echo-a. */
  
  InitGPIOPin(GPIOA, GPIO_Pin_8, GPIO_Mode_IPU, GPIO_Speed_50MHz);
  
  // Perioda je 5 ms sa korakom 1 us.
  InitTIM_TimeBase(TIM1, 24 - 1, 5000 - 1, TIM_CounterMode_Up, TIM_CKD_DIV1, 0xFF);
  InitTIM_IC(TIM1, TIM_Channel_1, TIM_ICSelection_DirectTI, TIM_ICPSC_DIV1, 0x0, TIM_ICPolarity_Rising);
  InitTIM_IC(TIM1, TIM_Channel_2, TIM_ICSelection_IndirectTI, TIM_ICPSC_DIV1, 0x0, TIM_ICPolarity_Falling);
  
  // Trigger za interrupt nam dolazi od ulaza na prvi kanal a gore smo vec definisali da se gleda samo
  // uzlazna ivica ovog signala pa ce ona generisati interrupt.
  TIM_SelectInputTrigger(TIM1, TIM_TS_TI1FP1);
  
  // Definise sta se desava posle detekcije interrupt-a. U ovom slucaju brojac se resetuje.
  TIM_SelectSlaveMode(TIM1, TIM_SlaveMode_Reset); 
  TIM_SelectMasterSlaveMode(TIM1,TIM_MasterSlaveMode_Enable);
  
  // Osposobljava se generisanje prekida od strane kanala 2. Prekid ce se generisati kad se izvrsi
  // capture vrednosti u CCR2. Gore smo podesili da se to desava na silaznu ivicu ulaznog signala.
  TIM_ITConfig(TIM1, TIM_IT_CC2, ENABLE);
  
  // Podesavanje NVIC-a.
  InitNVICChannel(TIM1_CC_IRQn, 0, 0, ENABLE);
  
 while (1) {
  }
}

