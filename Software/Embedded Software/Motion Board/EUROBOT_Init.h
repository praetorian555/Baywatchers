/** 
*   @file:    EUROBOT_Init.h
*   @author:  Cuvari plaze(Praetorian)
*   @version: v1.01.030816.0348 (Praetorian)
*   @date:    03/08/2016
*   @brief:   Deklaracije funkcija za inicijalizaciju periferija.
*/

#ifndef __EUROBOT_INIT_H__
#define __EUROBOT_INIT_H__

/* Funkcije za inicijalizaciju periferija */

// Inicijalizacija pina.
void InitGPIO_Pin(GPIO_TypeDef *GPIOx, uint16_t pin, GPIOMode_TypeDef mode, GPIOSpeed_TypeDef speed);
// Inicijalizacija tajmera.
void InitTIM_TimeBase(TIM_TypeDef *TIMx, uint16_t prescaler, uint16_t period, uint16_t counter_mode, uint16_t clock_division, uint8_t repetition_counter);
// Inicijalizacija kanala tajmera za output mode.
void InitTIM_OC(TIM_TypeDef *TIMx, uint16_t channel, uint16_t output_state, uint16_t oc_mode, uint16_t pulse, uint16_t polarity);
// Inicijalizacija kanala tajmera za input mode.
void InitTIM_IC(TIM_TypeDef *TIMx, uint16_t channel, uint16_t selection, uint16_t prescaler, uint16_t filter, uint16_t polarity);
// Inicijalizacija nekog od NVIC-ovih kanala.
void InitNVICChannel(uint8_t NVIC_Channel, uint8_t NVIC_PreemptionPriority, uint8_t NVIC_SubPriority, FunctionalState NVIC_ChannelCmd);
// Inicijalizacija eksterne linije prekida.
void InitEXTI(uint8_t GPIO_PortSourceX, uint32_t Line, EXTIMode_TypeDef Mode, EXTITrigger_TypeDef Trigger, FunctionalState LineCmd);
// Inicijalizacija USART periferije.
void InitUSART(USART_TypeDef* USARTx, uint16_t Mode, uint32_t BaudRate, uint16_t WordLength, uint16_t StopBits, uint16_t Parity);
// Inicijalizacija USART periferije sa default parametrima.
void InitDefaultUSART(USART_TypeDef* USARTx, uint16_t Mode, uint32_t BaudRate);

/**
*   @brief: Tajmer pocinje ili prestaje da generise prekide tipa TIM_IT. Ova
*           funkcija je vec definisana u standarnoj biblioteci za ovaj mikrokontroler,
*           ovde je samo navedena deklaracija.
*   @param: TIMx pokazivac na zeljeni tajmer koji sadrzi adresu odgovarajuce
*           periferije.
*   @param: TIM_IT odredjuje kakav tip prekida tajmer generise.
*           Moguce vrednosti su:
*              @arg TIM_IT_Update,  generise prekid kad brojac izbroji do zadate vrednosti ili
*                                   kad se inicijalizuje tajmer (ili softverski ili od strane
*                                   nekog trigger-a.
*              @arg TIM_IT_Trigger, henerise prekid kada tajmer pocne da broji, kad stane da broji
*                                   ili kad se inicijalizuje.
*              @arg TIM_IT_Break,   TIM Break Interrupt source
*              @arg TIM_IT_CC1,     TIM Capture Compare 1 Interrupt source
*              @arg TIM_IT_CC2,     TIM Capture Compare 2 Interrupt source
*              @arg TIM_IT_CC3,     TIM Capture Compare 3 Interrupt source
*              @arg TIM_IT_CC4,     TIM Capture Compare 4 Interrupt source
*   @param: NewState odredjuje da li tajmer generise odredjen tip prekida ili ne. Po default-u svi
*           prekidi su iskljuceni.
*           Moguce vrednosti su ENABLE ili DISABLE.
*   @return: Nema povratnih vrednosti.
*
*/
void TIM_ITConfig(TIM_TypeDef *TIMx, uint16_t TIM_IT, FunctionalState NewState);

#endif