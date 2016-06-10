/**
  ******************************************************************************
  * @file    Examples/GPIOToggle/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    15/09/2010
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
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
#include "stm32f10x_it.h"
#include "STM32vldiscovery.h"

/** @addtogroup Examples
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

  //TIM3

  /*ch3*/
  uint8_t index_3_ch3 = 0;         // Predstavlja broj do sada prikupljenih izmerenih udaljnosti na osnovu pristiglih impulsa.
  double distance_3_ch3 = 0;       // Predstavlja izracunatu udaljenost od objekta.
  uint16_t measured_distance_3_ch3[20]; // Niz u koji se smestaju izmerene udaljenosti za 20 impulsa.
  uint16_t measured_time_3uzlazna_ch3;
  uint16_t measured_time_3silazna_ch3;
  uint16_t measured_time3_ch3;
  int state_tim3_ch3 = 0;    // 0 ako se hvata uzlazna ivica, 1 ako se hvata silazna ivica
  /*ch4*/
  uint8_t index_3_ch4 = 0;         // Predstavlja broj do sada prikupljenih izmerenih udaljnosti na osnovu pristiglih impulsa.
  double distance_3_ch4 = 0;       // Predstavlja izracunatu udaljenost od objekta.
  uint16_t measured_distance_3_ch4[20]; // Niz u koji se smestaju izmerene udaljenosti za 20 impulsa.
  uint16_t measured_time_3uzlazna_ch4;
  uint16_t measured_time_3silazna_ch4;
  uint16_t measured_time3_ch4;
  int state_tim3_ch4 = 0;    // 0 ako se hvata uzlazna ivica, 1 ako se hvata silazna ivica
  
  //TIM4

  /*ch3*/
  uint8_t index_4_ch3 = 0;         // Predstavlja broj do sada prikupljenih izmerenih udaljnosti na osnovu pristiglih impulsa.
  double distance_4_ch3 = 0;       // Predstavlja izracunatu udaljenost od objekta.
  uint16_t measured_distance_4_ch3[20]; // Niz u koji se smestaju izmerene udaljenosti za 20 impulsa.
  uint16_t measured_time_4uzlazna_ch3;
  uint16_t measured_time_4silazna_ch3;
  uint16_t measured_time4_ch3;
  int state_tim4_ch3 = 0;    // 0 ako se hvata uzlazna ivica, 1 ako se hvata silazna ivica
  /*ch4*/
  uint8_t index_4_ch4 = 0;         // Predstavlja broj do sada prikupljenih izmerenih udaljnosti na osnovu pristiglih impulsa.
  double distance_4_ch4 = 0;       // Predstavlja izracunatu udaljenost od objekta.
  uint16_t measured_distance_4_ch4[20]; // Niz u koji se smestaju izmerene udaljenosti za 20 impulsa.
  uint16_t measured_time_4uzlazna_ch4;
  uint16_t measured_time_4silazna_ch4;
  uint16_t measured_time4_ch4;
  int state_tim4_ch4 = 0;    // 0 ako se hvata uzlazna ivica, 1 ako se hvata silazna ivica
  
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */



/**
  * @brief  Obradjuje prekide pristigle od nekog kanala tajmera 3.
  * @param  None
  * @retval None
  */

void TIM3_IRQHandler(void)
{
  
  // Provera da li je stigao zahtev za prekid od kanala 2.
  if (TIM_GetITStatus(TIM3, TIM_IT_CC3) == SET)
  {
    // Brisanje pending bita ovog zahteva za prekid.
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
    
    switch(state_tim3_ch3)
    {
    case 0: 
      {
        measured_time_3uzlazna_ch3 = TIM_GetCapture3(TIM3);
        InitTIM_IC(TIM3, TIM_Channel_3, TIM_ICSelection_DirectTI, TIM_ICPSC_DIV1, 0x0, TIM_ICPolarity_Falling);
        state_tim3_ch3 = 1;
        break;
      }
     case 1:
       {
        measured_time_3silazna_ch3 = TIM_GetCapture3(TIM3);
        InitTIM_IC(TIM3, TIM_Channel_3, TIM_ICSelection_DirectTI, TIM_ICPSC_DIV1, 0x0, TIM_ICPolarity_Rising);
        state_tim3_ch3 = 0;
       }
     default: break; 
    }
    
    measured_time3_ch3 = measured_time_3silazna_ch3 - measured_time_3uzlazna_ch3;
    
    if (index_3_ch3 == 20)
    {
      int temp_distance = 0;
      for (int i = 0; i < index_3_ch3; i++) temp_distance += measured_distance_3_ch3[i];
      distance_3_ch3 = temp_distance/ 20;
      index_3_ch3 = 0;
    }
    
    // Ako se nije nakupilo 20 merenja, nastavlja se prikupljanje merenja.
    else measured_distance_3_ch3[index_3_ch3++] = measured_time3_ch3/58;
  }
  else if (TIM_GetITStatus(TIM3, TIM_IT_CC4) == SET)
  {
    // Brisanje pending bita ovog zahteva za prekid.

    TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
    
    switch(state_tim3_ch4)
    {
    case 0: 
      {
        measured_time_3uzlazna_ch4 = TIM_GetCapture4(TIM3);
        InitTIM_IC(TIM3, TIM_Channel_4, TIM_ICSelection_DirectTI, TIM_ICPSC_DIV1, 0x0, TIM_ICPolarity_Falling);
        state_tim3_ch4 = 1;
        break;
      }
     case 1:
       {
        measured_time_3silazna_ch4 = TIM_GetCapture4(TIM3);
        InitTIM_IC(TIM3, TIM_Channel_4, TIM_ICSelection_DirectTI, TIM_ICPSC_DIV1, 0x0, TIM_ICPolarity_Rising);
        state_tim3_ch4 = 0;
       }
     default: break; 
    }
    
    measured_time3_ch4 = measured_time_3silazna_ch4 - measured_time_3uzlazna_ch4;
    
    if (index_3_ch4 == 20)
    {
      int temp_distance = 0;
      for (int i = 0; i < index_3_ch4; i++) temp_distance += measured_distance_3_ch4[i];
      distance_3_ch4 = temp_distance/ 20;
      index_3_ch4 = 0;
    }
    
    // Ako se nije nakupilo 20 merenja, nastavlja se prikupljanje merenja.
    else measured_distance_3_ch4[index_3_ch4++] = measured_time3_ch4/58;
    
  }
}

/**
  * @brief  Obradjuje prekide pristigle od nekog kanala tajmera 4.
  * @param  None
  * @retval None
  */

void TIM4_IRQHandler(void)
{
  
  // Provera da li je stigao zahtev za prekid od kanala 2.
  if (TIM_GetITStatus(TIM4, TIM_IT_CC3) == SET)
  {
    // Brisanje pending bita ovog zahteva za prekid.
    TIM_ClearITPendingBit(TIM4, TIM_IT_CC3);
    
    switch(state_tim4_ch3)
    {
    case 0: 
      {
        measured_time_4uzlazna_ch3 = TIM_GetCapture3(TIM4);
        InitTIM_IC(TIM4, TIM_Channel_3, TIM_ICSelection_DirectTI, TIM_ICPSC_DIV1, 0x0, TIM_ICPolarity_Falling);
        state_tim4_ch3 = 1;
        break;
      }
     case 1:
       {
        measured_time_4silazna_ch3 = TIM_GetCapture3(TIM4);
        InitTIM_IC(TIM4, TIM_Channel_3, TIM_ICSelection_DirectTI, TIM_ICPSC_DIV1, 0x0, TIM_ICPolarity_Rising);
        state_tim4_ch3 = 0;
       }
     default: break; 
    }
    
    measured_time4_ch3 = measured_time_4silazna_ch3 - measured_time_4uzlazna_ch3;
    
    if (index_4_ch3 == 20)
    {
      int temp_distance = 0;
      for (int i = 0; i < index_4_ch3; i++) temp_distance += measured_distance_4_ch3[i];
      distance_4_ch3 = temp_distance/ 20;
      index_4_ch3 = 0;
    }
    
    // Ako se nije nakupilo 20 merenja, nastavlja se prikupljanje merenja.
    else measured_distance_4_ch3[index_4_ch3++] = measured_time4_ch3/58;
  }
  else if (TIM_GetITStatus(TIM4, TIM_IT_CC4) == SET)
  {
    // Brisanje pending bita ovog zahteva za prekid.

    TIM_ClearITPendingBit(TIM4, TIM_IT_CC4);
    
    switch(state_tim4_ch4)
    {
    case 0: 
      {
        measured_time_4uzlazna_ch4 = TIM_GetCapture4(TIM4);
        InitTIM_IC(TIM4, TIM_Channel_4, TIM_ICSelection_DirectTI, TIM_ICPSC_DIV1, 0x0, TIM_ICPolarity_Falling);
        state_tim4_ch4 = 1;
        break;
      }
     case 1:
       {
        measured_time_4silazna_ch4 = TIM_GetCapture4(TIM3);
        InitTIM_IC(TIM4, TIM_Channel_4, TIM_ICSelection_DirectTI, TIM_ICPSC_DIV1, 0x0, TIM_ICPolarity_Rising);
        state_tim4_ch4 = 0;
       }
     default: break; 
    }
    
    measured_time4_ch4 = measured_time_4silazna_ch4 - measured_time_4uzlazna_ch4;
    
    if (index_4_ch4 == 20)
    {
      int temp_distance = 0;
      for (int i = 0; i < index_4_ch4; i++) temp_distance += measured_distance_4_ch4[i];
      distance_4_ch4 = temp_distance/ 20;
      index_4_ch4 = 0;
    }
    
    // Ako se nije nakupilo 20 merenja, nastavlja se prikupljanje merenja.
    else measured_distance_4_ch4[index_4_ch4++] = measured_time4_ch4/58;
    
  }
}


/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/

/**
  * @brief  This function handles External line0 interrupt request.
  * @param  None
  * @retval None
  */

void EXTI0_IRQHandler(void)
{
}


/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
