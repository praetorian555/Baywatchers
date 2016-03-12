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
  uint8_t index = 0;         // Predstavlja broj do sada prikupljenih izmerenih udaljnosti na osnovu pristiglih impulsa.
  double distance = 0;       // Predstavlja izracunatu udaljenost od objekta.
  uint16_t measured_distance[20]; // Niz u koji se smestaju izmerene udaljenosti za 20 impulsa.

  
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

void TIM2_IRQHandler(void)
{ 
}

/**
  * @brief  Obradjuje prekide pristigle od nekog kanala tajmera 1.
  * @param  None
  * @retval None
  */
void TIM1_CC_IRQHandler(void)
{
  
  // Provera da li je stigao zahtev za prekid od kanala 2.
  if (TIM_GetITStatus(TIM1, TIM_IT_CC2) == SET)
  {
    // Brisanje pending bita ovog zahteva za prekid.
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);
    
    // Dohvatanje sadrzaja registra CCR2 koji sadrzi sirinu pristiglog impulsa.
    uint16_t measured_time = TIM_GetCapture2(TIM1);
    
    // Odredjivanje srednje vrednosti udaljenosti na osnovu 20 izmerenih udaljenosti.
    if (index == 20)
    {
      int temp_distance = 0;
      for (int i = 0; i < index; i++) temp_distance += measured_distance[i];
      distance = temp_distance / 20;
      index = 0;
    }
    
    // Ako se nije nakupilo 20 merenja, nastavlja se prikupljanje merenja.
    else measured_distance[index++] = measured_time/58;
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
