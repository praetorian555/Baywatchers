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
#include "Communication.h"
  

/** @addtogroup Examples
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define MOTION_DEVICE_ADDRESS ( 0x0A )
    
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
    
int global_tick = 0;
int cnt_90 = 0;
bool FLAG_stop = FALSE;

/* GLobal variables ---------------------------------------------------------*/

extern char sending_array[];
extern int sending_iterator;
extern int sending_length;
uint16_t received_byte;

extern bool flag_go_to_stop;
/* Private function prototypes -----------------------------------------------*/

void sleep(int);
void BateryDisp (void);

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



/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/

/**
  * @brief  This function handles External line0 interrupt request.
  * @param  None
  * @retval None
  */

void TIM7_IRQHandler(void)
{
  if( TIM_GetITStatus( TIM7, TIM_IT_Update ) == SET )
  {
    TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
    if( cnt_90 == 10 )
      TIM3->CCR3 = 950;
    else cnt_90++;
  }
}

/**
  * @brief  Obradjuje prekide pristigle od nekog kanala tajmera 2.
  * @param  None
  * @retval None
  */

void SysTick_Handler(void) // tajmer koji nam broji 100 
{ 
  if (global_tick>0) global_tick--;
}

/**
  * @brief  Obradjuje prekide pristigle od nekog kanala tajmera 3.
  * @param  None
  * @retval None
  */

void TIM3_IRQHandler(void) // tajmer koji nam broji 90s 
{ 
  if( TIM_GetITStatus( TIM3, TIM_IT_Update ) == SET )
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    BateryDisp();
  }
}

void USART3_IRQHandler( void )
{  
  /* Prijem poruke. */
  if( ( USART_GetITStatus( USART3, USART_IT_RXNE ) != RESET ) )
  {
    receiveByte( USART_ReceiveData( USART3 ) );
  }
  
  /* Slanje poruke. */
  else if ( USART_GetITStatus( USART3, USART_IT_TC ) != RESET )
  {
    USART_ClearITPendingBit( USART3, USART_IT_TC );
    
    /* Salju se bajtovi poruke. */
    if ( sending_iterator < sending_length ) USART_SendData( USART3, sending_array[ sending_iterator++ ] );
    
    /* Kraj slanja poruke. */
    else GPIO_ResetBits( GPIOC, GPIO_Pin_12 ); // Iskljucivanje pristupa magistrali.
  }
  
  /* Ako se detektuje overrun greska. */
  else
  {
    USART_ITConfig( USART3, USART_IT_TC, DISABLE );
    received_byte = USART_ReceiveData( USART3 );
    USART3->SR = 0x00000000;//D8
    USART_ClearITPendingBit( USART3, USART_IT_TXE );
    USART_ITConfig( USART3, USART_IT_TC, ENABLE );
  }
}


/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
