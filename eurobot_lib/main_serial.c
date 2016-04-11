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
#include "stm32f10x.h"
#include "STM32vldiscovery.h"

#include "EUROBOT_Init.h"
#include "EUROBOT_serial.h"
    
/* User defined function prototypes */
void led_toggle(void);

volatile int commandFlag = 0;


void DecodeCommand(){
  commandFlag = 1;
}

/*******************************************
 * Main program 
 *******************************************/
int main(void)
{
    // Init Pin C8 za LED
    InitGPIOPin(GPIOC, GPIO_Pin_8, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);    
    initEurobotRS485(9600, 0xBB, 0xAA, 0);
       
    int i = 0;
    
    while(1)
    {
      
      i++;
      if(i == 5000000)
      {
          SendMessage(0xAA, "Hello World!!!");
          led_toggle();
		  i = 0;
      }
      
      // Prijem poruke
      if(commandFlag == 1)
      {
          commandFlag = 0;
      }
 
      
    }
} 
 
/*******************************************
 * Toggle LED 
 *******************************************/
void led_toggle(void)
{
    /* Read LED output (GPIOA PIN8) status */
    uint8_t led_bit = GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_8);
     
    /* If LED output set, clear it */
    if(led_bit == (uint8_t)Bit_SET)
    {
        GPIO_ResetBits(GPIOC, GPIO_Pin_8);
    }
    /* If LED output clear, set it */
    else
    {
        GPIO_SetBits(GPIOC, GPIO_Pin_8);
    }
}