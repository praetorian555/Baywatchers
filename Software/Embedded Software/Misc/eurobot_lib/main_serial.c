/**
  ******************************************************************************
  * @file    main_serial.c 
  * @author  Jovan Blanuša
  * @version V1.0.0
  * @date    11/04/2015
  * @brief   Glavni program za serijsku komunikaciju.
  ******************************************************************************
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