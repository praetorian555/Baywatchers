
#include "UartDebug.h"
#include "print.h"
#include "stm32f10x.h"
#include "STM32vldiscovery.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "misc.h"

void DelayUSART(int);

char string[32];
unsigned short iterator = 0;

// -------------------------------------------------------------------------------------------------------------------------

void UsartInit ()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    USART_ClockInitTypeDef USART_ClockInitStructure;
 
        // Enable APB2 bus clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    
    DelayUSART(2);
    
    //Set USART1 Tx (PB.10) as AF push-pull --> PC10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    //Set USART1 Rx (PB.11) as input floating --> PC11
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    USART_ClockStructInit(&USART_ClockInitStructure);
    USART_ClockInit(USART3, &USART_ClockInitStructure);
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    
    //Write USART3 parameters
    USART_Init(USART3, &USART_InitStructure);
    //Enable USART3
    USART_Cmd(USART3, ENABLE);
    
        //configure NVIC
    NVIC_InitTypeDef NVIC_InitStructure;
    //select NVIC channel to configure
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    //set priority to lowest
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    //set subpriority to lowest
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    //enable IRQ channel
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //update NVIC registers
    NVIC_Init(&NVIC_InitStructure);
    //disable Transmit Data Register empty interrupt
  // USART_ITConfig(USART3, USART_IT_TXE, DISABLE);  // proveri!
    //enable Receive Data register not empty interrupt
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART3, USART_IT_TC, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;            // konfiguracija za alternativnu funkciju - Push Pull
    GPIO_Init( GPIOC, &GPIO_InitStructure );
    
    GPIO_ResetBits(GPIOC,GPIO_Pin_12); 
 
    GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);

}


// -------------------------------------------------------------------------------------------------------------------------

void UsartPut (uint8_t ch)
{
    USART_SendData(USART3, (uint8_t) ch);
    //Loop until the end of transmission
    while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
}

//-------------------------------------------------------------------------------------------------------------------------
//ADDED BY VLADIMIR 

char UsartGet(void){
	while ( USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == RESET);
	return (char)USART_ReceiveData(USART3);
}


// -------------------------------------------------------------------------------------------------------------------------

void Send (const char *buffer, unsigned count)
{    
    while (count--)                  // Loop while there are more characters to send.
        UsartPut( *buffer++ );      // Write the next character to the UART.
}

// -------------------------------------------------------------------------------------------------------------------------

void Clear ()
{
    unsigned int i;
    for (i = 0; i < 32; i++)
        string[i] = '\0';    
}

// -------------------------------------------------------------------------------------------------------------------------

void SendString ()
{
    unsigned int i = 0;
    while ( string[i] != '\0' )
    {
        UsartPut ( string[i] );
        i++;
    }        
}

void DelayUSART(int nCount)
{
  for(; nCount != 0; nCount--);
}
