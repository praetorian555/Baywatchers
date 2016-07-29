/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_conf.h"
#include "EUROBOT_Init.h"
#include "Communication.h"

#define MOTION_DEVICE_ADDRESS ( 0x0A )

/* GLobal variables ----------------------------------------------------------*/

extern bool FLAG_arriveOnDest;
extern bool FLAG_ackReceived;
bool FLAG_strategyLeft = TRUE;
extern int global_tick;
extern uint16_t command_ID;
int state_robot = 0;

bool flag_go_to_stop;

/* Function prototype --------------------------------------------------------*/

void initTimerServo( void );

void waitAck( CommandNameType, int, uint16_t );
void initTimer90( void );
void initTimerSleep( void );
void sleep( int );
void checkStrategy( void );
void UsartInit ( void );

#define ADC1_DR_Address    ((u32)0x4001244C) 
  vu16 ADC_RegularConvertedValueTab[4];
  
  void DisplayBarGraph(unsigned char disp);//prototip funkcije, opseg 0-7
  void DisplayBarGraphBinary(unsigned char disp);//prototip funkcije, opseg 0-255
  void DisplayBarGraphOneHot(unsigned char disp);//prototip funkcije, opseg 0-8
  void BateryDisp (void);
  unsigned char test=0;
  
  float baterija_Acc, servo_5V;
  float baterija_Acc_const, servo_5V_const;
  
  //Deklaracija struktura za razne periferije
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
  TIM_OCInitTypeDef TIM_OCInitStruct;
  NVIC_InitTypeDef NVIC_InitStructure;  
  EXTI_InitTypeDef EXTI_InitStructure;
  ADC_InitTypeDef ADC_InitStructure;
  DMA_InitTypeDef   DMA_InitStructure;

int main( void )
{
  //konfiguracija taktova
  RCC_ADCCLKConfig(RCC_PCLK2_Div2);//konfigurisanje takta za ADC 
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);//dovodjenje takta za DMA kontroler 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_TIM1 | RCC_APB2Periph_ADC1, ENABLE);//dovodjenje takta portu A, B, C, tajmeru TIM1 i ADC-u
  
  //konfiguracija portova - analogni ulazi
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  //konfiguracija portova - bargraph tj. DIGIO konektor
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_5 | GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
   
  /* DMA1 channel1 configuration ----------------------------------------------*/
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;//adresa izvorista za dma prenos - DATA REGISTER ADC-a
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)ADC_RegularConvertedValueTab;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 4;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  /* Enable DMA1 channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);
  
  /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 4;
  ADC_Init(ADC1, &ADC_InitStructure);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 1, ADC_SampleTime_1Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 2, ADC_SampleTime_1Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 3, ADC_SampleTime_1Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 4, ADC_SampleTime_1Cycles5);
  ADC_ExternalTrigConvCmd(ADC1, ENABLE);//omogucavanje externog triger moda
  ADC_DMACmd(ADC1, ENABLE);//omogucavanje DMA prenosa za ADC
  ADC_Cmd(ADC1, ENABLE);
  
  ADC_ResetCalibration(ADC1);//adc kalibracija
  while(ADC_GetResetCalibrationStatus(ADC1));
  ADC_StartCalibration(ADC1);
  while(ADC_GetCalibrationStatus(ADC1));
 
  //konfiguracija tajmera TIM1 koji radi u PWM modu, i svoj izlaz koristi za trigerovanje ADC-a
  TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
  TIM_TimeBaseInitStruct.TIM_Period = 150 - 1;
  TIM_TimeBaseInitStruct.TIM_Prescaler = 0;
  TIM_TimeBaseInitStruct.TIM_ClockDivision = 0x0;
  TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStruct); 
  TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStruct.TIM_Pulse = 150 / 2;
  TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OC1Init(TIM1, &TIM_OCInitStruct);
  
  TIM_Cmd(TIM1, ENABLE);//dozovla rada tajmera tek kada se konfigurisu i DMA i ADC
  TIM_CtrlPWMOutputs(TIM1, ENABLE);//generisanje PWM izlaza za tajmer 1
  
  baterija_Acc_const=0.004032;
  servo_5V_const=0.004032;
  
  
  
  
    
  
  /* Inicijalizacija. */
  InitGPIO_Pin(GPIOB, GPIO_Pin_11, GPIO_Mode_IPU, GPIO_Speed_50MHz);
  UsartInit();
  
  /* Inicijalizacija glavnog tajmera. */
  initTimerServo();//zbog ovoga se baterija meri i dok je prekidac uvucen
  
  /* Glavna masina stanja. */
  while(1)
  {
    switch (state_robot)
    {
      /* Pocetno stanje u kome se inicijalizaciju sistemi, podesava preskaler, ukljucuje UV. */ 
      case 0:  // pocetno stanje, sve inicijalizujemo i krenemo napred
        
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11)) // ako je ocitano Vcc, tj. krene se sa izvrsavanjem, u suprotnom ostajemo u 
                                                       // istom stanju
        {
          /* Inicijalizacija glavnog tajmera. */
          initTimer90();
          /* Inicijalizacija tajmera za proveru pozicije robota. */
          SysTick_Config( SysTick_Config( SystemCoreClock / 1000 ) );
          /* Provera koja je stategije. */
          checkStrategy();
          /* Zadavanje komandi. */
          issueCommand( START_RUNNING, MOTION_DEVICE_ADDRESS, 30 );
          waitAck( START_RUNNING, MOTION_DEVICE_ADDRESS, 30 );
          issueCommand( PRESCALER, MOTION_DEVICE_ADDRESS, 1000 );
          waitAck( PRESCALER, MOTION_DEVICE_ADDRESS, 1000 );
          issueCommand( ULTRASOUND_ON, MOTION_DEVICE_ADDRESS, 1 );
          waitAck( ULTRASOUND_ON, MOTION_DEVICE_ADDRESS, 1 );
          issueCommand( MOVE_FORWARD, MOTION_DEVICE_ADDRESS, 30 );
          waitAck( MOVE_FORWARD, MOTION_DEVICE_ADDRESS, 30 );
          while( TRUE )
          {
            issueCommand( CHECK_ARRIVE, MOTION_DEVICE_ADDRESS, 1 );
            sleep( 25 );            
            if ( FLAG_arriveOnDest ) break;
            sleep( 100 );
          }
          state_robot++;
          sleep(100);
        }
        break;
      
      /* Blago okretanje da bi se izbegla ivica na sredini terena. */  
      case 1:
        
        if( FLAG_strategyLeft )
        {
          issueCommand( ROTATE_LEFT, MOTION_DEVICE_ADDRESS, 20 );
          waitAck( ROTATE_LEFT, MOTION_DEVICE_ADDRESS, 20 );
          while( TRUE )
          {
            issueCommand( CHECK_ARRIVE, MOTION_DEVICE_ADDRESS, 1 );
            sleep( 25 );
            if ( FLAG_arriveOnDest ) break;
            sleep( 100 );
          }
        }
        else
        {
          issueCommand( ROTATE_RIGHT, MOTION_DEVICE_ADDRESS, 20 );
          waitAck( ROTATE_RIGHT, MOTION_DEVICE_ADDRESS, 20 );
          while( TRUE )
          {
            issueCommand( CHECK_ARRIVE, MOTION_DEVICE_ADDRESS, 1 );
            sleep( 25 );
            if ( FLAG_arriveOnDest ) break;
            sleep( 100 );
          }
        }
        
        state_robot++;
        sleep(100);
        break;
       
      /* Blago pomeranje napred, ka sredini terena. */
      case 2:
      {
        issueCommand( MOVE_FORWARD, MOTION_DEVICE_ADDRESS, 50 );
        waitAck( MOVE_FORWARD, MOTION_DEVICE_ADDRESS, 50 );
        while( TRUE )
        {
          issueCommand( CHECK_ARRIVE, MOTION_DEVICE_ADDRESS, 1 );
          sleep( 100 );
          if ( FLAG_arriveOnDest ) break;          
        }
        state_robot++;
        sleep(100);
        break;       
      }
        
      /* Blaga rotacija da bi se poravnali opet. */  
      case 3:

        if( FLAG_strategyLeft )
        {
          issueCommand( ROTATE_RIGHT, MOTION_DEVICE_ADDRESS, 25 );
          waitAck( ROTATE_RIGHT, MOTION_DEVICE_ADDRESS, 25 );
          while( TRUE )
          {
            issueCommand( CHECK_ARRIVE, MOTION_DEVICE_ADDRESS, 1 );
            sleep( 100 );
            if ( FLAG_arriveOnDest ) break;
          }
        }
        else
        {
          issueCommand( ROTATE_LEFT, MOTION_DEVICE_ADDRESS, 25 );
          waitAck( ROTATE_LEFT, MOTION_DEVICE_ADDRESS, 25 );
          while( TRUE )
          {
            issueCommand( CHECK_ARRIVE, MOTION_DEVICE_ADDRESS, 1 );
            sleep( 100 );
            if ( FLAG_arriveOnDest ) break;
          }          
        }
        state_robot++;
        sleep(100);
        break;

      /* Blago pomeranje napred da bi pomerili kocke u sredinu terena.  */
      case 4:
        
        issueCommand( MOVE_FORWARD, MOTION_DEVICE_ADDRESS, 10 );
        waitAck( MOVE_FORWARD, MOTION_DEVICE_ADDRESS, 10 );
        while( TRUE )
        {
          issueCommand( CHECK_ARRIVE, MOTION_DEVICE_ADDRESS, 1 );
          sleep( 100 );
          if ( FLAG_arriveOnDest ) break;
        }
        state_robot++;
        sleep(100);
        break;
        
      /* Vracanje unazad. */
      case 5:
        
        issueCommand( PRESCALER, MOTION_DEVICE_ADDRESS, 500 );
        waitAck( PRESCALER, MOTION_DEVICE_ADDRESS, 500 );
        issueCommand( MOVE_BACKWARD, MOTION_DEVICE_ADDRESS, 50 );
        waitAck( MOVE_BACKWARD, MOTION_DEVICE_ADDRESS, 50 );
        while( TRUE )
        {
          issueCommand( CHECK_ARRIVE, MOTION_DEVICE_ADDRESS, 1 );
          sleep( 100 );
          if ( FLAG_arriveOnDest ) break;
        }
        state_robot++;
        sleep(100);
        break;  
        
      /* Okretanje ka prvoj kucici, onoj daljoj od ivice terana i gasenje senzora. */
      case 6:
        
        if( FLAG_strategyLeft )
        {
          issueCommand( ROTATE_RIGHT, MOTION_DEVICE_ADDRESS, 175 );
          waitAck( ROTATE_RIGHT, MOTION_DEVICE_ADDRESS, 175 );
          while( TRUE )
          {
            issueCommand( CHECK_ARRIVE, MOTION_DEVICE_ADDRESS, 1 );
            sleep( 100 );
            if ( FLAG_arriveOnDest ) break;
          }
        }
        else
        {
          issueCommand( ROTATE_LEFT, MOTION_DEVICE_ADDRESS, 175 );
          waitAck( ROTATE_LEFT, MOTION_DEVICE_ADDRESS, 175 );
          while( TRUE )
          {
            issueCommand( CHECK_ARRIVE, MOTION_DEVICE_ADDRESS, 1 );
            sleep( 100 );
            if ( FLAG_arriveOnDest ) break;
          }          
        } 
        
        issueCommand( ULTRASOUND_OFF, MOTION_DEVICE_ADDRESS, 1 );
        waitAck( ULTRASOUND_OFF, MOTION_DEVICE_ADDRESS, 1 );
        
        state_robot++;
        sleep(100);
        break;
      
      /* Zatvaranje prvih vrata. */
      case 7:
        
        issueCommand( PRESCALER, MOTION_DEVICE_ADDRESS, 700 );
        waitAck( PRESCALER, MOTION_DEVICE_ADDRESS, 700 );
        issueCommand( MOVE_FORWARD, MOTION_DEVICE_ADDRESS, 100 );
        waitAck( MOVE_FORWARD, MOTION_DEVICE_ADDRESS, 100 );
        while( TRUE )
        {
          issueCommand( CHECK_ARRIVE, MOTION_DEVICE_ADDRESS, 1 );
          sleep( 100 );
          if ( FLAG_arriveOnDest ) break;
        }
        state_robot++;
        sleep(100);
        break;        
        
      /* Vracanje unazad. */
      case 8:
        issueCommand( ULTRASOUND_ON, MOTION_DEVICE_ADDRESS, 1 );
        waitAck( ULTRASOUND_ON, MOTION_DEVICE_ADDRESS, 1 );
        issueCommand( PRESCALER, MOTION_DEVICE_ADDRESS, 500 );
        waitAck( PRESCALER, MOTION_DEVICE_ADDRESS, 500 );
        issueCommand( MOVE_BACKWARD, MOTION_DEVICE_ADDRESS, 60 );
        waitAck( MOVE_BACKWARD, MOTION_DEVICE_ADDRESS, 60 );
        while( TRUE )
        {
          issueCommand( CHECK_ARRIVE, MOTION_DEVICE_ADDRESS, 1 );
          sleep( 100 );
          if ( FLAG_arriveOnDest ) break;
        }
        
        state_robot++;
        sleep(100);
        break;        
       
      /* Okretanje za 180 stepeni ka pocetnoj poziciji. */
      case 9:

        //issueCommand( ULTRASOUND_ON, MOTION_DEVICE_ADDRESS, 1 );
        //waitAck( ULTRASOUND_ON, MOTION_DEVICE_ADDRESS, 1 );
          
        if( FLAG_strategyLeft )
        {
          issueCommand( ROTATE_RIGHT, MOTION_DEVICE_ADDRESS, 180 );
          waitAck( ROTATE_RIGHT, MOTION_DEVICE_ADDRESS, 180 );
          while( TRUE )
          {
            issueCommand( CHECK_ARRIVE, MOTION_DEVICE_ADDRESS, 1 );
            sleep( 100 );
            if ( FLAG_arriveOnDest ) break;
          }
        }
        else
        {
          issueCommand( ROTATE_LEFT, MOTION_DEVICE_ADDRESS, 180 );
          waitAck( ROTATE_LEFT, MOTION_DEVICE_ADDRESS, 180 );
          while( TRUE )
          {
            issueCommand( CHECK_ARRIVE, MOTION_DEVICE_ADDRESS, 1 );
            sleep( 100 );
            if ( FLAG_arriveOnDest ) break;
          }          
        } 
        
        state_robot++;
        sleep(100);
        break; 
      
      /* Odlazak naspram druge kucice. */
      case 10:

        issueCommand( MOVE_FORWARD, MOTION_DEVICE_ADDRESS, 15 );
        waitAck( MOVE_FORWARD, MOTION_DEVICE_ADDRESS, 15 );
        while( TRUE )
        {
          issueCommand( CHECK_ARRIVE, MOTION_DEVICE_ADDRESS, 1 );
          sleep( 100 );
          if ( FLAG_arriveOnDest ) break;
        }
        state_robot++;
        sleep(100);
        break;             
       
      /* Okretanje ka kucici. */
      case 11:

        if( FLAG_strategyLeft )
        {
          issueCommand( ROTATE_LEFT, MOTION_DEVICE_ADDRESS, 175 );
          waitAck( ROTATE_LEFT, MOTION_DEVICE_ADDRESS, 175 );
          while( TRUE )
          {
            issueCommand( CHECK_ARRIVE, MOTION_DEVICE_ADDRESS, 1 );
            sleep( 100 );
            if ( FLAG_arriveOnDest ) break;
          }
        }
        else
        {
          issueCommand( ROTATE_RIGHT, MOTION_DEVICE_ADDRESS, 175 );
          waitAck( ROTATE_RIGHT, MOTION_DEVICE_ADDRESS, 175 );
          while( TRUE )
          {
            issueCommand( CHECK_ARRIVE, MOTION_DEVICE_ADDRESS, 1 );
            sleep( 100 );
            if ( FLAG_arriveOnDest ) break;
          }
        }
        
        issueCommand( ULTRASOUND_OFF, MOTION_DEVICE_ADDRESS, 1 );
        waitAck( ULTRASOUND_OFF, MOTION_DEVICE_ADDRESS, 1 );
        
        state_robot++;
        sleep(100);
        break;        
        
         
      /* Zatvaranje druge kucice. */
      case 12:
        
        issueCommand( ULTRASOUND_OFF, MOTION_DEVICE_ADDRESS, 1 );
        waitAck( ULTRASOUND_OFF, MOTION_DEVICE_ADDRESS, 1 );
        issueCommand( PRESCALER, MOTION_DEVICE_ADDRESS, 700 );
        waitAck( PRESCALER, MOTION_DEVICE_ADDRESS, 700 );
        issueCommand( MOVE_FORWARD, MOTION_DEVICE_ADDRESS, 60 );
        waitAck( MOVE_FORWARD, MOTION_DEVICE_ADDRESS, 60 );
        while( TRUE )
        {
          issueCommand( CHECK_ARRIVE, MOTION_DEVICE_ADDRESS, 1 );
          sleep( 100 );
          if ( FLAG_arriveOnDest ) break;
        }
        state_robot++;
        sleep(100);
        break; 
        
      /* Vracanje unazad. */  
      case 13:
        
        issueCommand( ULTRASOUND_ON, MOTION_DEVICE_ADDRESS, 1 );
        waitAck( ULTRASOUND_ON, MOTION_DEVICE_ADDRESS, 1 );
        issueCommand( PRESCALER, MOTION_DEVICE_ADDRESS, 500 );
        waitAck( PRESCALER, MOTION_DEVICE_ADDRESS, 500 );
        issueCommand( MOVE_BACKWARD, MOTION_DEVICE_ADDRESS, 60 );
        waitAck( MOVE_BACKWARD, MOTION_DEVICE_ADDRESS, 60 );
        while( TRUE )
        {
          issueCommand( CHECK_ARRIVE, MOTION_DEVICE_ADDRESS, 1 );
          sleep( 100 );
          if ( FLAG_arriveOnDest ) break;
        }
        state_robot++;
        sleep(100);
        break; 
      
      /* Okretanje ka centru naseg dela terena. */  
      case 14:
        
        if( FLAG_strategyLeft )
        {
          issueCommand( ROTATE_LEFT, MOTION_DEVICE_ADDRESS, 270 );
          waitAck( ROTATE_LEFT, MOTION_DEVICE_ADDRESS, 270 );
          while( TRUE )
          {
            issueCommand( CHECK_ARRIVE, MOTION_DEVICE_ADDRESS, 1 );
            sleep( 100 );
            if ( FLAG_arriveOnDest ) break;
          }
        }
        else
        {
          issueCommand( ROTATE_RIGHT, MOTION_DEVICE_ADDRESS, 270 );
          waitAck( ROTATE_RIGHT, MOTION_DEVICE_ADDRESS, 270 );
          while( TRUE )
          {
            issueCommand( CHECK_ARRIVE, MOTION_DEVICE_ADDRESS, 1 );
            sleep( 100 );
            if ( FLAG_arriveOnDest ) break;
          }
        }
        state_robot++;
        sleep(100);
        break; 
        
      case 15:
      
        issueCommand( MOVE_FORWARD, MOTION_DEVICE_ADDRESS, 40 );
        waitAck( MOVE_FORWARD, MOTION_DEVICE_ADDRESS, 40 );
        while( TRUE )
        {
          issueCommand( CHECK_ARRIVE, MOTION_DEVICE_ADDRESS, 1 );
          sleep( 100 );
          if ( FLAG_arriveOnDest ) break;
        }
        state_robot++;
        sleep(100);
        break; 
        
      /* Podrazumevano stanje u kome se ne radi nista. */  
      default:
        break;
    }
  }
}
/*----------------------------------------------------------------------------*/


/**
  * @brief  Funkcija koja ceka na acknowledge signal od primaoca komande.
  * @param  delay predstavlja broj milisekundi koliko zelimo da cekamo.
  * @retval Nema.
  * @author praetorian ( archmarko92@gmail.com )
  */
void waitAck( CommandNameType command_name, int address, uint16_t data )
{
  while( TRUE )
  {
    sleep( 25 );
    if( FLAG_ackReceived )
    {
      FLAG_ackReceived = FALSE;
      command_ID++;
      break;
    }
    issueCommand( command_name, address, data );
  }
}
/*----------------------------------------------------------------------------*/


/**
  * @brief  Inicijalizacija tajmera koji signalizira kraj igre.
  * @param  delay predstavlja broj milisekundi koliko zelimo da cekamo.
  * @retval Nema.
  * @author praetorian ( archmarko92@gmail.com )
  */
void initTimerServo( void ) 
{
  InitGPIO_Pin( GPIOB, GPIO_Pin_0, GPIO_Mode_AF_PP, GPIO_Speed_50MHz );
  InitTIM_TimeBase(TIM3, 240 - 1, 1000 - 1, TIM_CounterMode_Up, TIM_CKD_DIV1, 0);
  TIM_Cmd(TIM3, ENABLE);
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
  InitNVICChannel(TIM3_IRQn, 0, 0, ENABLE);
  InitTIM_OC(TIM3, TIM_Channel_3, TIM_OutputState_Enable, TIM_OCMode_PWM1, 0, TIM_OCPolarity_High);
}
/*----------------------------------------------------------------------------*/


/**
  * @brief  Funkcija koja unosi zeljeno kasnjenje.
  * @param  delay predstavlja broj milisekundi koliko zelimo da cekamo.
  * @retval Nema.
  * @author praetorian ( archmarko92@gmail.com )
  */
void sleep( int delay )
{
  global_tick = delay;
  while( global_tick != 0 );
}
/*----------------------------------------------------------------------------*/


/**
  * @brief  Inicijalizacija tajmera za odredjivanje vremena cekanja.
  * @param  Nema.
  * @retval Nema.
  * @author praetorian ( archmarko92@gmail.com )
  */
void initTimer90( void )
{
  /* Period je 1ms, korak je 1us. */
  /* Inicijalizacija prostog tajmera 7. */ 
  RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM7, ENABLE );
  TIM7->ARR = 1000;
  TIM7->PSC = 24000;
  TIM7->CNT = 0;
  TIM7->DIER = 1;
  InitNVICChannel( TIM7_IRQn, 1, 1, ENABLE );
  TIM7->CR1 = 1;
};
/*----------------------------------------------------------------------------*/


/**
  * @brief  Odredjivanje koja je strategija igre u pitanju na osnovu prekidaca.
  * @param  Nema.
  * @retval Nema.
  * @author praetorian ( archmarko92@gmail.com )
  */
void checkStrategy( void )
{
  if( GPIO_ReadInputDataBit( GPIOB, GPIO_Pin_12 ) ) FLAG_strategyLeft = TRUE;
  else FLAG_strategyLeft = FALSE;  
}
/*----------------------------------------------------------------------------*/


void DelayUSART(int nCount)
{
  for(; nCount != 0; nCount--);
}
/*----------------------------------------------------------------------------*/


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
/*----------------------------------------------------------------------------*/



//funkcija za ispis na bargraph
void DisplayBarGraphBinary(unsigned char disp){
  //disp je 8-bitni broj
  if (disp&0x01) GPIO_SetBits(GPIOA,GPIO_Pin_6);
  else GPIO_ResetBits(GPIOA,GPIO_Pin_6);  
  disp=disp>>1;
  if (disp&0x01) GPIO_SetBits(GPIOA,GPIO_Pin_3);
  else GPIO_ResetBits(GPIOA,GPIO_Pin_3);  
  disp=disp>>1;
  if (disp&0x01) GPIO_SetBits(GPIOA,GPIO_Pin_2);
  else GPIO_ResetBits(GPIOA,GPIO_Pin_2);     
  disp=disp>>1;  
  if (disp&0x01) GPIO_SetBits(GPIOC,GPIO_Pin_13);
  else GPIO_ResetBits(GPIOC,GPIO_Pin_13);
  disp=disp>>1;
  if (disp&0x01) GPIO_SetBits(GPIOA,GPIO_Pin_5);
  else GPIO_ResetBits(GPIOA,GPIO_Pin_5); 
  disp=disp>>1;
  if (disp&0x01) GPIO_SetBits(GPIOC,GPIO_Pin_4);
  else GPIO_ResetBits(GPIOC,GPIO_Pin_4); 
  disp=disp>>1;
  if (disp&0x01) GPIO_SetBits(GPIOC,GPIO_Pin_5);
  else GPIO_ResetBits(GPIOC,GPIO_Pin_5); 
  disp=disp>>1;
  if (disp&0x01) GPIO_SetBits(GPIOB,GPIO_Pin_2);
  else GPIO_ResetBits(GPIOB,GPIO_Pin_2);     
}

//funkcija za ispis na bargraph
void DisplayBarGraph(unsigned char disp){
  //disp je u opsegu od 0-8
  if (disp>0) GPIO_SetBits(GPIOA,GPIO_Pin_6);
  else GPIO_ResetBits(GPIOA,GPIO_Pin_6);  
  if (disp>1) GPIO_SetBits(GPIOA,GPIO_Pin_3);
  else GPIO_ResetBits(GPIOA,GPIO_Pin_3);
  if (disp>2) GPIO_SetBits(GPIOA,GPIO_Pin_2);
  else GPIO_ResetBits(GPIOA,GPIO_Pin_2);  
  if (disp>3) GPIO_SetBits(GPIOC,GPIO_Pin_13);
  else GPIO_ResetBits(GPIOC,GPIO_Pin_13);   
  if (disp>4) GPIO_SetBits(GPIOA,GPIO_Pin_5);
  else GPIO_ResetBits(GPIOA,GPIO_Pin_5); 
  if (disp>5) GPIO_SetBits(GPIOC,GPIO_Pin_4);
  else GPIO_ResetBits(GPIOC,GPIO_Pin_4); 
  if (disp>6) GPIO_SetBits(GPIOC,GPIO_Pin_5);
  else GPIO_ResetBits(GPIOC,GPIO_Pin_5); 
  if (disp>7) GPIO_SetBits(GPIOB,GPIO_Pin_2);
  else GPIO_ResetBits(GPIOB,GPIO_Pin_2);     
}

//funkcija za ispis na bargraph
void DisplayBarGraphOneHot(unsigned char disp){
  //disp je u opsegu od 1-8
  if (disp==1) GPIO_SetBits(GPIOA,GPIO_Pin_6);
  else GPIO_ResetBits(GPIOA,GPIO_Pin_6);  
  if (disp==2) GPIO_SetBits(GPIOA,GPIO_Pin_3);
  else GPIO_ResetBits(GPIOA,GPIO_Pin_3);  
  if (disp==3) GPIO_SetBits(GPIOA,GPIO_Pin_2);
  else GPIO_ResetBits(GPIOA,GPIO_Pin_2); 
  if (disp==4) GPIO_SetBits(GPIOC,GPIO_Pin_13);
  else GPIO_ResetBits(GPIOC,GPIO_Pin_13);  
  if (disp==5) GPIO_SetBits(GPIOA,GPIO_Pin_5);
  else GPIO_ResetBits(GPIOA,GPIO_Pin_5); 
  if (disp==6) GPIO_SetBits(GPIOC,GPIO_Pin_4);
  else GPIO_ResetBits(GPIOC,GPIO_Pin_4); 
  if (disp==7) GPIO_SetBits(GPIOC,GPIO_Pin_5);
  else GPIO_ResetBits(GPIOC,GPIO_Pin_5); 
  if (disp==8) GPIO_SetBits(GPIOB,GPIO_Pin_2);
  else GPIO_ResetBits(GPIOB,GPIO_Pin_2);     
}
/*----------------------------------------------------------------------------*/


void BateryDisp (void){
    baterija_Acc=baterija_Acc_const*ADC_RegularConvertedValueTab[3];
    servo_5V=servo_5V_const*ADC_RegularConvertedValueTab[2];//ovo se ne prikazuje sada....
    float calculatedf=(baterija_Acc-10)*3.125;
    unsigned char calculated=1+(unsigned char)calculatedf;
    DisplayBarGraph(calculated);
}
/*----------------------------------------------------------------------------*/
