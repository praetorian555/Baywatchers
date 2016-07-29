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
#include "UartDebug.h"
#include "continous_movement.h"
#include "EUROBOT_Init.h"

#include "functions.h"
#include "variables.h"

/** @addtogroup Examples
  * @{
  */
    void InitTimer6();

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

  int brojac = 0;
  int brzina = 0;
  int prethodna_greska1=0;
  int prethodna_greska2=0;
  //float Kp=0.9,Ki=0.06,Kd=0.1,Int=0;
  float Kp=30,Ki=2,Kd=0, Int1=0, Int2=0;//10,1,0.5
 
  
  float Kp_poz=0.5,Ki_poz=0.9,Kd_poz=0.05,Int_poz=0;
  volatile unsigned int Pos1=32767, Pos2=32767;
  int prethodna_greska_poz=0;
  int speed_ref_pos[10]={130, 250, 350, 500, 750, 1000, 1250, 1500, 1750, 2000};
  
  volatile int putanja_counter = 0;          ///////nesto nece da se definise gore????
  long putanja[100][2];
  int flag_a=0;
  int flag_c=0;
  int flag_first_step_continous=1;
  extern bool stigao;
  
  bool running=FALSE;
    
extern bool FLAG_sensorEnable;
/* Private function prototypes -----------------------------------------------*/
void InitUltrasoundHCSR04( void );

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
     
    //inicijalizacija
    GPIO_InitTypeDef  GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_OCInitTypeDef TIM_OCInitStruct;
    
    /* Pokretanje SysTick prekida u kome se prsi PID kontrola. */
    SysTick_Config(SystemCoreClock / 100);
    
     /* Enable the Clock for used peripherals*/
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE );
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM3, ENABLE );
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    
    /* Configure Encoder pins as input */
    // ENC1 A - PA0, ENC1 A - PA1, ENC1 B - PA2, ENC1 B - PA3
    // ENC2 A - PA4, ENC2 A - PA5, ENC2 B - PA6, ENC2 B - PA7    
    /*GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init( GPIOA, &GPIO_InitStructure );
    GPIO_StructInit(&GPIO_InitStructure);*/ 
    
    //stara ploca
    /*
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init( GPIOB, &GPIO_InitStructure );
    GPIO_StructInit(&GPIO_InitStructure); */
    
    //stara ploca koristi prekide za enkodere
    // Connect Encoder EXTI Line to Encoder GPIO Pin
    /*GPIO_EXTILineConfig( GPIO_PortSourceGPIOA, GPIO_PinSource0 );  // povezuje se pin A0 na prekidnu liniju EXTI0
    GPIO_EXTILineConfig( GPIO_PortSourceGPIOA, GPIO_PinSource1 );  // povezuje se pin A1 na prekidnu liniju EXTI1
    GPIO_EXTILineConfig( GPIO_PortSourceGPIOA, GPIO_PinSource2 );  // povezuje se pin A2 na prekidnu liniju EXTI2
    GPIO_EXTILineConfig( GPIO_PortSourceGPIOA, GPIO_PinSource3 );  // povezuje se pin A3 na prekidnu liniju EXTI3
    GPIO_EXTILineConfig( GPIO_PortSourceGPIOA, GPIO_PinSource4 );  // povezuje se pin A4 na prekidnu liniju EXTI4
    GPIO_EXTILineConfig( GPIO_PortSourceGPIOA, GPIO_PinSource5 );  // povezuje se pin A5 na prekidnu liniju EXTI5
    GPIO_EXTILineConfig( GPIO_PortSourceGPIOA, GPIO_PinSource6 );  // povezuje se pin A6 na prekidnu liniju EXTI6
    GPIO_EXTILineConfig( GPIO_PortSourceGPIOA, GPIO_PinSource7 );  // povezuje se pin A7 na prekidnu liniju EXTI7
    */   

    //stara ploca koristi i komplementarne izlaze za WM drajvere 
    /*
    // Setup PB13 PB14  as PWM generator for dc motor driver
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;            // konfiguracija za alternativnu funkciju - Push Pull
    GPIO_Init( GPIOB, &GPIO_InitStructure );
    //GPIO_PinRemapConfig( GPIO_FullRemap_TIM1, ENABLE );        // Map TIM3_CH3 to GPIOC.Pin8, TIM3_CH4 to GPIOC.Pin9
    */
    
    //kod nove ploce isti izlazi su za PWM
    //Setup PA8 PA9  as PWM generator for dc motor driver
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;            // konfiguracija za alternativnu funkciju - Push Pull
    GPIO_Init( GPIOA, &GPIO_InitStructure );
    //GPIO_PinRemapConfig( GPIO_FullRemap_TIM1, ENABLE );        // Map TIM3_CH3 to GPIOC.Pin8, TIM3_CH4 to GPIOC.Pin9
    
    
    //Tajmer TI3 radi u modu generisanja PWM signala
    //Inicijalizacija vremenske baze tajmera TIM3
    // Let PWM frequency equal 100Hz.
    // Let period equal 1000. Therefore, timer runs from zero to 1000. Gives 0.1Hz resolution.
    // Solving for prescaler gives 240.
    TIM_TimeBaseStructInit( &TIM_TimeBaseInitStruct );
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV4;
    TIM_TimeBaseInitStruct.TIM_Period = 3000 - 1;   // 0..999
    TIM_TimeBaseInitStruct.TIM_Prescaler = 240 - 1; // Div 240
    TIM_TimeBaseInit( TIM3, &TIM_TimeBaseInitStruct );
 

    //Tajmer TIM1 radi u modu generisanja PWM signala
    //Inicijalizacija vremenske baze tajmera TIM1
    // Let PWM frequency equal 100Hz.
    // Let period equal 1000. Therefore, timer runs from zero to 1000. Gives 0.1Hz resolution.
    // Solving for prescaler gives 240.
    TIM_TimeBaseStructInit( &TIM_TimeBaseInitStruct );
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV4;
    TIM_TimeBaseInitStruct.TIM_Period = 1000 - 1;   // 0..999
    TIM_TimeBaseInitStruct.TIM_Prescaler = 12 - 1; // Div 12
    TIM_TimeBaseInit( TIM1, &TIM_TimeBaseInitStruct );
    
    /* Channel 1 output configuration */
    TIM_OCStructInit(&TIM_OCInitStruct);
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_Pulse = 0;
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OC1Init(TIM1, &TIM_OCInitStruct);	
    /*Outputs Enable */
    TIM_OC1NPolarityConfig(TIM1, TIM_OCNPolarity_High);
    TIM_OC1PolarityConfig(TIM1, TIM_OCPolarity_High);        
    
    /* Channel 2 output configuration */
    TIM_OCStructInit(&TIM_OCInitStruct);
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_Pulse = 0;
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OC2Init(TIM1, &TIM_OCInitStruct);	
    /*Outputs Config */
    TIM_OC2NPolarityConfig(TIM1, TIM_OCNPolarity_High);
    TIM_OC2PolarityConfig(TIM1, TIM_OCPolarity_High);        
    
    /*Outputs Enable */
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_CCxNCmd(TIM1,TIM_Channel_1, TIM_CCxN_Enable);
    TIM_CCxNCmd(TIM1,TIM_Channel_2, TIM_CCxN_Enable);
    
    /*counter enable */
    TIM_Cmd(TIM1, ENABLE);
    
    //starat ploca
    /*
    GPIO_ResetBits(GPIOB, GPIO_Pin_12);//DISABLE M1
    GPIO_ResetBits(GPIOB, GPIO_Pin_15);//DISABLE M2
    TIM1->CCR1=600;
    TIM1->CCR2=400; 
    GPIO_SetBits(GPIOB, GPIO_Pin_12);//ENABLE M1   
    GPIO_SetBits(GPIOB, GPIO_Pin_15);//ENABLE M2
    */
    
    //nova ploca
    // PA4 - ina1, PA10 - inb1
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init( GPIOA, &GPIO_InitStructure );
    GPIO_StructInit(&GPIO_InitStructure);    
    //M1 CW: PA4=1, PA10=0
    //GPIO_SetBits(GPIOA,GPIO_Pin_4);
    //GPIO_ResetBits(GPIOA,GPIO_Pin_10);
    //M1 CCW: PA4=0, PA10=1
    //GPIO_ResetBits(GPIOA,GPIO_Pin_4);
    //GPIO_SetBits(GPIOA,GPIO_Pin_10);

      
    
    // PC9 - ina2, PC8 - in2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init( GPIOC, &GPIO_InitStructure );
    GPIO_StructInit(&GPIO_InitStructure);
    //M2 CW: PC9=1, PC8=0
    //GPIO_SetBits(GPIOC,GPIO_Pin_9);
    //GPIO_ResetBits(GPIOC,GPIO_Pin_8);
    //M2 CCW: PC9=0, PC8=1
    //GPIO_ResetBits(GPIOC,GPIO_Pin_9);
    //GPIO_SetBits(GPIOC,GPIO_Pin_8); 
    
     
    
    TIM1->CCR1=500;
    TIM1->CCR2=500; 
    
    //DOZVOLA GENERISANJA PREKIDA
    //Dozvola generisanja prekida od strane TIM2 periferije
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    

    //stara ploca - prekidi za enkodere
    /*
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;      
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    EXTI_InitStructure.EXTI_Line = EXTI_Line1;      
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    EXTI_InitStructure.EXTI_Line = EXTI_Line2;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    EXTI_InitStructure.EXTI_Line = EXTI_Line3;      
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    EXTI_InitStructure.EXTI_Line = EXTI_Line4;      
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    EXTI_InitStructure.EXTI_Line = EXTI_Line5;      
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    EXTI_InitStructure.EXTI_Line = EXTI_Line6;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    EXTI_InitStructure.EXTI_Line = EXTI_Line7;      
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
   
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure); 
    
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn; 
    NVIC_Init(&NVIC_InitStructure); 

    NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn; 
    NVIC_Init(&NVIC_InitStructure); 

    NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn; 
    NVIC_Init(&NVIC_InitStructure); 

    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn; 
    NVIC_Init(&NVIC_InitStructure); 

    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn; 
    NVIC_Init(&NVIC_InitStructure); 
    */
    
    //nova ploca
      
    GPIO_EXTILineConfig( GPIO_PortSourceGPIOD, GPIO_PinSource2 );  // 
    GPIO_EXTILineConfig( GPIO_PortSourceGPIOB, GPIO_PinSource5 );  //        
    GPIO_EXTILineConfig( GPIO_PortSourceGPIOB, GPIO_PinSource14 );  // 
    GPIO_EXTILineConfig( GPIO_PortSourceGPIOB, GPIO_PinSource15 );  //

    EXTI_InitStructure.EXTI_Line = EXTI_Line2;      
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    EXTI_InitStructure.EXTI_Line = EXTI_Line5;      
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);    
    
    EXTI_InitStructure.EXTI_Line = EXTI_Line14;      
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    EXTI_InitStructure.EXTI_Line = EXTI_Line15;      
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);     
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn; 
    NVIC_Init(&NVIC_InitStructure);    
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn; 
    NVIC_Init(&NVIC_InitStructure);
   
    //PIN PA5 na DIGIO konektoru za test
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init( GPIOA, &GPIO_InitStructure );
    GPIO_StructInit(&GPIO_InitStructure);     

    UsartInit();    
    PositionControllerInit();
    InitUltrasoundHCSR04();
    
    /*dakle plan jesledeci:
    glavni kontroler salje komandu start counting cim se pokrene tj. u nultom stanju
    ovaj kontorler kretanja po rijemu te komande startuje timer 6.
    inicijalizaciju tajmera uraditi u tom prekidu kada isparsira komandu
    timer 6 po isteku vremena od 90s obara running flag na FALSE i to je to...
    //proveriti da nije negde ostalo disable_interrupt osim na samom kraju koda...
    
  */
  while (!running)//ovde ceka dadobije running od glavnog kontrolera
  {
  } 
  
  //ovde inicijalizauje TIMER6
  InitTimer6();
  
  while (running)
  {
    //odje vozimo
  }
  //odje gasimo sve jer je timer 6 rekid opet oborio running na FALSE
    
    __disable_interrupt();    
    GPIO_ResetBits(GPIOA,GPIO_Pin_4);
    GPIO_ResetBits(GPIOA,GPIO_Pin_10);    
    GPIO_ResetBits(GPIOC,GPIO_Pin_9);
    GPIO_ResetBits(GPIOC,GPIO_Pin_8);
  
  while(1);
}

void InitTimer6(void){
  RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM6, ENABLE );
  TIM6->ARR = 1000;
  TIM6->PSC = 24000;
  TIM6->CNT = 0;
  TIM6->DIER = 1;
  InitNVICChannel( TIM6_DAC_IRQn, 1, 1, ENABLE );
  TIM6->CR1 = 1;
    
};

/**
  * @brief  Funkcja za inicijalizaciju cetiri ultrazvucna senzora HC-SR04.
  *         Trigger1 - TIM3CH3 - PB0.
  *         Trigger2 - TIM3CH4 - PB1.
  *         Echo1 -TIM4CH1 - PB6.
  *         Echo2 -TIM4CH2 - PB7.  
  *         Echo3 -TIM4CH3 - PB8.
  *         Echo4 -TIM4CH4 - PB9.
  * @param  Nema ulaznih parametara.
  * @retval Nema povratnih vrednosti. 
  * @author Milica Stojiljkovic
  *
  */
void InitUltrasoundHCSR04( void )
{ 
  /* Inicijalizacija pinova na kojima se generise trigger. */
  InitGPIO_Pin( GPIOB, GPIO_Pin_0, GPIO_Mode_AF_PP, GPIO_Speed_50MHz ); // channel 3 tim3 - B0
  InitGPIO_Pin( GPIOB, GPIO_Pin_1, GPIO_Mode_AF_PP, GPIO_Speed_50MHz ); // channel 4 tim3 - B1
  
  
  /* Perioda trigger signala je 75 ms sa korakom 5us. Jedan je edge-aligned a drugi centre-aligned
     da ne bi smetali jedan drugom. Maksimalno trajanja signala echo je 2.5 ms tako da je trenutna
     razlika od ~37ms izmedju signala sasvim dovoljna. */
  InitTIM_TimeBase( TIM3, 120 - 1, 15000 - 1, TIM_CounterMode_Up, TIM_CKD_DIV1, 0x00 );
  InitTIM_OC( TIM3, TIM_Channel_3, TIM_OutputState_Enable, TIM_OCMode_PWM1, 2, TIM_OCPolarity_High );  // edge-aligned
  InitTIM_OC( TIM3, TIM_Channel_4, TIM_OutputState_Enable, TIM_OCMode_PWM2, 2, TIM_OCPolarity_High );  // centre-aligned
  
  
  /* Inicijalizacija pinvoa za prijem echo-a. */
  InitGPIO_Pin( GPIOB, GPIO_Pin_6, GPIO_Mode_IN_FLOATING, GPIO_Speed_50MHz );
  InitGPIO_Pin( GPIOB, GPIO_Pin_7, GPIO_Mode_IN_FLOATING, GPIO_Speed_50MHz );
  InitGPIO_Pin( GPIOB, GPIO_Pin_8, GPIO_Mode_IN_FLOATING, GPIO_Speed_50MHz );
  InitGPIO_Pin( GPIOB, GPIO_Pin_9, GPIO_Mode_IN_FLOATING, GPIO_Speed_50MHz );
  
  
  /* Perioda tajmera koji prima echo signal je 50 ms sa korakom 1 us. Svaki kanal meri signal
     sa jednog ultrazvucnog senzora. */ 
  InitTIM_TimeBase( TIM4, 24 - 1, 50000 - 1, TIM_CounterMode_Up, TIM_CKD_DIV1, 0x00 );
  InitTIM_IC( TIM4, TIM_Channel_1, TIM_ICSelection_DirectTI, TIM_ICPSC_DIV1, 0x00, TIM_ICPolarity_Rising );
  InitTIM_IC( TIM4, TIM_Channel_2, TIM_ICSelection_DirectTI, TIM_ICPSC_DIV1, 0x00, TIM_ICPolarity_Rising );
  InitTIM_IC( TIM4, TIM_Channel_3, TIM_ICSelection_DirectTI, TIM_ICPSC_DIV1, 0x00, TIM_ICPolarity_Rising );
  InitTIM_IC( TIM4, TIM_Channel_4, TIM_ICSelection_DirectTI, TIM_ICPSC_DIV1, 0x00, TIM_ICPolarity_Rising );
  
  
  /* Trigger za interrupt nam dolazi od prvog ulaza u tajmer. */
  TIM_SelectInputTrigger( TIM4, TIM_TS_TI1FP1 ); // TIM_TS_TI1FP1 znaci Filtered Timer Input 1
  
 
  /* Osposabljavanje prekida za belezenje trenutne vrednosti u brojacu tajmera radi racunanja udaljenosti objekta. */
  TIM_ITConfig( TIM4, TIM_IT_CC1, ENABLE );
  TIM_ITConfig( TIM4, TIM_IT_CC2, ENABLE );
  TIM_ITConfig( TIM4, TIM_IT_CC3, ENABLE );
  TIM_ITConfig( TIM4, TIM_IT_CC4, ENABLE );
  
  
  /* Inicijalizacija NVIC kanala. */
  InitNVICChannel( TIM4_IRQn, 0, 0, ENABLE );
  
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/

//to be done
/*
potrebne su komande za pdoesavanje nekih konstanti - za pocetak uglovna konstanta
brzina je implementirana
reset na neku nultu poziciju i enkodera svega ostalog...
tj. ne na nultu nego na zadatu...
mozda trebe emergency stop. to se pozitze kada se trenutna pocija izjednaci sa zadatom
mozda bi trebalo prosiriti i status robota...


*/
