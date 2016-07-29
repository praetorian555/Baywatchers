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
#include "UartDebug.h"
#include "print.h"
#include "stm32f10x_it.h"
#include "STM32vldiscovery.h"
#include "continous_movement.h"

#include "variables.h"


#include <math.h>
#define PI 3.14159265
#define CTM 1
#define MTC 1
#define CTU 0.03076923076//0.03783538289
#define UTC 32.5 //31.55
#define STC 281.6976
#define forwardConstant 1
#define MAX_TRANSX_LEN 200
#define ADDR 0x0A
#define SCALE 1
#define MEASUREMENT_NUM 5          // Broj merenja, nakon cega se racuna srednja vrednost.
#define ECHO_TIMER_PERIOD 50000    // Period tajmera koji meri echo.
#define CONST_MICRO_SEC_TO_MM 5.8  // Konstanta za pretvaranje us u mm.
#define MAX_DISTANCE_MM 300        // Rastojanje koje se detektuje kada treba da se zaustavimo. 
#define STOP_DISTANCE 0
#define PROXIMITY_CONSTANT 10   

//#define angularConstant 0.00798226//0.01538461538//0.01891769144// ovo se dobija kao 180/broj impulsa za rotaciju

// DODATO, STATUS ROBOTA
int robot_status = 0;

typedef struct{
  long x;
  long y;
  int theta;
}absPosition;
/** @addtogroup Examples
* @{
*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
char state = 0;
//extern unsigned long ENC1, ENC2, ENC1_old, ENC2_old;
int err = 0;
static absPosition apsolutnaPozicija={.x=0, .y=0, .theta=0};
extern int brojac, brzina,prethodna_greska1, prethodna_greska2;

extern float Kp,Ki,Kd,Int1, Int2;
int x;  //temp promenljiva za pomeraj
extern float Kp_poz,Ki_poz,Kd_poz,Int_poz;
extern volatile unsigned int Pos1, Pos2;        //Pozicije motora, za sad
extern int prethodna_greska_poz;
extern int speed_ref_pos[10];
int byte_counter = 0;
int message_size = -1;
int znak = 1;
char komanda;
int enkoder1=0;
int enkoder2=0;
bool pocetak_prijema = 0;
bool stigao, stigao1 = 0;
int cnt_90 = 0;

static int neki_brojac=0;
static int sending_array[260];
static int sending_length=0;
static int sending_iterator=0;

bool ulm_received=0; //unknown length message received
int ulm_length=0;
///stefan
bool advanced_segment_stigao=FALSE;
int bajt=0;
int cmd_counter=1;
extern long putanja[100][2];
extern int flag_a;
extern int putanja_counter;
////// ende stefan



extern int flag_c;
Coordinates polyline_path[50];
int polyline_length=0;
extern int flag_first_step_continous;
int MAX_DIFF_SPEED=24;
int MAX_SPEED=75;
int MAX_SPEED_TOTAL=99;

int ruka_pozicija=0;
int ruka_zatvorena=50,ruka_otvorena=100;
bool otvori=0,zatvori=0;
//static unsigned char chksum=0,schksum=0;
static int brzina_prim=0;

//flags

static bool flag_SR=FALSE, flag_AR=FALSE, flag_CS=FALSE, flag_following_active=FALSE;

char chksum_reception=0,chksum_transmission=0;
static int byte_count=MAX_TRANSX_LEN;
int c_counter=0;
int diff_brzina=0;
  static float Propc, Difc,err_c=0;
float Intc=0;
    Vector temp;
        Coordinates temp_point;
        
short int Speed1=0, Speed2=0;

int MBEDwatchdog=0;

extern volatile unsigned int Pos1;
extern volatile unsigned int Pos2;
int zapamcena_pozicija_X = 32767;
int zapamcena_pozicija_Y = 32767;

/*TIM4CH1 - senzor nazad, levo. */
unsigned long index_ch1 = 0;         		      // Predstavlja index poslednjeg merenja u nizu merenja.
bool bad_measurement_end_ch1 = FALSE;                 // Jedinica signalizira da nema vise pocetnih, losih merenja.
float distance_bl_ch1 = 0;       		      // Predstavlja izracunatu udaljenost od objekta.
uint16_t measured_distance_bl_ch1[MEASUREMENT_NUM];   // Niz u koji se smestaju izmerene udaljenosti za MEASUREMENT_NUM impulsa.
uint16_t measured_time_bl_rising_ch1 = 0;             // Trenutak kada se desila uzlazna ivica.
uint16_t measured_time_bl_falling_ch1 = 0;            // Trenutak kada se desila silazna ivica.
uint16_t measured_time_bl_ch1 = 0;                    // Vreme trajanje echo signala.
uint8_t state_bl_ch1 = 0;    			      // 0 ako se hvata uzlazna ivica, 1 ako se hvata silazna ivica.
  
/*TIM4CH2 - senzor nazad, desno. */
unsigned long index_ch2 = 0;         		      // Predstavlja index poslednjeg merenja u nizu merenja.
bool bad_measurement_end_ch2 = FALSE;                 // Jedinica signalizira da nema vise pocetnih, losih merenja.
float distance_br_ch2 = 0;       		      // Predstavlja izracunatu udaljenost od objekta.
uint16_t measured_distance_br_ch2[MEASUREMENT_NUM];   // Niz u koji se smestaju izmerene udaljenosti za MEASUREMENT_NUM impulsa.
uint16_t measured_time_br_rising_ch2 = 0;             // Trenutak kada se desila uzlazna ivica.
uint16_t measured_time_br_falling_ch2 = 0;            // Trenutak kada se desila silazna ivica.
uint16_t measured_time_br_ch2 = 0;                    // Vreme trajanje echo signala.
uint8_t state_br_ch2 = 0;    			      // 0 ako se hvata uzlazna ivica, 1 ako se hvata silazna ivica.
  
/*TIM4CH3 - senzor napred desno. */
unsigned long index_ch3 = 0;         		      // Predstavlja index poslednjeg merenja u nizu merenja.
bool bad_measurement_end_ch3 = FALSE;                 // Jedinica signalizira da nema vise pocetnih, losih merenja.
float distance_fr_ch3 = 0;       		      // Predstavlja izracunatu udaljenost od objekta.
uint16_t measured_distance_fr_ch3[MEASUREMENT_NUM];   // Niz u koji se smestaju izmerene udaljenosti za MEASUREMENT_NUM impulsa.
uint16_t measured_time_fr_rising_ch3 = 0;             // Trenutak kada se desila uzlazna ivica.
uint16_t measured_time_fr_falling_ch3 = 0;            // Trenutak kada se desila silazna ivica.
uint16_t measured_time_fr_ch3 = 0;                    // Vreme trajanje echo signala.
uint8_t state_fr_ch3 = 0;    			      // 0 ako se hvata uzlazna ivica, 1 ako se hvata silazna ivica.
  
/* TIM4CH4 - senzor napred levo. */
unsigned long index_ch4 = 0;         		      // Predstavlja index poslednjeg merenja u nizu merenja.
bool bad_measurement_end_ch4 = FALSE;                 // Jedinica signalizira da nema vise pocetnih, losih merenja.
float distance_fl_ch4 = 0;       		      // Predstavlja izracunatu udaljenost od objekta.
uint16_t measured_distance_fl_ch4[MEASUREMENT_NUM];   // Niz u koji se smestaju izmerene udaljenosti za MEASUREMENT_NUM impulsa.
uint16_t measured_time_fl_rising_ch4 = 0;             // Trenutak kada se desila uzlazna ivica.
uint16_t measured_time_fl_falling_ch4 = 0;            // Trenutak kada se desila silazna ivica.
uint16_t measured_time_fl_ch4 = 0;                    // Vreme trajanje echo signala.
uint8_t state_fl_ch4 = 0;                             // 0 ako se hvata uzlazna ivica, 1 ako se hvata silazna ivica.

/* Flag-ovi senzora. */
bool FLAG_sensorEnable = FALSE;                      // Flag koji postavlja glavna ploca i definise da li gledamo senzore ili ne.
bool FLAG_sensorFrontEnable = FALSE;                 // Flag koji se postavlja ili brise na osnovu zadate instrukcije i definise da li gledamo prednje senzore.
bool FLAG_sensorBackEnable = FALSE;                  // Flag koji se postavlja ili brise na osnovu zadate instrukcije i definise da li gledamo zadnje senzore.
bool FLAG_obstacleDetected = FALSE;                 // Flag koji signalizira da je prepreka detektovana. Postavlja se samo prvi put kad je odredjena prepreka detektovana.

void InitTimer6(void);


/* Private function prototypes -----------------------------------------------*/
/* Proverava da li je robot stigao na cilj. */
_Bool checkIfAtDest( void );
/* Provera flag-ova senzora i zakljucivanje da li ima prepreke ili ne. */
_Bool checkForObstacle( void );
/* Staje sa lepim usporavanjem ako detekuje prepreku. */
_Bool stopIfObstacle( void );
/* Private functions ---------------------------------------------------------*/

extern  bool running;

/********************************************************************

Funkcija koja racuna apsolutnu poziciju, na osnovu infinitezimalno malih pomeraja motora

***********************************************************************/
/*
absPosition calculatePosition(int Left, int Right){
  static long X=0,Y=0;
  static float Theta=0;
  static unsigned long formerLeft=32767, formerRight=32767;
  absPosition temp;
  int D, dRight, dLeft;
  float dTheta;
  dLeft=(int)(Left-formerLeft);
  dRight=(int)(Right-formerRight);
  
  D=(dLeft+dRight)/2;
  dTheta=(float)(angularConstant*(dLeft-dRight));
  Theta+=dTheta;
  X+=(int)((float)(D*sin(Theta*PI/180)));
  Y+=(int)((float)(D*cos(Theta*PI/180)));
  
  temp.x=X;
  temp.y=Y;
  temp.theta=(((int)(Theta)) >= 0 ? 0 : 360) + ((int)(Theta)) % 360;
  formerLeft=Left;
  formerRight=Right;
  return temp;
}*/

absPosition calculatePosition(int Left, int Right){
  //static long abs_X=0,abs_Y=0;
  //static float abs_Theta=0;
  static unsigned long formerLeft=32767, formerRight=32767;
  absPosition temp;
  int D, dRight, dLeft;
  float dTheta;
  dLeft=(int)(Left-formerLeft);
  dRight=(int)(Right-formerRight);
  
  D=(dLeft+dRight)/2;
  dTheta=(float)(angularConstant*(dLeft-dRight));
  abs_Theta+=dTheta;
  abs_X+=(int)((float)(D*sin(abs_Theta*PI/180)));
  abs_Y+=(int)((float)(D*cos(abs_Theta*PI/180)));
  
  temp.x=abs_X;
  temp.y=abs_Y;
  temp.theta=(((int)(abs_Theta)) >= 0 ? 0 : 360) + ((int)(abs_Theta)) % 360;
  formerLeft=Left;
  formerRight=Right;
  return temp;
}

/******************************************************************************/
/*                 STM32F10x USART Interrupt Handler                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

int address;
unsigned int chksum=0,schksum=0;
unsigned char temp_ID;
unsigned char received_byte;
unsigned int received_array[MAX_TRANSX_LEN];

void SendAck(void){
        sending_length=4;
        sending_array[0]=0xFF;
        sending_array[1]=ADDR|0x40;
        sending_array[2]=sending_length-3;        
        schksum=0;
        for(int i=1;i<sending_length-1;i++){
          schksum+=sending_array[i]&0xFF;
        }
        sending_array[3]=schksum&0x7F;//0x4B
        GPIO_SetBits(GPIOC,GPIO_Pin_12); //enejbluje se RS485 predaja
        USART_SendData(USART3,  sending_array[0]);//salje prvi podatak
        sending_iterator=1;
};

//Funkcija za slanje pozicije kada je primljena odgovarajuca komanda
void SendPosition( void )
{ 
  int x_inp, y_inp, xy_inp;
        sending_length=25;
        sending_array[0] = 0xFF;
        sending_array[1] = ADDR|0x40;
        sending_array[2] = sending_length-3;
        sending_array[3] = (apsolutnaPozicija.x & 0x0000000F);
        sending_array[4] = (apsolutnaPozicija.x & 0x000000F0)>>4;
        sending_array[5] = (apsolutnaPozicija.x & 0x00000F00)>>8;
        sending_array[6] = (apsolutnaPozicija.x & 0x0000F000)>>12;
        sending_array[7] = (apsolutnaPozicija.x & 0x000F0000)>>16;
        sending_array[8] = (apsolutnaPozicija.x & 0x00F00000)>>20;
        sending_array[9] = (apsolutnaPozicija.x & 0x0F000000)>>24;
        sending_array[10]=(apsolutnaPozicija.x & 0xF0000000)>>28;
        
        sending_array[11]=(apsolutnaPozicija.y & 0x0000000F);
        sending_array[12]=(apsolutnaPozicija.y & 0x000000F0)>>4;
        sending_array[13]=(apsolutnaPozicija.y & 0x00000F00)>>8;
        sending_array[14]=(apsolutnaPozicija.y & 0x0000F000)>>12;
        sending_array[15]=(apsolutnaPozicija.y & 0x000F0000)>>16;
        sending_array[16]=(apsolutnaPozicija.y & 0x00F00000)>>20;
        sending_array[17]=(apsolutnaPozicija.y & 0x0F000000)>>24;
        sending_array[18]=(apsolutnaPozicija.y & 0xF0000000)>>28;
        
        sending_array[19]=(apsolutnaPozicija.theta & 0x000F);
        sending_array[20]=(apsolutnaPozicija.theta & 0x00F0)>>4;
        sending_array[21]=(apsolutnaPozicija.theta & 0x0F00)>>8;
        sending_array[22]=(apsolutnaPozicija.theta & 0xF000)>>12;
        
        // Ready bit, vraca da li je robot stigao na zeljenu destinaciju
        x_inp=0;
        y_inp=0;
        xy_inp=0;
        if ((trenutna_pozicija_X<zapamcena_pozicija_X+INP_TOLERANCE)&&(trenutna_pozicija_X>zapamcena_pozicija_X-INP_TOLERANCE)) x_inp=1;
        if ((trenutna_pozicija_Y<zapamcena_pozicija_Y+INP_TOLERANCE)&&(trenutna_pozicija_Y>zapamcena_pozicija_Y-INP_TOLERANCE)) y_inp=1;
        if ((x_inp==1)&&(y_inp==1)) {
          xy_inp=1;
        }
        
        sending_array[23]=xy_inp?0x71:0x70;
        
        schksum=0;
        for(int i=1;i<sending_length-1;i++){
          schksum+=sending_array[i]&0xFF;
        }
        sending_array[sending_length-1]=schksum&0x7F;
        
        GPIO_SetBits(GPIOC,GPIO_Pin_12); //enejbluje se RS485 predaja
        USART_SendData(USART3,  sending_array[0]);//salje prvi podatak
        sending_iterator=1;
};

void SendDestFlag( void )
{
  bool arrive_X = FALSE;
  bool arrive_Y = FALSE;
  bool arriveEnable = FALSE;
  int x_inp, y_inp, xy_inp;
  x_inp=0;
  y_inp=0;
  xy_inp=0;
        if ((trenutna_pozicija_X<zapamcena_pozicija_X+INP_TOLERANCE)&&(trenutna_pozicija_X>zapamcena_pozicija_X-INP_TOLERANCE)) x_inp=1;
        if ((trenutna_pozicija_Y<zapamcena_pozicija_Y+INP_TOLERANCE)&&(trenutna_pozicija_Y>zapamcena_pozicija_Y-INP_TOLERANCE)) y_inp=1;
        if ((x_inp==1)&&(y_inp==1)) {
          xy_inp=1;
        }
  if ( xy_inp == 1 ) arriveEnable = TRUE;
  else arriveEnable = FALSE;
  
  /* Zapocinjanje slanja poruke. */
  sending_length = 6;
  sending_array[0]=0xFF;
  sending_array[1]=ADDR|0x40;
  sending_array[2]=sending_length-3;
  sending_array[3] = arriveEnable & 0x0F;
  sending_array[4] = (arriveEnable >> 4) & 0x0F;
  schksum=0;
  for(int i=1;i<sending_length-1;i++) schksum+=sending_array[i]&0xFF;
  sending_array[5]=schksum&0x7F;//0x4B
  GPIO_SetBits(GPIOC,GPIO_Pin_12); //enejbluje se RS485 predaja
  USART_SendData(USART3,  sending_array[0]);//salje prvi podatak
  sending_iterator=1;
}

/**
  *
  */
void Response(void){
  
  unsigned char temp_ID;
  
  /* Dekodovanje poruke. */
  if (address == ADDR)
  {
    switch (received_array[0])
    {
      case 0xE0: komanda = 'a'; break;  //Podesavanje uglovne konstante
      case 0x01: komanda = 'b'; break;  //Setovanje pozicije robota(x, y, ugao)
      case 0xE2: komanda = 'c'; break;  //Reset pozicije na nulu
      case 0x0A: komanda = 'd'; break;  //Emergency stop
      case 0x04: komanda = 'l'; znak = 1; break;  //Kretanje napred
      case 0x05: komanda = 'l'; znak = -1; break; //Kretanje nazad
      case 0x06: komanda = 'r'; znak = 1; break;  //Rotacija desno
      case 0x07: komanda = 'r'; znak = -1; break; //Rotacija levo
      case 0x11: komanda = 's'; break;  // Paljenje senzora                              PITATI JOVICICA!!!
      case 0x12: komanda = 'q'; break;  // Gasenje senzora
      case 0xF7: komanda = 'v'; break;  // Inspektorska, vraca status poruku koja sadrzi poziciju, rotaciju, status robota i ready flag(oznacava da li je robot stigao u zadatu poziciju)
      case 0xFC: komanda='p'; break;    // Inspektorska, vraca da li je robot stigao u zadatu poziciju.
      case 0xFB: komanda = 'z'; break;  //Podesavanje preskalera za brzinu
      case 0xFA: komanda = 'n'; break;  //zaustavi sve....
      default: komanda = 'v'; break;
    };
    
    /* Izvrsavanje instrukcije. */
    
    /* Kretanje napred ili nazad. */
    if (komanda == 'l') {        
        x = znak * (received_array[1] | received_array[2]<<4 | received_array[3]<<8 | received_array[4]<<12);        
        temp_ID=(received_array[5] | received_array[6]<<4);         
        if (temp_ID != command_ID){
          Pos1 = zadata_pozicija_X + x;
          Pos2 = zadata_pozicija_Y + x;
          zadata_pozicija_X = Pos1;
          zadata_pozicija_Y = Pos2;
          zapamcena_pozicija_X = zadata_pozicija_X;
          zapamcena_pozicija_Y = zadata_pozicija_Y;
          if ( znak == 1 )
          {
            FLAG_sensorFrontEnable = TRUE;
            FLAG_sensorBackEnable = FALSE;
          }
          else if ( znak == -1 )
          {
            FLAG_sensorFrontEnable = FALSE;
            FLAG_sensorBackEnable = TRUE;
          }
          command_ID=temp_ID;
        };        
        SendAck();
    }
    /* Rotacija levo ili desno. */
    else if (komanda == 'r') {        
        x = (received_array[1] | received_array[2]<<4 | received_array[3]<<8 | received_array[4]<<12);
        x = x * znak;        
        temp_ID=(received_array[5] | received_array[6]<<4);         
        if (temp_ID!=command_ID){
          Pos1 = zadata_pozicija_X + x;
          Pos2 = zadata_pozicija_Y - x;
          zadata_pozicija_X=Pos1;
          zadata_pozicija_Y=Pos2;
          zapamcena_pozicija_X = zadata_pozicija_X;
          zapamcena_pozicija_Y = zadata_pozicija_Y;
          command_ID=temp_ID;
          
          FLAG_sensorFrontEnable = FALSE;
          FLAG_sensorBackEnable = FALSE;
        };       
        SendAck();
     }
    //Podesavanje preskalera za brzinu
     else if (komanda == 'z') {
        x = (received_array[1] | received_array[2]<<4 | received_array[3]<<8 | received_array[4]<<12);  
        TIM2->PSC=x;
        TIM7->PSC=x;
        SendAck();        
     }
     //Inspektorska, vraca status poruku koja sadrzi poziciju, rotaciju i ready flag(oznacava da li je robot stigao u zadatu poziciju)
     else if (komanda == 'v') {
        SendPosition();
     }
    //Podesavanje uglovne konstante
     else if (komanda == 'a') {
        x = (received_array[1] | received_array[2]<<4 | received_array[3]<<8 | received_array[4]<<12);  
        angularConstant=180/((float)x);
        SendAck();        
     }
     //Setovanje pozicije robota(x, y, ugao)
     else if (komanda == 'b') {        
        abs_X = (received_array[1] | received_array[2]<<4 | received_array[3]<<8 | received_array[4]<<12 | received_array[5]<<16 | received_array[6]<<20 | received_array[7]<<24 | received_array[8]<<28);
        abs_Y = (received_array[9] | received_array[10]<<4 | received_array[11]<<8 | received_array[12]<<12 | received_array[13]<<16 | received_array[14]<<20 | received_array[15]<<24 | received_array[16]<<28);
        abs_Theta = (received_array[17] | received_array[18]<<4 | received_array[19]<<8 | received_array[20]<<12 | received_array[21]<<16 | received_array[22]<<20 | received_array[23]<<24 | received_array[24]<<28);
        SendAck();
     }
     //Reset pozicije na nulu
     else if (komanda == 'c') {        
        abs_X=0;
        abs_Y=0;
        abs_Theta=0;     
        SendAck();
     }
    //Emergency stop
     else if (komanda == 'd') {
        zadata_pozicija_X=trenutna_pozicija_X;
        zadata_pozicija_Y=trenutna_pozicija_Y;
        SendAck();
     }
     /* Paljenje UV senzora. */
     else if(komanda == 's')
     {
       FLAG_sensorEnable = TRUE;
       SendAck();
     }
     /* Gasenje UV senzora. */
     else if(komanda == 'q')
     {
       FLAG_sensorEnable = FALSE;
       SendAck();
     }
     else if(komanda == 'p')
     {
       SendDestFlag();
     }
     else if (komanda == 'n') {
        running=TRUE;
        SendAck();
     }
  }
};


//PITATI JOVICICA
void SendLog(void)
{
        sending_length=4;//sending_length je . pored inforamcionih N informacionih bajtova salje se jos 4 dodatna bajta. dakle ako je max 256 korisnih podataka duzina poruke je 260
        sending_array[0]=0xFF;
        sending_array[1]=ADDR|0x40;
        sending_array[2]=sending_length-3;        
        schksum=0;
        for(int i=1;i<sending_length-1;i++){
          schksum+=sending_array[i]&0xFF;
        }
        sending_array[3]=schksum&0x7F;//0x4B
      
        USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
        USART_ITConfig(USART3, USART_IT_TXE, ENABLE);

};

void SendLogSingle(int index)
{
        sending_length=8;//sending_length je . pored inforamcionih N informacionih bajtova salje se jos 4 dodatna bajta. dakle ako je max 256 korisnih podataka duzina poruke je 260
        sending_array[0]=0xFF;
        sending_array[1]=ADDR|0x40;
        sending_array[2]=sending_length-3;  
     
        sending_array[3]=(data_log[index] & 0x0000000F);
        sending_array[4]=(data_log[index] & 0x000000F0)>>4;
        sending_array[5]=(data_log[index] & 0x00000F00)>>8;
        sending_array[6]=(data_log[index] & 0x0000F000)>>12;   
  
        schksum=0;
        for(int i=1;i<sending_length-1;i++){
          schksum+=sending_array[i]&0xFF;
        }
        sending_array[7]=schksum&0x7F;//0x4B
      
        USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
        USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
};

void USART3_IRQHandler( void )
{  
  /* Prijem poruke. */
  if((USART_GetITStatus(USART3, USART_IT_RXNE) != RESET))
  {
    USART_ClearITPendingBit(USART3, USART_IT_RXNE);
    received_byte = USART_ReceiveData(USART3);
    if (received_byte==0xFF){
      chksum=0;
      byte_count=0;
      message_size=MAX_TRANSX_LEN;
    }
    else {          
      if (byte_count==0) {//prvi bajt
        address=received_byte;
        chksum=received_byte;
      }
      else if (byte_count==1) {//drugi bajt
        message_size=received_byte;
        chksum=chksum+received_byte;
      }
      else if (byte_count==message_size+1){//poslednji bajt - checksum-a
        //provera checksume
        if ((chksum&0x7F)==received_byte){
          //korektna checksum-a - ide na obradu
          Response();
        }
      }
      else {
        received_array[byte_count-2]=received_byte;  
        chksum=chksum+received_byte;
      }
      if (byte_count<MAX_TRANSX_LEN) byte_count++;
    }    
  }  
  
  /* Slanje poruke. */
  else if ( USART_GetITStatus( USART3, USART_IT_TC ) != RESET ) // Transmit the string in a loop
  {
    USART_ClearITPendingBit( USART3, USART_IT_TC );
    if ( sending_iterator < sending_length )
    {
        USART_SendData( USART3, sending_array[ sending_iterator ] );
        sending_iterator++;
    }
    else 
    {
      sending_iterator = 0;
      GPIO_ResetBits( GPIOC, GPIO_Pin_12 );//disabluje RS485
    }
  }
  
  //PITATI JOVICICA
  else{//ovo je valjda za onu overrun gresku 
    USART_ITConfig(USART3, USART_IT_TC, DISABLE);
    received_byte = USART_ReceiveData(USART3);
    USART3->SR=0x00000000;//D8
    USART_ClearITPendingBit(USART3, USART_IT_TXE);
    USART_ITConfig(USART3, USART_IT_TC, ENABLE);
  }
}

/******************************************************************************/
/*                          STM32F10x EXTI Handler                            */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

void EXTI2_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line2) != RESET)
  {
    /* Clear the  EXTI line pending bit */
    EXTI_ClearITPendingBit(EXTI_Line2);
    if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2) == 1) {//rastuca ivica
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5) == 1 )
              ENC1--;
        else 
              ENC1++;
    }
    else{
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5) == 1 )
              ENC1++;
        else 
              ENC1--;
    }
  }
  else err++;
}

void EXTI9_5_IRQHandler(void)
{
  
  if (EXTI_GetITStatus(EXTI_Line5) != RESET)
  {
    /* Clear the  EXTI line pending bit */
    EXTI_ClearITPendingBit(EXTI_Line5);
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5) == 1) {//rastuca ivica
        if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2) == 1 )
              ENC1++;
        else 
              ENC1--;
    }
    else{

        if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2) == 1 )
              ENC1--;
        else 
              ENC1++;
    }    
  }
  else err++;
}

void EXTI15_10_IRQHandler(void)
{  
  GPIO_SetBits(GPIOA,GPIO_Pin_5);
  EXTI_InitTypeDef EXTI_InitStructure;
  if (EXTI_GetITStatus(EXTI_Line14) != RESET)
  {
    /* Clear the  EXTI line pending bit */
    EXTI_ClearITPendingBit(EXTI_Line14);            
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14) == 1) {//rastuca ivica
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15) == 1 )
              ENC2++;
        else 
              ENC2--;
    }
    else{

        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15) == 1 )
              ENC2--;
        else 
              ENC2++;
    }    
  }
  else if (EXTI_GetITStatus(EXTI_Line15) != RESET)
  {
    /* Clear the  EXTI line pending bit */
    EXTI_ClearITPendingBit(EXTI_Line15);            
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15) == 1) {//rastuca ivica
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14) == 1 )
              ENC2--;
        else 
              ENC2++;
    }
    else{
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14) == 1 )
              ENC2++;
        else 
              ENC2--;
    }
  }
  else err++;
  GPIO_ResetBits(GPIOA,GPIO_Pin_5);
}
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


//f-ja za prijem sa uarta



void SysTick_Handler(void)
{
  int pwm_command;
  //TIM_ClearFlag(TIM2, TIM_FLAG_Update);
  
  //svakih 10ms tajmer 2 inkrementira neki pomocni brojac
  //ako brojac dodje do timeout vrednosti gase se motori
  //u suprotnom se ne gase
  //prijemnik uart-a resetuje brojac na nulu
    
   /* if(MBEDwatchdog<12){
      MBEDwatchdog++;
    }
    else {
      Speed1=0;
      Speed2=0;
    }*/
  

  if (neki_brojac>1){
    apsolutnaPozicija=calculatePosition(ENC1, ENC2);
    neki_brojac=0;
  }
  else
  {
    neki_brojac++;
  }
  
  if(flag_c==1){
    int Motor1=0,Motor2=0,MotorJaciDelta=0;

    temp.origin=polyline_path[c_counter];
    temp.point=polyline_path[c_counter+1];
    temp_point.x=apsolutnaPozicija.x;
    temp_point.y=apsolutnaPozicija.y;
    if(flag_first_step_continous && (advanced_segment_stigao==1)){
      float proba;

      proba=angular_Displacement(temp);
      Pos1=ENC1-(float)(UTC*(angular_Displacement(temp)+apsolutnaPozicija.theta));
      Pos2=ENC2+ENC1-Pos1;
      flag_following_active=FALSE;
    }else if(flag_following_active){
      MAX_SPEED=75;
      MAX_DIFF_SPEED=24;
      MAX_SPEED_TOTAL=MAX_SPEED+MAX_DIFF_SPEED;
      float till_end=0;
      stigao=0;
      till_end=point_Projection(temp_point, temp)-Intensity(temp);
    if(till_end>-4000){
        if(c_counter==polyline_length-2)
        {
            MAX_SPEED=(float)(-0.005*till_end+55);

        }
        else{
            MAX_SPEED=(float)(-0.005*till_end+55);
        }
            MAX_DIFF_SPEED=MAX_SPEED-51;
            MAX_SPEED_TOTAL=MAX_SPEED+MAX_DIFF_SPEED;
      }
      if(neki_brojac==2){
        
        err_c=point_To_Vector(temp_point, temp);
        diff_brzina=PID_continous(err_c);
      }
      Motor1=10*MAX_SPEED+10*diff_brzina;
      Motor2=10*MAX_SPEED-10*diff_brzina;
      MotorJaciDelta=Motor1>Motor2?(10*MAX_SPEED_TOTAL-Motor1):(10*MAX_SPEED_TOTAL-Motor2);
   

      TIM1->CCR1 = 10*MAX_SPEED+10*diff_brzina+MotorJaciDelta;
      TIM1->CCR2 = 10*MAX_SPEED-10*diff_brzina+MotorJaciDelta;
      if((point_Projection(temp_point, temp)>Intensity(temp))&&(c_counter<polyline_length)){ // bilo polyline_length-1
        c_counter++;
        return;
      }
//      if((Distance(temp_point, temp.point)<2000)&&(c_counter==polyline_length-1)){ // bilo polyline_length-1
//        c_counter++;
//        return;
//      }
     
      if(c_counter==polyline_length-1){
        ENC1_old = ENC1;
        ENC2_old = ENC2;
        Pos1=ENC1;
        Pos2=ENC2;
        flag_c=0;
        stigao=advanced_segment_stigao;
        c_counter=0;
        flag_first_step_continous=1;
        return;
      }
      ENC1_old = ENC1;
      ENC2_old = ENC2;     
      GPIO_SetBits(GPIOB, GPIO_Pin_15);//ENABLE M2
      
      GPIO_SetBits(GPIOB, GPIO_Pin_12);
      
      return;
      
    }
    
  }
  
  
  //brzina=PID_poz_NELIN(Pos1,ENC1);
  //brzina=PID_poz(Pos1,ENC1);
  brzina=PID_poz(trenutna_pozicija_X,ENC1);
  //brzina=Speed_profile_X(Pos1,ENC1); 
  //brzina=Speed1;
  brzina_prim=brzina;
  err = ENC1-ENC1_old;
  pwm_command = PID1(brzina,err);
  if (pwm_command>100){
    GPIO_ResetBits(GPIOA,GPIO_Pin_4);
    GPIO_SetBits(GPIOA,GPIO_Pin_10);
  }
  else if (pwm_command<-100){
    GPIO_SetBits(GPIOA,GPIO_Pin_4);
    GPIO_ResetBits(GPIOA,GPIO_Pin_10);
    pwm_command=-pwm_command;
  }
  else {
    GPIO_ResetBits(GPIOA,GPIO_Pin_4);
    GPIO_ResetBits(GPIOA,GPIO_Pin_10);
    pwm_command=990;
  };
  TIM1->CCR2 = 1000-pwm_command;
  ENC1_old = ENC1;
  stigao1 = (brzina==0);
  
  //brzina=PID_poz_NELIN(Pos2,ENC2);  // naci u ccr1 idje brzina koju zelis da ostvaris
  //brzina=PID_poz(Pos2,ENC2);
  brzina=PID_poz(trenutna_pozicija_Y,ENC2);
  //brzina=Speed_profile_Y(Pos2,ENC2);
  //brzina=Speed2;
  err = ENC2-ENC2_old;  
  pwm_command = PID2(brzina,err);
  if (pwm_command>100){
    GPIO_SetBits(GPIOC,GPIO_Pin_9);
    GPIO_ResetBits(GPIOC,GPIO_Pin_8);
  }
  else if (pwm_command<-100){
    GPIO_ResetBits(GPIOC,GPIO_Pin_9);
    GPIO_SetBits(GPIOC,GPIO_Pin_8);
    pwm_command=-pwm_command;
  }
  else {
    GPIO_ResetBits(GPIOC,GPIO_Pin_9);
    GPIO_ResetBits(GPIOC,GPIO_Pin_8);
    pwm_command=990;
  };
  TIM1->CCR1=1000-pwm_command;
  ENC2_old = ENC2;
  
  
  advanced_segment_stigao=stigao1&&(brzina==0);
  if ((flag_a==1)||(flag_c==1)){
  }
  else
  {
    stigao = advanced_segment_stigao;
  }
  
  if (advanced_segment_stigao == 1)
  { 
    
    if ((flag_a==1))
    {
      putanja_counter++;
      Pos1=putanja[putanja_counter][0];
      Pos2=putanja[putanja_counter][1];
      

      if (putanja_counter>=ulm_length){
        stigao=advanced_segment_stigao;
        flag_a=0;
        putanja_counter=0;
        ulm_length=0;
      }
    }
  }
  if(flag_first_step_continous&&(flag_following_active==FALSE)&&advanced_segment_stigao){
    flag_first_step_continous=FALSE;
    flag_following_active=TRUE;
  }
  
  //USART_SendData(USART3, err);
  //USART_SendData(USART3, ((brzina == 0) ? 43 : 91));
  
  if ((brzina==0)){
    //GPIO_ResetBits(GPIOB, GPIO_Pin_12);//DISABLE M1
    GPIO_ResetBits(GPIOB, GPIO_Pin_15);//DISABLE M2
  }
  else {
    //GPIO_SetBits(GPIOB, GPIO_Pin_12);//ENABLE M1
    GPIO_SetBits(GPIOB, GPIO_Pin_15);//ENABLE M2
  };
  if ((brzina_prim==0)){
    GPIO_ResetBits(GPIOB, GPIO_Pin_12);//DISABLE M1
    //GPIO_ResetBits(GPIOB, GPIO_Pin_15);//DISABLE M2
  }
  else {
    GPIO_SetBits(GPIOB, GPIO_Pin_12);//ENABLE M1
    //GPIO_SetBits(GPIOB, GPIO_Pin_15);//ENABLE M2
  };
  
  /*if(otvori&&(ruka_pozicija<ruka_otvorena)){
    ruka_pozicija++;
    TIM3->CCR3=ruka_pozicija;
  } else if(zatvori&&(ruka_pozicija>ruka_zatvorena)){
    ruka_pozicija--;
    TIM3->CCR3=ruka_pozicija;
  }else{
    otvori=zatvori=0;
  }*/
} 

int CTPWM(int crtice){
  return crtice;
}
int PWMTC(int pwm){
  return pwm;
}
int PID_continous(float gr){
  static float Kpc=0.03, Kic=0, Kdc=0.7, Kac=0.5;
  static int counter=0;
  static float delta_err,err_previous=0;
  float Propc, Difc, Propac;
  int  Reg;
 // float anglePID=0, formerAnglePID, deltaAngle;
 // formerAnglePID=anglePID;
  counter++;
 // anglePID=((int)(angular_Displacement(temp)+apsolutnaPozicija.theta+720))%360;
 // anglePID=anglePID<180?anglePID:(anglePID-360);
//  deltaAngle=anglePID-formerAnglePID;
  delta_err = gr-err_previous;
  Propc =Kpc * (float)gr;
//  Propac=(float)(Kac*(deltaAngle>10?deltaAngle:0));
  Difc = Kdc * (float)(delta_err);
//  Intc=Intc+(float)(Kic*Propc)+Propac;
  if(Intc>MAX_DIFF_SPEED)Intc=MAX_DIFF_SPEED;
  if(Intc<-MAX_DIFF_SPEED)Intc=-MAX_DIFF_SPEED;
  //if ((gr>0?gr:-gr)<30){ Intc=0;}
  Reg = (int)(Propc + Difc + Intc);
  if (Reg>MAX_DIFF_SPEED) Reg=MAX_DIFF_SPEED;
  if (Reg<-MAX_DIFF_SPEED) Reg=-MAX_DIFF_SPEED;
  err_previous=gr;
  return Reg;  
}
int PID1(int zeljena, int trenutna)
{
  int greska, Reg;
  int speed;
  float Prop,Dif;
  greska = zeljena-trenutna;
  Prop = Kp * (float)greska;
  Dif = Kd * (float)(greska - prethodna_greska1);
  //if (((greska - prethodna_greska1)>-2)&&((greska - prethodna_greska1)<2)) Int1=0;
  //if ((greska >-4)&&(greska<4)&&(Int1>0)) Int1=Int1-4*Ki;
  //if ((greska >-4)&&(greska<4)&&(Int1<0)) Int1=Int1+4*Ki;
  Int1 += Ki * (float)(greska); // integral part
  if (Int1>900) Int1=900;
  if (Int1<-900) Int1=-900;
  Reg = (int)(Prop + Dif + Int1);
  if (Reg>990) Reg=990;
  if (Reg<-990) Reg=-990; 
  //if ((greska ==0)&&(speed_current_X==0)) {Reg=0; Int1=0;}
  if ((greska<1)&&(greska>-1)&&(speed_current_X==0)) {Reg=0; Int1=0;}
  prethodna_greska1=greska;
  //return (Reg-500)*SCALE+500;
  
  return Reg;  
}

int PID2(int zeljena, int trenutna)
{
  int greska, Reg;
  float Prop,Dif;
  greska = zeljena-trenutna;
  Prop = Kp * (float)greska;
  Dif = Kd * (float)(greska - prethodna_greska2);
  //if ((greska<2)&&(greska>-2)) Int2=0;
  Int2 += Ki * (float)(greska); // integral part
  if (Int2>900) Int2=900;
  if (Int2<-900) Int2=-900;
  Reg = (int)(Prop + Dif + Int2);
  if (Reg>990) Reg=990;
  if (Reg<-990) Reg=-990;  
  //if ((greska ==0)&&(greska==prethodna_greska2)) {Reg=0; Int2=0;}
  //if ((greska ==0)&&(speed_current_Y==0)) {Reg=0; Int2=0;}
  if ((greska<1)&&(greska>-1)&&(speed_current_Y==0)) {Reg=0; Int2=0;}
  prethodna_greska2=greska;
  //return (Reg-500)*SCALE+500;  
  return Reg; 
}



/*
int PID_poz(int zeljena, int trenutna)
{
int greska, Reg;
float Prop,Dif;
greska = zeljena-trenutna;
Prop = Kp_poz * (float)greska;
Dif = Kd_poz * (float)(greska - prethodna_greska);
Int_poz += Ki_poz * (float)(greska); // integral part
if (Int_poz>50) Int_poz=50;
if (Int_poz<-50) Int_poz=-50;
Reg = (int)(Prop + Dif + Int_poz);
if (Reg>100) Reg=100;
if (Reg<-100) Reg=-100;
prethodna_greska_poz=greska;
return Reg;  
}*/

int PID_poz_NELIN(int zeljena, int trenutna)
{
  int greska, Reg;
  float Prop,Dif;
  greska = zeljena-trenutna;
  if (greska>speed_ref_pos[9]) return 100*SCALE;
  else if (greska>speed_ref_pos[8]) return 90*SCALE; 
  else if (greska>speed_ref_pos[7]) return 80*SCALE; 
  else if (greska>speed_ref_pos[6]) return 70*SCALE; 
  else if (greska>speed_ref_pos[5]) return 60*SCALE; 
  else if (greska>speed_ref_pos[4]) return 50*SCALE; 
  else if (greska>speed_ref_pos[3]) return 40*SCALE; 
  else if (greska>speed_ref_pos[2]) return 30*SCALE; 
  else if (greska>speed_ref_pos[2]) return 20*SCALE;
  else if (greska>speed_ref_pos[0]) return 10*SCALE; 
  else if (greska>0) return 0;  
  
  greska=-greska;
  if (greska>speed_ref_pos[9]) return -100*SCALE;
  else if (greska>speed_ref_pos[8]) return -90*SCALE; 
  else if (greska>speed_ref_pos[7]) return -80*SCALE; 
  else if (greska>speed_ref_pos[6]) return -70*SCALE; 
  else if (greska>speed_ref_pos[5]) return -60*SCALE; 
  else if (greska>speed_ref_pos[4]) return -50*SCALE; 
  else if (greska>speed_ref_pos[3]) return -40*SCALE; 
  else if (greska>speed_ref_pos[2]) return -30*SCALE; 
  else if (greska>speed_ref_pos[1]) return -20*SCALE;
  else if (greska>speed_ref_pos[0]) return -10*SCALE; 
  else return 0;   
  
}

int PID_poz(int zeljena, int trenutna)
{
  int greska, Reg;
  float Prop,Dif;
  greska = zeljena-trenutna;
  Reg=SCALE*greska/12;
  //if (Reg>90) Reg=90;
  //if (Reg<-90) Reg=-90;
  return Reg; 
}

/**
  * @brief  Proverava da li je robot stigao na cilj.
  * @param  Nema ulaznih argumenata.
  * @retval Vraca TRUE ako je robot stigao na cilj inace vraca FALSE.
  * @author Praetorian ( archmarko92@gmail.com )
  */
_Bool checkIfAtDest( void )
{
  /* Definisanje okoline cilja u kojoj se smatra da je motor stigao na cilj. */
  int dest_low_bound_m1 = zadata_pozicija_X - PROXIMITY_CONSTANT;
  int dest_high_bound_m1 = zadata_pozicija_X + PROXIMITY_CONSTANT;
  int dest_low_bound_m2 = zadata_pozicija_Y - PROXIMITY_CONSTANT;
  int dest_high_bound_m2 = zadata_pozicija_Y + PROXIMITY_CONSTANT;
  
  
  /* Provera da li smo stigli na cilj. */
  if ( trenutna_pozicija_X >= dest_low_bound_m1 && trenutna_pozicija_X <= dest_high_bound_m1 &&
       trenutna_pozicija_Y >= dest_low_bound_m2 && trenutna_pozicija_Y <= dest_high_bound_m2 ) return TRUE;
  else return FALSE;
  
}
/*----------------------------------------------------------------------------*/

/**
  * @brief  Proverava senzore da li je detektovana prepreka i vraca TRUE ako jeste.
  * @param  Nema ulaznih argumenata.
  * @retval Vraca TRUE ako je prepreka detektovana inace vraca FALSE.
  * @author Praetorian ( archmarko92@gmail.com )
  */
_Bool checkForObstacle( void )
{
  /* Deaktiviranje flag-a koji signalizira detektovanu prepreku ako smo stigli na poziciju, inace ga ne diramo. */
  if( checkIfAtDest() ) FLAG_obstacleDetected = FALSE;
  
  /* Provera da li se senzori gledaju. */
  if( FLAG_sensorEnable )
  {
    /* Provera da li se gledaju prednji senzori. */
    if( FLAG_sensorFrontEnable )
    {
      if( ( ( distance_fr_ch3 <= MAX_DISTANCE_MM ) || ( distance_fl_ch4 <= MAX_DISTANCE_MM ) ) && !FLAG_obstacleDetected )
         return TRUE;
      else
         return FALSE;
    }
    
    /* Provera da li se gledaju zadnji senzori. */
    else if( FLAG_sensorBackEnable )
    {
      if( ( ( distance_bl_ch1 <= MAX_DISTANCE_MM ) || ( distance_br_ch2 <= MAX_DISTANCE_MM ) ) && !FLAG_obstacleDetected )
         return TRUE;
      else
         return FALSE;
    }
    
    /* Nijedni senzori se ne gledaju. */
    else return FALSE;
  }
  
  /* Ne gledaju se senzori. */
  else return FALSE;
              
}
/*----------------------------------------------------------------------------*/
              

/**
  * @brief  Staje sa lepim usporavanjem ako detekuje prepreku.
  * @param  Nema ulaznih argumenata.
  * @retval Vraca TRUE ako se staje usled prepreke, inace se vraca FALSE.
  * @author Praetorian ( archmarko92@gmail.com )
  */
_Bool stopIfObstacle( void )     
{
   /* Ako je detekovana prepreka zapocni zaustavljanje. */
   if( checkForObstacle() && !checkIfAtDest() )
   { 
     
     /* Provera da li se robot krece napred. */
     if( zadata_pozicija_X > trenutna_pozicija_X && zadata_pozicija_Y > trenutna_pozicija_Y )
     {
       FLAG_obstacleDetected = TRUE;
       zapamcena_pozicija_X = zadata_pozicija_X;
       zapamcena_pozicija_Y = zadata_pozicija_Y;
       zadata_pozicija_X = trenutna_pozicija_X + STOP_DISTANCE;
       zadata_pozicija_Y = trenutna_pozicija_Y + STOP_DISTANCE;
       return TRUE;
     }
     
     /* Provera da li se robot krece unazad. */
     else if ( zadata_pozicija_X < trenutna_pozicija_X && zadata_pozicija_Y < trenutna_pozicija_Y )
     {
       FLAG_obstacleDetected = TRUE;
       zapamcena_pozicija_X = zadata_pozicija_X;
       zapamcena_pozicija_Y = zadata_pozicija_Y;
       zadata_pozicija_X = trenutna_pozicija_X - STOP_DISTANCE;
       zadata_pozicija_Y = trenutna_pozicija_Y - STOP_DISTANCE;
       return TRUE;
     }
     
     /* Ne radi nista ako je rotaciono kretanje. */
     else return FALSE;
   }
   
   /* Ako je detektovana prepreka a u poziciji smo. */
   else if( !checkForObstacle() && checkIfAtDest() )
   {
     zadata_pozicija_X = zapamcena_pozicija_X;
     zadata_pozicija_Y = zapamcena_pozicija_Y;
   }
   
   /* Ako nije detekovana prepreka ne radi nista. */
   else return FALSE;
}
/*----------------------------------------------------------------------------*/

/**
  * @brief  Prekidna rutina tajmera 4. Merenje udaljenosti od objekata pomocu
  *         ultrazvucnih senzora.
  * @param  Nema ulaznih argumenata.
  * @retval Nema izlaznih argumenata.
  * @author Milica Stojiljkovic
  */
void TIM4_IRQHandler( void )
{
  /* Provera da li je stigao zahtev za prekid od kanala 1. Senzor nazad, levo. */
  if ( TIM_GetITStatus( TIM4, TIM_IT_CC1 ) == SET )
  {
    TIM_ClearITPendingBit( TIM4, TIM_IT_CC1 );
    
    /* Masina stanja za racunanje udaljenosti prepreke od senzora. */
    switch( state_bl_ch1 )
    {
      /* Hvatanje uzlazne ivice. */
      case 0: 
        
          measured_time_bl_rising_ch1 = TIM_GetCapture1( TIM4 );
          TIM_OC1PolarityConfig( TIM4, TIM_ICPolarity_Falling );
          state_bl_ch1 = 1;
          break;
      
      /* Hvatanje silazne ivice. */
      case 1:
        
          measured_time_bl_falling_ch1 = TIM_GetCapture1( TIM4 );
          TIM_OC1PolarityConfig( TIM4, TIM_ICPolarity_Rising );  // sada hvatamo uzlaznu ivicu
          state_bl_ch1 = 0;
          
          /* Racunanje udaljenosti objekta od senzora. */
          if( measured_time_bl_falling_ch1 > measured_time_bl_rising_ch1 )
               measured_time_bl_ch1 = measured_time_bl_falling_ch1 - measured_time_bl_rising_ch1;
          else measured_time_bl_ch1 = ECHO_TIMER_PERIOD - measured_time_bl_rising_ch1 + measured_time_bl_falling_ch1;
          
          /* Novu izmerenu vrednost dodajemo u niz. */
          measured_distance_bl_ch1[ ( index_ch1++ ) % MEASUREMENT_NUM ] =  measured_time_bl_ch1; 
          
          /* Provera da li smo prosli prvih par losih merenja. */
          if ( index_ch1 > MEASUREMENT_NUM ) bad_measurement_end_ch1 = TRUE;
          
          /* Ako smo prosli prvih par losih merenja, mozemo da racunamo udaljenost od objekta. */
          if ( bad_measurement_end_ch1 )
          {
            int temp_distance = 0;
            for ( int j = 0; j < MEASUREMENT_NUM; j++ ) temp_distance += measured_distance_bl_ch1[ j ];
            distance_bl_ch1 = temp_distance / MEASUREMENT_NUM;
            distance_bl_ch1 /= CONST_MICRO_SEC_TO_MM;
          }
          else
          {
            distance_bl_ch1 = measured_distance_bl_ch1[ index_ch1 ] / CONST_MICRO_SEC_TO_MM;
          }
          
          break;
          
          
       default:
        
          break; 
    }
  }
  
  
  /* Provera da li je stigao zahtev za prekid od kanala 2. Senzor nazad, desno. */
  else if ( TIM_GetITStatus( TIM4, TIM_IT_CC2 ) == SET )
  {
    TIM_ClearITPendingBit( TIM4, TIM_IT_CC2 );
    
    /* Masina stanja za racunanje udaljenosti prepreke od senzora. */
    switch(state_br_ch2)
    {
      /* Hvatanje uzlazne ivice. */
      case 0: 
      {
          measured_time_br_rising_ch2 = TIM_GetCapture2( TIM4 );
          TIM_OC2PolarityConfig( TIM4, TIM_ICPolarity_Falling );
          state_br_ch2 = 1;
          break;
      }
      
      /* Hvatanje silazne ivice. */
      case 1:
        
          measured_time_br_falling_ch2 = TIM_GetCapture2( TIM4 );
          TIM_OC2PolarityConfig( TIM4, TIM_ICPolarity_Rising );  // sada hvatamo uzlaznu ivicu
          state_br_ch2 = 0;
          
          /* Racunanje udaljenosti objekta od senzora. */
          if( measured_time_br_falling_ch2 > measured_time_br_rising_ch2 )
               measured_time_br_ch2 = measured_time_br_falling_ch2 - measured_time_br_rising_ch2;
          else measured_time_br_ch2 = ECHO_TIMER_PERIOD - measured_time_br_rising_ch2 + measured_time_br_falling_ch2;
          
          /* Novu izmerenu vrednost dodajemo u niz. */
          measured_distance_br_ch2[ ( index_ch2++ ) % MEASUREMENT_NUM ] =  measured_time_br_ch2; 
          
          /* Provera da li smo prosli prvih par losih merenja. */
          if ( index_ch2 > MEASUREMENT_NUM ) bad_measurement_end_ch2 = TRUE;
          
          /* Ako smo prosli prvih par losih merenja, mozemo da racunamo udaljenost od objekta. */
          if ( bad_measurement_end_ch2 )
          {
            int temp_distance = 0;
            for ( int j = 0; j < MEASUREMENT_NUM; j++ ) temp_distance += measured_distance_br_ch2[ j ];
            distance_br_ch2 = temp_distance / MEASUREMENT_NUM;
            distance_br_ch2 /= CONST_MICRO_SEC_TO_MM;
          }
          else
          {
            distance_br_ch2 = measured_distance_br_ch2[ index_ch2 ] / CONST_MICRO_SEC_TO_MM;
          }
          
          break;
          
          
       default:
        
          break; 
    }
  }
  
  
  /* Provera da li je stigao zahtev za prekid od kanala 3. Senzor napred, desno. */
  else if ( TIM_GetITStatus(TIM4, TIM_IT_CC3) == SET )
  {
    TIM_ClearITPendingBit( TIM4, TIM_IT_CC3 );
    
    /* Masina stanja za racunanje udaljenosti prepreke od senzora. */
    switch( state_fr_ch3 )
    {
      /* Hvatanje uzlazne ivice. */
      case 0: 
      {
          measured_time_fr_rising_ch3 = TIM_GetCapture3( TIM4 );
          TIM_OC3PolarityConfig( TIM4, TIM_ICPolarity_Falling ); // sada hvatamo silaznu ivicu
          state_fr_ch3 = 1;
          break;
      }
        
      /* Hvatanje silazne ivice. */
      case 1:
          measured_time_fr_falling_ch3 = TIM_GetCapture3( TIM4 );
          TIM_OC3PolarityConfig( TIM4, TIM_ICPolarity_Rising );  // sada hvatamo uzlaznu ivicu
          state_fr_ch3 = 0;
          
          /* Racunanje udaljenosti objekta od senzora. */
          if( measured_time_fr_falling_ch3 > measured_time_fr_rising_ch3 )
               measured_time_fr_ch3 = measured_time_fr_falling_ch3 - measured_time_fr_rising_ch3;
          else measured_time_fr_ch3 = ECHO_TIMER_PERIOD - measured_time_fr_rising_ch3 + measured_time_fr_falling_ch3;
          
          /* Novu izmerenu vrednost dodajemo u niz. */
          measured_distance_fr_ch3[ ( index_ch3++ ) % MEASUREMENT_NUM ] =  measured_time_fr_ch3; 
          
          /* Provera da li smo prosli prvih par losih merenja. */
          if ( index_ch3 > MEASUREMENT_NUM ) bad_measurement_end_ch3 = TRUE;
          
          /* Ako smo prosli prvih par losih merenja, mozemo da racunamo udaljenost od objekta. */
          if ( bad_measurement_end_ch3 )
          {
            int temp_distance = 0;
            for ( int j = 0; j < MEASUREMENT_NUM; j++ ) temp_distance += measured_distance_fr_ch3[ j ];
            distance_fr_ch3 = temp_distance / MEASUREMENT_NUM;
            distance_fr_ch3 /= CONST_MICRO_SEC_TO_MM;
          }
          else
          {
            distance_fr_ch3 = measured_distance_fr_ch3[ index_ch3 ] / CONST_MICRO_SEC_TO_MM;
          }
          
          break;
          
          
       default:
        
          break; 
    }
  }
  
  
  /* Provera da li je stigao zahtev za prekid od kanala 4. Senzor napred, levo. */
  else if ( TIM_GetITStatus( TIM4, TIM_IT_CC4 ) == SET )
  {
    TIM_ClearITPendingBit( TIM4, TIM_IT_CC4 );
    
    /* Masina stanja za racunanje udaljenosti prepreke od senzora. */
    switch( state_fl_ch4 )
    {
      /* Hvatanje uzlazne ivice. */
      case 0: 
      {
          measured_time_fl_rising_ch4 = TIM_GetCapture4( TIM4 );
          TIM_OC4PolarityConfig( TIM4, TIM_ICPolarity_Falling ); // sada hvatamo silaznu ivicu
          state_fl_ch4 = 1;
          break;
      }
      
      /* Hvatanje silazne ivice. */
      case 1:
        
          measured_time_fl_falling_ch4 = TIM_GetCapture4( TIM4 );
          TIM_OC4PolarityConfig( TIM4, TIM_ICPolarity_Rising );  // sada hvatamo uzlaznu ivicu
          state_fl_ch4 = 0;
          
          /* Racunanje udaljenosti objekta od senzora. */
          if( measured_time_fl_falling_ch4 > measured_time_fl_rising_ch4 )
               measured_time_fl_ch4 = measured_time_fl_falling_ch4 - measured_time_fl_rising_ch4;
          else measured_time_fl_ch4 = ECHO_TIMER_PERIOD - measured_time_fl_rising_ch4 + measured_time_fl_falling_ch4;
          
          /* Novu izmerenu vrednost dodajemo u niz. */
          measured_distance_fl_ch4[ ( index_ch4++ ) % MEASUREMENT_NUM ] =  measured_time_fl_ch4; 
          
          /* Provera da li smo prosli prvih par losih merenja. */
          if ( index_ch4 > MEASUREMENT_NUM ) bad_measurement_end_ch4 = TRUE;
          
          /* Ako smo prosli prvih par losih merenja, mozemo da racunamo udaljenost od objekta. */
          if ( bad_measurement_end_ch4 )
          {
            int temp_distance = 0;
            for ( int j = 0; j < MEASUREMENT_NUM; j++ ) temp_distance += measured_distance_fl_ch4[ j ];
            distance_fl_ch4 = temp_distance / MEASUREMENT_NUM;
            distance_fl_ch4 /= CONST_MICRO_SEC_TO_MM;
          }
          else
          {
            distance_fl_ch4 = measured_distance_fl_ch4[ index_ch4 ] / CONST_MICRO_SEC_TO_MM;
          }
          
          break;
          
          
       default:
        
          break; 
    }
  }
  
  
  /* Provera da li je detektovana prepreka i da li treba stati posle svakog novog merenja. */
  stopIfObstacle();
  
}
/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/

/**
* @brief  This function handles External line0 interrupt request.
* @param  None
* @retval None
*/


/**
  * @brief  Prekidna rutina tajmera 4. Merenje udaljenosti od objekata pomocu
  *         ultrazvucnih senzora.
  * @param  Nema ulaznih argumenata.
  * @retval Nema izlaznih argumenata.
  * @author Milica Stojiljkovic
  */
void TIM6_DAC_IRQHandler( void )
{
  /* Provera da li je stigao zahtev za prekid od kanala 1. Senzor nazad, levo. */
  if ( TIM_GetITStatus( TIM6, TIM_IT_Update ) == SET )
  {
    TIM_ClearITPendingBit( TIM6, TIM_IT_Update );
    if(cnt_90 == 90) running = FALSE;
    else cnt_90++;

  }
}
/******************************************************************************/

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
