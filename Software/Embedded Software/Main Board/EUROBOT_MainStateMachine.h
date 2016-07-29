#ifndef _EUROBOT_MSM_
#define _EUROBOT_MSM_

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_conf.h"
#include "EUROBOT_Init.h"
#include "EUROBOT_serial.h"
#include "stdlib.h"

/* DEFINES */
#define slaveAddress (0xAA)
#define maxAckTime (20)    // max vrednost za koju bi trebalo da stigne ACK po profesoru (u ms)

/* VARIABLES*/

extern int leftRight = 1;  // prekidac koji govori da li smo levi (1) ili desni (0) takmicar; POVEZATI HARDVERSKI!
extern int PositionArrayLeft[12]={0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // otkucaji enkodera ako smo levi
extern int PositionArrayRight[12]={0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // otukcaji enkodera ako smo desni
extern int PositionArray[12]={0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // otkucaji enkodera koji su za ovaj mec
extern int ack = 0;         // fleg koji kaze da li je stigao ACK signal;
extern int stop = 0;        // fleg koji kaze da treba sa zausatvimo
extern int start = 0;       // fleg koji kaze da treba da krenemo sa igrom
extern int gimme = 0;       // fleg koji javlja da treba da se traze novi podaci o kretanju i senzorima; POSTAVLJA PREKIDNA RUTINA
extern int stateRobot = 0;
extern unsigned char* movementOrientation = "f"; // pravac (orijentacija) kretanja
char flagMotorInPosition = 0; // fleg koji se prima od druge ploce, da li je zavrsio sa kretanjem
unsigned char* arrive = "arrive"; // string[9] poruke koju smo dobili od druge ploce treba da bude "arrive" da bi znali da smo 
                         //stigli do kraja kretanja
extern unsigned char* messageNextPos; // poruka koju saljemo drugoj ploci za sledecu poziciju

/* FUNCTION DEFINITION */


/**
  * @brief  Inicijalizacija tajmera TIM3 koji odbrojava trajanje meca, 90s. Korak je 1ms, period 1s.
  * @param  nema.
  * @retval nema.
  */
void initTimer90() 
{
  InitTIM_TimeBase(TIM3, 24000 - 1, 1000 - 1, TIM_CounterMode_Up, TIM_CKD_DIV1, 0);
  TIM_Cmd(TIM3, ENABLE);
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
  InitNVICChannel(TIM3_IRQn, 0, 0, ENABLE);
};

/**
  * @brief  Inicalizacija tajmera dva da generise prekid na 100ms, i tada trazimo od druge ploce otkucaje enkodera
  * @param  nema.
  * @retval nema.
  */
void initTimer100() 
{
  InitTIM_TimeBase(TIM2, 24000 - 1, 100 - 1, TIM_CounterMode_Up, TIM_CKD_DIV1, 0);  // 1ms korak, a 100ms period
  TIM_Cmd(TIM2, ENABLE);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  InitNVICChannel(TIM2_IRQn, 0, 0, ENABLE); 
};

/**
  * @brief  Inicijalizacije pozicija da li smo levo ili desno
  * @param  nema.
  * @retval nema.
  */
void initLeftOrRight()
{
  if (leftRight) strcpy(PositionArray, PositionArrayLeft);
  else strcpy(PositionArray, PositionArrayRight);
}
/**
  * @brief  Dekodovanje pristigle poruke, ako je doslo "a" onda smo stigli do kraja kretanja
  * @param  nema.
  * @retval nema.
  */
void DecodeCommand()
{
  if(IsAck) ack = 1;
  else 
  {
    if (recieved.data[0] == "a") flagMotorInPosition = 1;
  }  
}
/**
  * @brief  Pripremanje podatka tipa int za slanje kao niz od sest karaktera.
  *         Najnizi bajt je poslednji clan niza. Prvi bajt je za stanje, a drugi za otkucaje enkodera.
  * @param  data je podataka tipa int koji zelimo da pretvorimo u niz od pet karaktera.
  * @retval Funkcija vraca niz od sest karaktera.
  */
unsigned char* prepareForSendPos( char data1, int data2 ) 
{
  string[ 5 ] = data1;
  string[ 4 ] = data2;
  string[ 3 ] = data2 >> 8;
  string[ 2 ] = data2 >> 16;
  string[ 1 ] = data2 >> 24;
  string[ 0 ] = "/0";
  return string;
}
/**
  * @brief  Pripremanje podatka tipa int za slanje kao niz od tri karaktera.
  *         Najnizi bajt je poslednji clan niza.
  * @param  data je podataka tipa int koji zelimo da pretvorimo u niz od tri karaktera.
  * @retval Funkcija vraca niz od tri karaktera.
  */
unsigned char* prepareForSendSens( char data1, char data2 )
{
  string[ 2 ] = data1;
  string[ 1 ] = data2;
  string[ 0 ] = "/0";
  return string;
}


#endif