
#include "stm32f10x_conf.h"
#include "EUROBOT_Init.h"  // NAPOMENA: verovatno se koriste malo starije verzije
#include "EUROBOT_Servo.h"
#include "EUROBOT_serial.h"
#include "EUROBOT_MainStateMachine.h"  // fajl sa konstantama, promenljivima, tipovima i funkcijama potrebnim za glavnu masinu stanja


////*///////BOLJE DA SE U MESSAGES NALAZE POZICIJE, DA BI SE SVAKI PUT SRACUNALA RELATIVNA
int main(void)
{
    
  // pin C14 se koristi za pokretanje robota na pocetku meca: na njega treba da bude povezan kablic koji mu 'dovodi' masu
  // kad se kablic otkaci, napon na pinu preko pulap otpornika postaje Vcc i izlazi se iz idle stanja
  InitGPIOPin(GPIOC, GPIO_Pin_14, GPIO_Mode_IPU, GPIO_Speed_50MHz);
  
  // inicijalizacija serijske komunikacije
  // PROVERITI PARAMETRE
  initEurobotRS485(9600, 0xBB, slaveAddress, 0);  

  // 'inicijalizacija Systick-a' da generise prekid na svakih 100ms
  // koristi se da se od druge ploce zatraze podaci o otkucajima enkodera i flegovima UV senzora
  if(SysTick_Config(SystemCoreClock/100000))  // generise prekid na svakih 100ms
       {
         while(1);
       }
  
  // funkcija koja postavlja pocetne parametre prema tome da li smo desni ili levi takmicar
  // leftRight je za sad promenljiva, to treba da bude stanje prekidaca na robotu
  initLeftRight(leftRight);
  
  
  // program se vrti u petlji i prelazi iz stanja u stanje
  while (1)
  {
    setNextState();  // odreðuje sledeæe stanje; ovde ili nakon switch-a, verovatno je svejedno
      
    switch(nextState) {
      
    case idle:  // idle 0
      if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14)) // ako je ocitano Vcc, tj. kablic je sklonjen
        initTimer90(); // inicijalizacija tajmera koji odbrojava trajanje meca (TIM3)
        initDelayTimer();  // inicijalizacija tajmera za delay f-ju, potrebnu za servo i popravku pozicije robota (TIM2)
        start = 1;  // iz IdleState-a treba preæi u goForward
      break;  
      
    case goForward:  // goForward 1
      if (!wait) {
        for(int i = 0; i < waitAck; i++) {
          if(repair)
            SendMessage(slaveAddress, repairMessages[repair--], messageLength);
          else
            SendMessage(slaveAddress, nextMessage(cnt) /*bilo messages[cnt]*/, messageLength);
          delay(maxAckTime);  // mora se sacekati ACK ili poruka poslati ponovo
          if (ack) { // ako je ACK stigao, super, idemo dalje
              ack = 0;
              break;
          }
        }
        wait = 1;    
      }
      break;
      
    case goBackward:  // goBackward 2
      if (!wait) {
        for(int i = 0; i < waitAck; i++) {
          SendMessage(slaveAddress, nextMessage(cnt) /*bilo messages[cnt]*/, messageLength);
          delay(maxAckTime);
          if (ack) {
              ack = 0;
              break;
          }
        }
        wait = 1;
      } 
      break;
      
    case rotate:  // rotate 3
      if (!wait) {
        for(int i = 0; i < waitAck; i++) {
          if(repair) {
            SendMessage(slaveAddress, repairMessages[repair--], messageLength);
            if (repair == 0)
              movementState = 'x'; // stanje nakon repair, signalizira da se ne treba ponovo raditi repair
          }
          else
            // ne treba da se salje ovo, nego treba na osnovu currentPosition i positions[cnt] napraviti poruku
            SendMessage(slaveAddress, messages[cnt], messageLength);
          delay(maxAckTime);
          if (ack) {
              ack = 0;
              break;
          }
        }
        wait = 1;
      }  
      break;

    case stop:  // stop 4
      // ceka se dangerUV == 0
      // eventualno, moze se nesto ugasiti?
      break;
      
    case doNothing:  // doNothing 5
      // takodje, ne radi se nista, vec se ceka da timer90 promeni stanje na shade
      // opet, eventualno, moze se nesto ugasiti?
      break;
      
    case shade:  // shade 6
      if (!already)
        initParasolServo();  // inicijalizacija servoa na pinu B9, koristi se TIM4, Channel4 i delay funkcija tajmera TIM2
        TIM_Cmd(TIM4, ENABLE); // verovatno mora, zbog koriscenja starih funkcija za inicijalizaciju  
        Servo_SetPosition(TIM4, TIM_Channel_4, 60);  // za sad postoji 1 servo pa funkcija nije menjana
        already = 1;
        // eventualno neki delay(1500) a zatim se polako moze ugasiti sve
      break;
      
    case fullstop:  // fullstop
      // SADA SE OBAVEZNO GASI SVE, OSIM NAPAJANJA :-)
      break;
      
    default:
      // default moze biti i doNothing stanje
      break;
    }
    
    // ako je SysTick odbrojao 100ms i u prekidnoj rutini setovan fleg gimme, traze se podaci od druge ploce za kretanje i senzore
    if(gimme) {    
      for(int i = 0; i < waitAck; i++) {
          SendMessage(slaveAddress, "gimme", 5);  // ACK, provera
          delay(maxAckTime);
          if (ack) {
              ack = 0;
              break;
          }
      }
      gimme = 0;  // mozda treba u for-a, zavisi sta se gleda
    }
    
    if (update) {  // ako je druga ploca poslala podatke i setovane je fleg update, obradi te podatke
      updatePosition();
      update = 0;
    }
    
  }
}

