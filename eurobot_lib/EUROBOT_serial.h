/**
*	@file: 	  EUROBOT_serial.c
*	@author:  Jovan Blanusa, Vitez21 (koprivica.slobodan92@gmail.com) starije verzije
*	@version: ?
*	@date: 	  23.1.2016
*	@brief:	  API funkcije koje olaksavaju komunikaciju pojedinih STM-ova u
*                 robotu. Ideja je da korisnik ovih funkcija ne misli o formatu
*                 poruke i eventualnim greskama do kojih bi moglo doci prilikom
*                 slanja poruke.
*/

/* Potrebno jos uraditi:
      - Utvrditi kad se tacno salje ACK i na koju adresu i ubaciti to u kod

      - Sta se desava kad je pristigla nevalidna poruka, tj kad check sume ne 
            odgovaraju ili kad je pristigao start bajt pre kraja trenutne poruke?

      - Poruka se trenutno salje kao string, pa samim tim ne sme ni jedan bajt biti
            0, da li ce ovo smetati ili je potrebno napraviti novi nacin slanja 
            poruke?

      - Proveriti sa ostatkom tima da li je potrebno da se realizuju jos neke API
            funkcije, i da li ove odgovaraju potrebama.

      - Napisati dokument koji detaljnije objasnjava protokol slanja poruka, pogotovo
            SendMessage i ExtractMessage jer se vrsi modifikacija bajtova poruke
            tako da se ne desi da neki bajt ima vrednost start bajta, pa da samim
            tim ne dodje do greske u komunikaciji.
                    
      - Videti sa timom da se funkcije DecodeCommand, ExecuteCommand i SendPosition
            iz EUROBOT_Communication fajlova koji predstavlja stariju verziju ove
            biblioteke izmeste na odgovarajuca mesta i promene da bi odgovarale 
            trenutnom API-ju za komunikaciju(Potrebno je samo da se SendPosition
            izmeni, tj da se implementira koriscenjem SendMessage jer je laksa za
            koriscenje)
*/
#ifndef __EUROBOT_SERIAL_H__
#define __EUROBOT_SERIAL_H__

#include "stm32f10x.h"
#include "stm32f10x_conf.h"

/******************************************************************************/
/************************** API funkcije **************************************/
/******************************************************************************/

// Slanje poruke
void SendMessage(unsigned char address, unsigned char* message, unsigned int usartNo);
// Dohvatanje poslednje validne poruke
char* GetMessage(unsigned int usartNo);
// Dohvatanje adrese posledenje validne poruke
char GetAddress(unsigned int usartNo);
// Slanje ACK signala kao odgovor da je poruka primljena
void SendACK(unsigned char address, unsigned int usartNo);

/******************************************************************************/
/******************************************************************************/

//Interna funkcija koja se poziva u prekidnoj rutini za odgovarajuci kanal.
void usart_interrupt(unsigned int IRQNo);

// Korisnik treba da pozove sledeci makro da bi se definisala potrebna prekidna rutina

/**
*   @brief: Makro za definisanje prekidne rutine potrebne za USART komunikaciju
*   @param: Broj USART kanala za koji se definise prekidna rutina
*   @return: Nema povratnu vrednost
*
*/
#define PREPARE_USART(no)          \
    void  USART##no##_IRQHandler(void){  \
 	usart_interrupt(no);  \
    }

// Korisnik treba da definise sledecu funkciju da bi odredio sta program radi po
// pristizanju poruke
    
/**
*   @brief: Funkcija koja se izvrsava po pristizanju validne poruke sa U
*   @param: Broj USART kanala sa kog je pristigla poruka
*   @return: Nema povratnu vrednost
*
*/    
void DecodeCommand(unsigned int IRQNo);


#endif