/**
*	@file: 	  EUROBOT_serial.c
*	@author:  Jovan Blanusa (jovan.blanusa@gmail.com)
*	@version: v1.02.230116.0344 
*	@date: 	  23.1.2016
*	@brief:	  API funkcije koje olaksavaju komunikaciju pojedinih STM-ova u
*                 robotu. Ideja je da korisnik ovih funkcija ne misli o formatu
*                 poruke i eventualnim greskama do kojih bi moglo doci prilikom
*                 slanja poruke. Da bi ispravno radila biblioteka potrebno je 
*                 definisati funkciju DecodeCommand() koja se obavlja u prekidnoj 
*                 rutini posto se primi cela poruka.
*/

/* Potrebno jos uraditi:
      - Resiti problem integriteta poruke. Ako se zaredom salju dve poruke pomocu SendMessage
            da ne moze druga poruka da prebrise prvu, nego da se obe posalju.

      - Dodeliti adrese uredjajima, potom ispitati nacin dekodovanja komande i slanje ACK    
         
      - Poruka se trenutno salje kao string, pa samim tim ne sme ni jedan bajt biti
            0, da li ce ovo smetati ili je potrebno napraviti novi nacin slanja 
            poruke?

      - Dodati da ako je uredjaj SLAVE, da automatski odredjuje adresu, dok master mora
            da unosi adresu stalno

      - Napisati dokument koji detaljnije objasnjava protokol slanja poruka i koja 
            objasnjava kako koja funkcija radi
                    
*/

#ifndef __EUROBOT_SERIAL_H__
#define __EUROBOT_SERIAL_H__

#include "stm32f10x.h"
#include "stm32f10x_conf.h"

// Definise se koji se kanal koristi za komunikaciju
#define RS485 USART3

// Adresa mastera na magistrali
#define MASTER_ADDR 0xBB

//Adresa ovog uredjaja
#define THIS_ADDR 0xAA

#define SLAVE //MASTER
/******************************************************************************/
/************************** API funkcije **************************************/
/******************************************************************************/

// Slanje poruke
void SendMessage(unsigned char address, unsigned char* message);
// Dohvatanje poslednje validne poruke
char* GetMessage();
// Dohvatanje adrese posledenje validne poruke
char GetAddress();
// Slanje ACK signala kao odgovor da je poruka primljena
void SendACK(unsigned char address);
// Iniciranje RS485 komunikacije
void initEurobotRS485(uint32_t BaudRate, uint8_t master_addr, uint8_t this_addr, int is_master);
// Pocetak RS485 komunikacije
void EnableRS485();
// Kraj RS485 komunikacije
void DisableRS485();


/******************************************************************************/
/******************************************************************************/

// Korisnik treba da definise sledecu funkciju da bi odredio sta program radi po
// pristizanju poruke
    
/**
*   @brief: Funkcija koja se izvrsava po pristizanju validne poruke sa U
*   @param: Broj USART kanala sa kog je pristigla poruka
*   @return: Nema povratnu vrednost
*
*/    
void DecodeCommand();


#endif