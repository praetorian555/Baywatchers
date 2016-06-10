/**
*	@file: 	  EUROBOT_serial.c
*	@author:  Jovan Blanusa (jovan.blanusa@gmail.com)
*	@version: v1.03.140416.1033 
*	@date: 	  14.4.2016
*	@brief:	  API funkcije koje olaksavaju komunikaciju pojedinih STM-ova u
*                 robotu. Ideja je da korisnik ovih funkcija ne misli o formatu
*                 poruke i eventualnim greskama do kojih bi moglo doci prilikom
*                 slanja poruke. Da bi ispravno radila biblioteka potrebno je 
*                 definisati funkciju DecodeCommand() koja se obavlja u prekidnoj 
*                 rutini posto se primi cela poruka.
*/

/* Proveriti:

      - Nacin slanja ACK signala. Stavljeno je da samo SLAVE podesi ACK, po potrebi
            izmeniti to. ACK se salje u funkciji ProcessByte, case: CHECK

      - Adrese su nasumicne, potrebno je definisati adrese, i iskoristiti funkcije
            date u API-ju za komunikaciju

      - Prioritet prekida USART3 prekidne rutine je podesen u funkciji initEurobotRS485.
            Po potrebi ga promeniti, ja nisam znao koliki prioritet treba da bude.

      - Ne bi trebalo slati dosta poruka jednu za drugom(Jedna komanda SendMessage
            iza druge), komunikacija bi trebala da radi, ali ako ih ima vise, pobrljavice.

                    
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
void SendString(unsigned char address, unsigned char* message);
void SendMessage(unsigned char address, unsigned char* message, char length);
// Dohvatanje poslednje validne poruke
char* GetMessage();
// Dohvatanje adrese posledenje validne poruke
char GetAddress();
// Dohvatanje duzine posledenje validne poruke
char GetMessageLength();
// Provera da li je poslednja primljena poruka ACK
int IsAck();
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