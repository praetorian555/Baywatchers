/**
*	@file: 	  EUROBOT_Communication.h
*	@author:  Praetorian
*	@version: 1.0.230216.0053
*	@date: 	  23/2/2016
*	@brief:	  Deklaracije funkcija potrebnih za komunikaciju glavnog procesora i mikrokontrolera zaduzenog za kontrolu kretanja.
*/

#ifndef __EUROBOT_COMMUNICATION_H__
#define __EUROBOT_COMMUNICATION_H__

// Funkcija koja prima jedan bajt.
void byteReceived(unsigned char);
void DecodeCommand(void);
void ExecuteCommand(char, unsigned int);
void SendAck(void);
void SendPosition(void);
#endif