/**
*	@file: 	  EUROBOT_movement.h
*	@author:  Praetorian
*	@version: v1.01.280216.0244
*	@date: 	  28/02/2016
*	@brief:	  Deklaracije funkcija potrebnih za kretanje robota.
*/


#ifndef __EUROBOT_MOVEMENT_H__
#define __EUROBOT_MOVEMENT_H__

// Struktura koja predstavlja apsolutne koordinate.
typedef struct
{
	long x;
	long y;
	int angle;  // Ugao u odnosu na pozitivni deo x-ose.
} AbsolutePosition;

// Inicijalizacija PID-ova.
typedef struct {
	double pGain;      // Proporcionalna konstanta.
	double iGain;      // Integralna konstanta.
	double dGain;      // Diferencijalna konstanta.
	double iState;     // Cuva akumulaciju integralne greske.
	double dState;     // Cuva trenutnu brzinu prethodnog merenja.
	double iMin, iMax; // Granicne vrednosti za integralnu akumulaciju.
} PID;

// Sadrzi inforamcije o predjenom putu motora i njegovoj destinaciji.
typedef struct {
        int ID;
        int ENC_current;
        int ENC_dest;
        int ENC_old;
        int desired_velocity;
        int current_velocity;
        int velocity_index;
        char motion_type;
        char state;
        PID velocity_pid;
        PID position_pid;
} Motor;

// Funkcija koja izracunava trenutnu poziciju robota u apsolutnom koordinatnom sistemu.
void updateCurrentPosition(int, int);
// Funkcija koja vraca trenutnu poziciju robota u apsolutnom koordinatnom sistemu.
AbsolutePosition getCurrentPosition();
// Funkcija koja predstavlja komandu da se robot krece napred za zeljeni broj milimetara.
void moveForewardCmd(long);
// Funkcija koja predstavlja komandu da se robot krece unazad za zeljeni broj milimetara.
void moveBackwardCmd(long);
// Funkcija koja predstavlja komandu da se robot okrene za odredjeni ugao.
void rotateCmd(int);
// Funkcija koja prebacuje milimetre u otkucaje enkodera.
int mmToCPR(long);
// Funkcija koja prebacuje ugao u otkucaje enkodera.
int angleToCPR(int);
// Odredjuje zeljenu brzinu robota na osnovu trenutne pozicije  i zeljene pozicije.
void desiredVelocity(Motor *motor, int desired_position, int current_position);
// Funkcija koja odredjuje zeljeni duty cycle na osnovu trenutne i zeljene brzine.
int desiredPWM(PID *pid, int desired_velocity, int current_velocity);
// Korigovanje brzine pratioca u odnosu na vodju.
void velocityCorrection(Motor *leader, Motor *follower);
// Postavlja signale na drajeru motora tako da se on krece unapred.
void setDriverForward(int);
// Postavlja signale na drajeru motora tako da se on krece unazad.
void setDriverBackward(int);

#endif