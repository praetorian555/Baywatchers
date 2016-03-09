/**
*	@file: 	  EUROBOT_Movement.c
*	@author:  Praetorian
*	@version: 1.0.280216.0244 (Praetorian)
*	@date: 	  28/2/2016
*	@brief:	  Definicije funkcija potrebnih za kretanje robota.
*/

/** Includes. */
#include "EUROBOT_movement.h"
#include "stm32f10x_conf.h"
#include <math.h>

// Moras da ih deklarises sa extern u onom .c fajlu u kome zelis da ih koristis.
AbsolutePosition startPoint = {0, 0, 0};            // Krajnja tacka vektora kretanja u milimetrima.
AbsolutePosition destPoint = {0, 0, 0};             // Pocetna tacka vektora kretanja u milimetrima.
AbsolutePosition currentPosition = {0, 0, 0};       // Trenutna pozicija robota u milimetrima.
Motor motor1, motor2;    // Strukture koje sadrze sve informacije o motorima.
extern const unsigned char speed_table_decc[];
extern const unsigned char speed_table_acc[];

/** Konstante. */
#define WHEEL_RADIUS 45   // Poluprecnik koriscenog tocka [mm].
#define WHEEL_BASE 288    // Rastojanje izmedju tockova u osnovi [mm]. IZMERITI!
#define CPR 256           // Broj puta provere enkodera u jednoj rotaciji. (Counts Per Rotation - CPR).
#define N 50              // Prenosni faktor motora.
#define PI 3.141592

// Konstanta koja definise koliko jedan otkucaj enkodera odgovara predjenom putu,
// racuna se po formuli Cm = 2*PI*WHEEL_RADIUS/CPR/N.
#define Cm 0.14137


/**
*   @brief: Izracunava trenutnu poziciju robota u apsolutnom koordinatnom sistemu na osnovu
*           ocitavanja u enkoderima. Menja se globalna promenljiva currentPosition.
*   @param: Left predstavlja ocitavanja enkodera levog motora.
*   @param: Right predstavlja ocitavanja enkodera desnog motora.
*   @return: Nema povratnih vrednosti.
*
*/
void updateCurrentPosition(int Left, int Right){
	
  // Vrednosti u enkoderima tokom prethodnog merenja.
  static unsigned long formerLeft=32767, formerRight=32767;
  
  // Cuva informaciju o uglu u float formatu radi preciznosti izracunavanja.
  static float absAngle = 0;	
  
  // Pomocne promenljive.
  int D, dRight, dLeft;
  float dAngle;
  
  // Koliko je presao svaki motor od poslednjeg merenja.
  dLeft = (int)((Left-formerLeft) * Cm);
  dRight = (int)((Right-formerRight) * Cm); 
  D = (dLeft+dRight)/2;
  
  // Promena ugla u odnosu na prethodno merenje.
  dAngle = ((dLeft-dRight) / WHEEL_BASE);
  absAngle += dAngle; 
  
  // Koordinate robota u apsolutnom koordinatnom sistemu.
  currentPosition.x += (int)(D*cos(absAngle));
  currentPosition.y += (int)(D*sin(absAngle));
  currentPosition.angle = (absAngle >= 0 ? 0 : 360) + ((int)(absAngle)) % 360;
  
  formerLeft = Left;
  formerRight = Right;
}

/**
*   @brief: Vraca koordinate trenutne pozicije robota.
*   @param: Nema ulaznih argumenata.
*   @return: Vraca objekat strukture AbsolutePosition u koji su smestene trenutne koordinate robota.
*
*/
AbsolutePosition getCurrentPosition()
{
  // Odredjivanje trenutne pozicije robota.
  updateCurrentPosition(motor1.ENC_current, motor2.ENC_current);
  
  return currentPosition;
}

/**
*   @brief: Pomeranje robota napred za zeljenu vrednost.
*   @param: D predstavlja za koliko robot treba da se krece napred.
*   @return: Nema povratnih vrednosti.
*
*/
void moveForewardCmd(long D)
{
  // Odredjivanje trenutne pozicije robota.
  updateCurrentPosition(motor1.ENC_current, motor2.ENC_current);
	
  // Postavljanje nove pocetne tacke vektora kretanja.
  startPoint = currentPosition;
	
  // Postavljanje nove krajnje tacke vektora kretanja.
  destPoint.x = currentPosition.x + (int)(D*sin(currentPosition.angle));
  destPoint.y = currentPosition.y + (int)(D*cos(currentPosition.angle));
  destPoint.angle = currentPosition.angle;
	
  // Pretvaranje rastojanja u broj otkucaja enkodera.
  motor1.ENC_dest = motor1.ENC_current + mmToCPR(D);
  motor2.ENC_dest = motor2.ENC_current + mmToCPR(D);
}

/**
*   @brief: Pomeranje robota unazad za zeljenu vrednost.
*   @param: D predstavlja za koliko robot treba da se krece unazad.
*   @return: Nema povratnih vrednosti.
*
*/
void moveBackwardCmd(long D)
{
  // Odredjivanje trenutne pozicije robota.
  updateCurrentPosition(motor1.ENC_current, motor2.ENC_current);
	
  // Postavljanje nove pocetne tacke vektora kretanja.
  startPoint = currentPosition;
	
  // Postavljanje nove krajnje tacke vektora kretanja.
  destPoint.x = currentPosition.x - (int)(D*cos(currentPosition.angle));
  destPoint.y = currentPosition.y - (int)(D*sin(currentPosition.angle));
  destPoint.angle = currentPosition.angle;
	
  // Pretvaranje rastojanja u broj otkucaja enkodera.
  motor1.ENC_dest = motor1.ENC_current - mmToCPR(D);
  motor2.ENC_dest = motor2.ENC_current - mmToCPR(D);
}

/**
*   @brief: Okretanje robota za zeljeni ugao.
*   @param: dAngle predstavlja ugao za koji treba okrenuti robota.
*   @return: Nema povratnih vrednosti.
*
*/
void rotateCmd(int dAngle)
{
  // Odredjivanje trenutne pozicije robota.
  updateCurrentPosition(motor1.ENC_current, motor2.ENC_current);
	
  // Postavljanje nove pocetne tacke vektora kretanja.
  startPoint = currentPosition;
	
  // Postavljanje nove krajnje tacke vektora kretanja.
  destPoint.x = currentPosition.x;
  destPoint.y = currentPosition.y;
  destPoint.angle = currentPosition.angle + dAngle;
	
  // Pretvaranje ugla u broj otkucaja enkodera.
  if (dAngle >= 0)
  {
    motor1.ENC_dest = motor1.ENC_current - angleToCPR(dAngle);
    motor2.ENC_dest = motor2.ENC_current + angleToCPR(dAngle);
  }
  else
  {
    motor1.ENC_dest = motor1.ENC_current + angleToCPR(dAngle);
    motor2.ENC_dest = motor2.ENC_current - angleToCPR(dAngle);
  }
}

/**
*   @brief: Pretvara rastojanje iz milimetara u broj potrebnih otkucaja enkodera.
*   @param: D predstavlja rastojanje u milimetrima.
*   @return: Vraca broj otkucaja enkodera ekvivalentnih rastojanju u milimetrima.
*
*/
int mmToCPR(long D)
{
  return (int)(D / Cm);
}

/**
*   @brief: Pretvaranje ugla, zadatog u radijanima, u broj potrebnih otkucaja enkodera.
*   @param: angle presdstavlja ugao u radijanima.
*   @return: Vraca broj otkucaja enkodera ekvivalentnih zadatom uglu.
*
*/
int angleToCPR(int angle)
{
  return (int)(angle * WHEEL_RADIUS / Cm);
}

/**
*   @brief: PID kontrola DC motora robotske platforme.
*   @param: Nema ulaznih argumenata.
*   @return: Nema povratnih vrednosti.
*
*/
void pidMotionControl(void)
{
  // Odredjivanje zeljenih brzina oba motora.
  desiredVelocity(&motor1, motor1.ENC_dest, motor1.ENC_current);
  desiredVelocity(&motor2, motor2.ENC_dest, motor2.ENC_dest);
  
  // Korekcija zeljene brzine prateceg motora u odnosu na vodeceg.
  velocityCorrection(&motor1, &motor2);
  
  // Odredjivanje trenutnih brzina motora i pamcenje trenutne vrednosti u enkoderima.
  motor1.current_velocity = motor1.ENC_current - motor1.ENC_old;
  motor2.current_velocity = motor2.ENC_current - motor2.ENC_old;
  motor1.ENC_old = motor1.ENC_current;
  motor2.ENC_old = motor2.ENC_current;
  
  // Odredjivanje zeljenih duty cycle-ova za PWM-ove koji voze nase motore.
  uint16_t duty_cycle1 = desiredPWM(&(motor1.velocity_pid), motor1.desired_velocity, motor1.current_velocity);
  uint16_t duty_cycle2 = desiredPWM(&(motor2.velocity_pid), motor2.desired_velocity, motor2.current_velocity);
  
  // Postavljanje duty_cycle-ova.
  TIM_SetCompare1(TIM1, duty_cycle1);
  TIM_SetCompare2(TIM2, duty_cycle2);
}

/**
*   @brief: PID kontrola koja odredjuje zeljenu brzinu motora na osnovu trenutne pozicije i zeljene pozicije.
*   @param: motor predstavlja strukturu podataka koja sadrzi sve vazne informacije za jedan motor.
*   @param: desired_position predstavlja broj otkucaja enkodera motora koji oznacavaju stizanje
*           na zeljenu poziciju.
*   @param: current_position predstavlja broj otkucaja enkodera motora koji oznacavaju trenutnu poziciju.
*   @return: Nema povratnih argumenata.
*
*/
void desiredVelocity(Motor *motor, int desired_position, int current_position)
{
  // Oblast koja predstavlja stizanje na cilj.
  int dest_low_bound = desired_position - (int)(0.001 * desired_position);
  int dest_high_bound = desired_position + (int)(0.001 * desired_position);
  int last_index = 1999;
  int next_desired_velocity = 0;
  static int dynamic_index_inc = 0;
  int position_diff = 0;  

  switch (motor->state)
  {
    // Inicijalno stanje, motor zapocinje kretanje.
    case 'i':
             motor->desired_velocity = 0; // Zeljena brzina se postavlja na pocetnu nultu vrednost.
             motor->state = 'm';          // Stanje u koje se prelazi jeste stanje kretanja.
             motor->velocity_index = 0;   // Indeks u brzinskoj tabeli se postavlja na pocetak.
             motor->motion_type = 'a';    // Motor ubrzava.
			 
             break;
			 
    // Motor se krece i pravi svoj profil brzine.
    case 'm':
             // Provera da li smo stigli na cilj.
             if (current_position >= dest_low_bound || current_position <= dest_high_bound) motor1.state = 'w';
             // Ako nismo stigli na cilj, provera da li treba da se krecemo napred i podesavanje
             // drajvera.
             else if (desired_position > current_position) setDriverForward(motor->ID);
             // Ako nismo stigli na cilj, provera da li treba da se krecemo unazad i podesavanje
             // drajvera.
             else if (desired_position < current_position) setDriverBackward(motor->ID);
			 
             // Odredjivanje zeljene brzine i tipa kretanja.
             position_diff = (int)(abs(desired_position - current_position));
			 
             // Formula za dinamicko odredjivanje inkrementa indeksa na osnovu trenutne zeljene brzine.
             dynamic_index_inc = 1 + motor->desired_velocity;
			 
             if (position_diff > 0)
             {
               // Odredjivanje naredne zeljene brzine.
               if (position_diff > last_index) next_desired_velocity = speed_table_decc[last_index];
               else next_desired_velocity = speed_table_decc[position_diff];
			   
               // Ako motor usporava.
               if(motor->motion_type == 'd')
               {
                 if(motor->desired_velocity > next_desired_velocity)
                 {
                   motor->velocity_index -= dynamic_index_inc;
                   if(motor->velocity_index < 0) motor->velocity_index = 0;
                   motor->desired_velocity = speed_table_decc[motor->velocity_index];
                 }
                 else
                 {
                   motor->motion_type = 'a';
                   motor->velocity_index += dynamic_index_inc;
                   if(motor->velocity_index > last_index) motor->velocity_index = last_index;
                   motor->desired_velocity = speed_table_decc[motor->velocity_index];
                 }
               }
			   
               // Ako motor ubrzava.
               else 
               {
                 if (motor->desired_velocity > next_desired_velocity)
                 {
                   motor->motion_type = 'd';
                   motor->velocity_index -= dynamic_index_inc;
                   if(motor->velocity_index < 0) motor->velocity_index = 0;
                   motor->desired_velocity = speed_table_decc[motor->velocity_index];
                 }
                 else
                 {
                   motor->velocity_index += dynamic_index_inc;
                   if(motor->velocity_index > last_index) motor->velocity_index = last_index;
                   motor->desired_velocity = speed_table_decc[motor->velocity_index];
                 }
               }				   
             }

             break;
			 
    // Motor je stigao na odrediste i ceka narednu komandu.
    case 'w':			 
             // Ako smo van male okoline destinacije, znaci da je izdata komanda za kretanje.
             if (current_position < dest_low_bound || current_position > dest_high_bound) motor->state = 'i';
			 
             break;

  }
}

/**
*   @brief: PID kontrola koja kao ulaz uzima gresku u brzini a na izlazu daje novu vrednost PWM signala
*           kojim se definise brzina motora.
*   @param: pid predstavlja strukturu podataka koji sadrzi sve informacije za kontrolu zeljene greske.
*   @param: desired_velocity predstavlja zeljenu brzinu motora.
*   @param: current_velocity predstavlja trenutnu brzinu motora.
*   @return: Vraca se nova vrednost za duty cycle PWM-a koji kontrolise motor.
*
*/
int desiredPWM(PID *pid, int desired_velocity, int current_velocity)
{
  // Proporcionalne, diferencijalne i integralne komponente.
  double pTerm, dTerm, iTerm;
  
  // Greska.
  int error = desired_velocity - current_velocity;
  
  // Izracunavanje proporcionalne kontrole.
  pTerm = pid->pGain * error;   
  
  // Izracunavanje integralne kontrole sa ogranicenjima (da ne bi doslo do wind-up).
  pid->iState += error;
  if (pid->iState > pid->iMax) pid->iState = pid->iMax;
  else if (pid->iState < pid->iMin) pid->iState = pid->iMin;
  iTerm = pid->iGain * pid->iState;  // calculate the integral term
  
  // Izracunavanje diferencijalne kontrole.
  dTerm = pid->dGain * (error - pid->dState);
  pid->dState = error;
  
  // Novi duty cycle.
  int duty_cycle = (int)(pTerm + iTerm + dTerm);
  
  // Ogranicavanje maksimalne vrednosti za duty cycle.
  if (duty_cycle > 990) duty_cycle = 990;
  if (duty_cycle < -990) duty_cycle = -990;
  
  // Resetovanje prihvatnih registara ako je motor stigao u zeljenu poziciju.
  if ((error < 1) && (error > -1) && (current_velocity == 0))
  {
    duty_cycle = 0;
    pid->iState = 0;
    pid->dState = 0;
  }
  
  return duty_cycle;
   
}

/**
*   @brief: Korigovanje brzine pratioca u odnosu na vodju.
*   @param: leader predstavlja vodeci motor.
*   @param: follower predstavlja prateci motor.
*   @return: Nema povratnih vrednosti.
*
*/
void velocityCorrection(Motor *leader, Motor *follower)
{
  int position_diff1 = abs(leader->ENC_dest - leader->ENC_current);
  int position_diff2 = abs(follower->ENC_dest - follower->ENC_current);
  if (position_diff1 > position_diff2) follower->desired_velocity += 1;
  else if (position_diff1 < position_diff2) follower->desired_velocity -= 1;
}

/**
*   @brief: Postavlja signale na drajeru motora tako da se on krece unapred.
*   @param: id predstavlja jedinstven identifikator motora.
*   @return: Nema povratnih vrednosti.
*
*/
void setDriverForward(int id)
{
  if (id == 1)
  {
    GPIO_ResetBits(GPIOA, GPIO_Pin_4);
    GPIO_SetBits(GPIOA, GPIO_Pin_10);
  }
  else if (id == 2)
  {
    GPIO_ResetBits(GPIOC, GPIO_Pin_9);
    GPIO_SetBits(GPIOC, GPIO_Pin_8);
  }
}

/**
*   @brief: Postavlja signale na drajeru motora tako da se on krece unazad.
*   @param: id predstavlja jedinstven identifikator motora.
*   @return: Nema povratnih vrednosti.
*
*/
void setDriverBackward(int id)
{
  if (id == 1)
  {
    GPIO_SetBits(GPIOA, GPIO_Pin_4);
    GPIO_ResetBits(GPIOA, GPIO_Pin_10);
  }
  else if (id == 2)
  {
    GPIO_SetBits(GPIOC, GPIO_Pin_9);
    GPIO_ResetBits(GPIOC, GPIO_Pin_8);
  }
}