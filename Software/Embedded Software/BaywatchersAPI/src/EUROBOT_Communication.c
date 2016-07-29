/**
*	@file: 	  EUROBOT_Communication.c
*	@author:  Vitez21 (koprivica.slobodan92@gmail.com), Praetorian
*	@version: v1.01.240216.0205 (Praetorian)
*	@date: 	  24/02/2016
*	@brief:	  Definicije funkcija potrebnih za komunikaciju robota.
*/

/** Includes. */
#include "EUROBOT_Communication.h"
#include "EUROBOT_Movement.h"
#include "stm32f10x_conf.h"

/** Konstante. */
#define MAX_RECEIVED_LENGTH 200	 // Maksimalna duzina poruke koju je moguce primiti.
#define ADDR 0x0A        // Adresa robota.

/** Promenljive za prijem poruke. */
unsigned int received_array[MAX_RECEIVED_LENGTH]; // Niz u koji se smesta primljena poruka.
unsigned int received_check_sum = 0;           // Check suma pri prijemu poruke.
int received_byte_counter = MAX_RECEIVED_LENGTH;	// Brojac primljenih bajtova poruke.

int received_message_size = -1;  // Duzina primljene poruke.

/** Promenljive za slanje poruke. */
int sending_array[256];		// Niz u koji se stavlja poruka za slanje.
int sending_length = 0;		// Duzina poruke za slanje.
int received_address = 0;       // Predstavlja primeljenu adresu koja definise za koji mikrokontroler je pristigla komanda.
int sending_iterator = 0;	// Brojac poslatih bajtova.
unsigned char received_byte;
unsigned int sending_check_sum = 0;	// Promenljiva za racunanje check sume koja se salje.

unsigned char command_ID = 0;  // Predstavlja ID poslednje uspesno izvrsene instrukcije.
float angularConstant = 0.00798226;	// PITANJE: Sta je ugaona konstanta, dobija se kao 180/broj impulsa za rotaciju...?


/**
*   @brief: Funkcija u koju se ulazi kada se primi novi bajt preko USART3.
*   @param: received_byte predstavlja primljen bajt.
*   @return: Nema povratnih vrednosti.
*
*/
void byteReceived(unsigned char received_byte)
{
  // Primljen bajt 0xFF koji oznacava pocetak nove poruke.
  if (received_byte == 0xFF)
  {  
    received_check_sum = 0;
    received_byte_counter = 0;
    received_message_size = MAX_RECEIVED_LENGTH;
  }
  else
  {          
    // Primljen prvi bajt poruke koji oznacava adresu uredjaja kojem se poruka salje.
    if (received_byte_counter == 0)
    {  
      received_address = received_byte;
      received_check_sum = received_byte;
    }
    // Primljen drugi bajt poruke koji oznacava duzinu poruke.
    else if (received_byte_counter == 1) 
	 {  
           received_message_size = received_byte;
           received_check_sum += received_byte;	// Uvecaj check sumu.
         }
         // Primljen poslednji bajt poruke koji predstavlja check sumu.
         else if (received_byte_counter == received_message_size+1)
	      {  
                // Provera da li primljena check suma odgovara izracunatoj check sumi.
                if ((received_check_sum & 0x7F) == received_byte)
	        {  
                // Ukoliko odgovara dekoduj primljenu poruku.
                DecodeCommand();  
                }
	      }
              // Primljeni ostali bajtovi poruke.
              else 
              {  
                received_array[received_byte_counter - 2] = received_byte;  
                received_check_sum += received_byte;
              }
    // Uvecaj brojac primljenih bajtova poruke pri prijemu novog bajta.
    if (received_byte_counter < MAX_RECEIVED_LENGTH) received_byte_counter++; 
  }
}

/**
*   @brief: Vrsi dekodovanje primljene komande.
*   @param: Nema ulaznih argumenata.
*   @return: Nema povratnih vrednosti.
*
*/
void DecodeCommand(void)
{
  char command;
  unsigned int command_sign;
  //Ako se primljena adresa poklapa sa adresom mikrokontrolera.
  if (received_address == ADDR)
  {
    switch (received_array[0]) 
    {
      case 0xF1: command = 'l'; command_sign =  1; break;  //Kretanje napred.
      case 0xF2: command = 'l'; command_sign = -1; break;  //Kretanje nazad.
      case 0xF3: command = 'r'; command_sign =  1; break;  //Rotacija desno.
      case 0xF4: command = 'r'; command_sign = -1; break;  //Rotacija levo.
      case 0xE0: command = 'a'; break;                     //Podesavanje uglovne konstante.
      case 0xE1: command = 'b'; break;                     //Setovanje pozicije robota(x, y, ugao).
      case 0xE2: command = 'c'; break;                     //Reset pozicije na nulu.
      case 0xE3: command = 'd'; break;                     //Emergency STOP.
      case 0xF7: command = 'v'; break;                     //Inspektorska, vraca status poruku (x, y, ugao, stigao).
      case 0xFB: command = 'z'; break;                     //Podesavanje preskalera za brzinu.
      default: command = 'v'; break;                       //Default komanda/
    };
    
    // Izvrasavanje naredbe.
    ExecuteCommand(command, command_sign);
  }
}

/**
*   @brief: Vrsi izvrsavanje primljene komande.
*   @param: command oznacava koja je komanda u pitanju.
*   @param: command_sign oznacava smer komande u pojedinim komandama.
*   @return: Nema povratnih vrednosti.
*
*/
void ExecuteCommand(char command, unsigned int command_sign)
{
  int temp_data;
  int temp_ID;
  switch (command)
  {
    // Kretanje napred ili nazad.
    case 'l':
             // Parsiranje zadatog pomeraja translacije u milimetrima.
             temp_data = received_array[1] | received_array[2]<<4 | received_array[3]<<8 | received_array[4]<<12;        
             // Parsiranje ID-a komande.
             temp_ID = received_array[5] | received_array[6]<<4;    
    
             // Provera da li je primljen ID razlicit od ID-a komande.
             /* Ukoliko se desi da posle poslate poruke za npr. kretanje unapred(ili od strane racunara) robot ne vrati ACK poruku, 
             master ce ponovo poslati istu komandu. Medjutim, kako je zeljena vrednost pomeraja vec jednom uvecana,
             pri prijemu nove  iste komande bi bila ponovo jos jednom uvecana, cime bi robot otisao unapred za dva puta veci pomeraj.
             Da se ovo ne bi dogadjalo uvodi se ID komande koji uvecava zeljenu vrednost samo jednom.
             Isto vazi i za rotaciju i kretanje unazad.
             */
             if (temp_ID != command_ID)
             {
               
               // Zadavanje komande za kretanje napred.
               if (command_sign == 1) moveForewardCmd(temp_data);
			
               // Zadavanje komande za kretanje nazad.
               else if (command_sign == -1) moveBackwardCmd(temp_data);
             }
             
             command_ID = temp_ID;
             
             // Slanje potvrde glavnom procesoru da je instrukcija izvrsena.
             SendAck();
             break;
             
    // Rotiranje oko svoje ose.       
    case 'r':
             // Parsiranje zadatog pomeraja rotacije u stepenima.	
             temp_data = command_sign * (received_array[1] | received_array[2]<<4 | received_array[3]<<8 | received_array[4]<<12);      
             // Parsiranje ID-a komande
             temp_ID = (received_array[5] | received_array[6]<<4);
             
             // Provera da li je primljen ID razlicit od ID-a poslednje uspesno izvrsene komande.
             if (temp_ID != command_ID)	
               // Zadavanje komande za rotaciju.
               rotateCmd(command_sign * temp_data);
             
             command_ID = temp_ID;
             
             // Slanje potvrde glavnom procesoru da je instrukcija izvrsena.
             SendAck();
             break;
  }
  
  /**
	// Podesavanje uglovne konstante
	else if (komanda == 'a') 
	{
		//Parsiranje vrednosti uglovne konstante 
		x = (received_array[1] | received_array[2]<<4 | received_array[3]<<8 | received_array[4]<<12);  
		//PITANJE: Sta je angularConstant i da li se sa ovim deli?
		angularConstant = 180 / ((float)temp_x);
		SendACK();        
	}
	// Setovanje pozicije robota(x, y, ugao)
	else if (komanda == 'b') 
	{        
		// PITANJE: Kako smo mi nazvali abs_X, absY, abs_Theta
		abs_X = (received_array[1] | received_array[2]<<4 | received_array[3]<<8 | received_array[4]<<12 | received_array[5]<<16 | received_array[6]<<20 | received_array[7]<<24 | received_array[8]<<28);
		abs_Y = (received_array[9] | received_array[10]<<4 | received_array[11]<<8 | received_array[12]<<12 | received_array[13]<<16 | received_array[14]<<20 | received_array[15]<<24 | received_array[16]<<28);
		abs_Theta = (received_array[17] | received_array[18]<<4 | received_array[19]<<8 | received_array[20]<<12);
		SendACK();
	}
	// Reset pozicije na nulu
	else if (komanda == 'c')
	{        
		abs_X=0;
		abs_Y=0;
		abs_Theta=0;     
		SendACK();
	}
	// Emergency STOP
	else if (komanda == 'd') 
	{
		// POZOVI FUNKCIJU
		SendACK();
	}
	// Inspektorska, vraca status poruku (x, y, ugao, stigao)
	else if (komanda == 'v') 
	{
		SendPosition();
	}
	// Podesavanje preskalera za brzinu
	else if (komanda == 'z') 
	{
		// Parsiranje vrednosti preslakelera
		temp_x = (received_array[1] | received_array[2]<<4 | received_array[3]<<8 | received_array[4]<<12);  
		TIM4->PSC = temp_x;
		TIM7->PSC = temp_x;
		SendACK();        
	}
  */
}

/**
*   @brief: Slanje ACK signala masteru.
*   @param: Nema ulaznih parametara.
*   @return: Nema povratnih vrednosti.
*
*/
void SendAck(void)
{
  sending_length = 4;
  sending_array[0] = 0xFF;
  sending_array[1] = ADDR | 0x40;
  sending_array[2] = sending_length - 3;        
  sending_check_sum = 0;
  for (int i = 1;i < sending_length - 1; i++)
  {
    sending_check_sum += sending_array[i] & 0xFF;
  }
  // Salje se check suma kao poslednji bajt poruke, pri tom se vodi racuna(pomocu & 0x7F) da check suma ne sme biti 0xFF
  // (bajt za start nove poruke).
  sending_array[3]= sending_check_sum & 0x7F;

  /* Proveriti sta je ovo. */
  // Enable-uje se RS485 predaja
  //GPIO_SetBits(GPIOC, GPIO_Pin_12); 
  
  // Salje prvi podatak, prekidna rutina USART3 salje celu poruku	
  USART_SendData(USART3, sending_array[0]);	
  sending_iterator = 1;
}

/**
*   @brief: Slanje pozicije u formatu (x, y, ugao, stigao).
*   @param: Nema ulaznih parametara.
*   @return: Nema povratnih vrednosti.
*
*/
void SendPosition(void)
{ 

        AbsolutePosition currentPosition = getCurrentPosition();
	int robot_in_position = 0;

	sending_length = 25;
	sending_array[0] = 0xFF;
	sending_array[1] = ADDR|0x40;
	sending_array[2] = sending_length-3;
	//Slanje X koordinate
	sending_array[3] = (currentPosition.x & 0x0000000F);
	sending_array[4] = (currentPosition.x & 0x000000F0)>>4;
	sending_array[5] = (currentPosition.x & 0x00000F00)>>8;
	sending_array[6] = (currentPosition.x & 0x0000F000)>>12;
	sending_array[7] = (currentPosition.x & 0x000F0000)>>16;
	sending_array[8] = (currentPosition.x & 0x00F00000)>>20;
	sending_array[9] = (currentPosition.x & 0x0F000000)>>24;
	sending_array[10] = (currentPosition.x & 0xF0000000)>>28;
	//Slanje Y koordinate
	sending_array[11] = (currentPosition.y & 0x0000000F);
	sending_array[12] = (currentPosition.y & 0x000000F0)>>4;
	sending_array[13] = (currentPosition.y & 0x00000F00)>>8;
	sending_array[14] = (currentPosition.y & 0x0000F000)>>12;
	sending_array[15] = (currentPosition.y & 0x000F0000)>>16;
	sending_array[16] = (currentPosition.y & 0x00F00000)>>20;
	sending_array[17] = (currentPosition.y & 0x0F000000)>>24;
	sending_array[18] = (currentPosition.y & 0xF0000000)>>28;
	//Slanje ugla
	sending_array[19] = (currentPosition.angle & 0x000F);
	sending_array[20] = (currentPosition.angle & 0x00F0)>>4;
	sending_array[21] = (currentPosition.angle & 0x0F00)>>8;
	sending_array[22] = (currentPosition.angle & 0xF000)>>12;
	//PITANJE: Prilagoditi zakomentarisano nasim potrebama
	/*
	if ((ENC1<zadata_pozicija_X+INP_TOLERANCE)&&(ENC1>zadata_pozicija_X-INP_TOLERANCE)) x_inp=1;
	if ((ENC2<zadata_pozicija_Y+INP_TOLERANCE)&&(ENC2>zadata_pozicija_Y-INP_TOLERANCE)) y_inp=1;
	if ((x_inp==1)&&(y_inp==1)) {
	  xy_inp=1;
	}
	*/
	//Slanje da li je robot stigao u poziciju
	sending_array[23]= robot_in_position ? 0x71 : 0x70;
	//Racunanje i slanje check sume
	sending_check_sum = 0;
	for (int i = 1; i < sending_length - 1; i++){
	  sending_check_sum += sending_array[i] & 0xFF;
	}
	sending_array[sending_length-1]= sending_check_sum & 0x7F;
	
	// Enable-uje se RS485 predaja
	GPIO_SetBits(GPIOC,GPIO_Pin_12); 
	// Salje prvi podatak, prekidna rutina USART3 salje celu poruku	
	USART_SendData(USART3,  sending_array[0]);
	sending_iterator=1;
}
	
	
	
	
	
	
	
	
	
	
	
	
	


