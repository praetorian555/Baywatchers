/**
*	@file: 	  EUROBOT_serial.c
*	@author:  Jovan Blanusa, Vitez21 (koprivica.slobodan92@gmail.com) starije verzije
*	@version: ?
*	@date: 	  23.1.2016
*	@brief:	  API funkcije koje olaksavaju komunikaciju pojedinih STM-ova u
*                 robotu. Ideja je da korisnik ovih funkcija ne misli o formatu
*                 poruke i eventualnim greskama do kojih bi moglo doci prilikom
*                 slanja poruke. Da bi ispravno radila biblioteka potrebno je 
*                 uraditi dve stvari: 
*                    - Pozvati makro PREPARE_USART(no) za odgovarajuci kanal
*                         da bi se definisala odgovarajuca prekidna rutina
*                    - Definisati funkciju DecodeCommand() koja se obavlja u 
*                         prekidnoj rutini posto se primi cela poruka.
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

#include <string.h>
#include "EUROBOT_serial.h"


// Bajt koji oznacava pocetak poruke
#define START_BYTE 0xFF

// Najvisi bit ne ulazi u check sumu da se ne bi desila situacija da je ovaj bajt
// jednak bajtu koji oznacava pocetak poruke, pa da samim tim ne dodje do greske
// u prijemu poruke.
#define CHECHSUM_MASK 0x7F

// Maksimalna duzina podatka koji se salje
#define MAX_DATA_LENGTH 0xFE // Ne sme biti 0xFF da se ne bi pomesalo sa pocetkom poruke

// Struktura koja sadrzi podatke potrebne za USART komunikaciju
typedef struct {
  uint8_t data[MAX_DATA_LENGTH];        // Podatak koji se šalje
  char prev_data[MAX_DATA_LENGTH];      // Poslednji validan podatak
  uint8_t address;                      // Adresa gde se šalje
  char prev_address;                    // Adresa poslednjeg validnog podatka
  int message_length;                   // Dužina poruke
  uint8_t check_sum;                    // Chech suma - potvrda validnosti poruke
  unsigned int iter;                    // Iterator za prenos podataka
  int transaction_on;                   // Da li je prenos u toku
} data_package;

// Poslednji primljeni bajt
uint8_t received_byte;

// prev_data i prev_address sluze da bi se validan podatak mogao dohvatiti cak i 
// u slucaju da je trenutno prenos podataka u toku.


// Po 3 strukture za slanje i prijem
data_package sending[3];
data_package received[3];
USART_TypeDef* USARTa[3] = {USART1, USART2, USART3};

/**
*   @brief: Pomocna funkcija koja niz pristiglih bajtova konvertuje u validnu poruku
*   @param: String u koji se smesta poruka dobijena iz niza pristiglih bajtova
*   @param: String koji predstavlja niz pristiglih bajtova
*   @return: Nema povratnu vrednost
*
*/
void ExtractMessage(char* dest, char* src){
    unsigned int length = strlen(src);
    uint8_t high_bits = 0;
    //iterator za modifikovani niz, 3 pocetna vrednost, jer su prva 3 podatka vec postavljena u poruku
    int j = 0;
    // trenutni bajt koji se modifikuje u high_bits bajtu
    char bit;
    for(int i = 0; i < length; i++){
      if(i % 8 == 0){
        high_bits = src[i];
      } 
      else{
        bit = i % 8 - 1;
        dest[j] = src[i] | ( ((high_bits & ( 1<<bit )) ? 1 : 0)<<7 );
        j++;
      }     
    }
    dest[j] = 0x00;
}

/**
*   @brief: Obradjuje poslednji pristigli bajt preko USART-a. Po zavrsetku prijema
*           poruke izvrsava se funkcija DecodeCommand() koju je potrebno da korisnik
*           definise. Poruku koja je pristigla kao i adresu na koju je adresirana
*           poruka moguce je preuzeti pomocu funkcija: GetMessage(unsigned int)
*           i GetAddress(unsigned int usartNo). Potrebno je obratiti paznju da se
*           funkcija DecodeCommand() izvrsava u prekidnoj rutini.
*   @param: Prethodno pristigli bajt koji je potrebno da se obradi
*   @param: Broj USART kanala sa kog je pristigao bajt
*   @return: Nema povratnu vrednost
*
*/
void ProcessByte(uint8_t received_byte, unsigned int No)
{
  // Primljen bajt 0xFF koji oznacava pocetak nove poruke.
  if (received_byte == START_BYTE && received[No].transaction_on == 0)
  {  
    received[No].check_sum = 0;
    received[No].iter      = 0;
    received[No].message_length = MAX_DATA_LENGTH;
    received[No].transaction_on = 1;
  }
  else
  {          
    // Primljen prvi bajt poruke koji oznacava adresu uredjaja kojem se poruka salje.
    if (received[No].iter == 0)
    {  
      received[No].address = received_byte;
      received[No].check_sum = received_byte;
    }
    // Primljen drugi bajt poruke koji oznacava duzinu poruke.
    else if (received[No].iter == 1) 
	 {  
           received[No].message_length = (int)received_byte;
           received[No].check_sum += received_byte;	// Uvecaj check sumu.
         }
         // Primljen poslednji bajt poruke koji predstavlja check sumu.
         else if (received[No].iter == received[No].message_length + 2)
	      {  
                // Provera da li primljena check suma odgovara izracunatoj check sumi.
                if ((received[No].check_sum & CHECHSUM_MASK) == received_byte)
	        {  
                    received[No].data[received[No].iter - 2] = '\0'; 
                    ExtractMessage(received[No].prev_data,(char*)received[No].data);
                    received[No].data[0] = 0x00;
                    received[No].prev_address = (char) received[No].address;
                    received[No].transaction_on = 0;
                    // Poslati ACK?
                    
                    // Ukoliko odgovara dekoduj primljenu poruku.
                    DecodeCommand(No+1); 
                }
                // Sta se radi ako poruka nije ispravno poslata?
	      }
              // Primljeni ostali bajtovi poruke.
              else 
              { 
                received[No].data[received[No].iter - 2] = received_byte;                 
                received[No].check_sum += received_byte;
              }
    // Uvecaj brojac primljenih bajtova poruke pri prijemu novog bajta.
    if (received[No].iter < MAX_DATA_LENGTH) received[No].iter++; 
  }
}


/**
*   @brief: Interna funkcija koja se poziva u prekidnoj rutini za odgovarajuci 
*           kanal. Prekidna rutina se generise makroom PREPARE_USART(no)
*   @param: Broj USART kanala za koji se pravi funkcija koja se obavlja u 
*           prekidnoj rutini
*   @return: Nema povratnu vrednost
*
*/
inline void usart_interrupt(unsigned int IRQNo){
    IRQNo--;
    USART_TypeDef* USARTx = USARTa[IRQNo];
    
    if ((USART_GetITStatus(USARTx, USART_IT_RXNE) != RESET))
    {
            USART_ClearITPendingBit(USARTx, USART_IT_RXNE);
                
            // Prijem poruke
            received_byte = (char)USART_ReceiveData(USARTx);               
            ProcessByte(received_byte, IRQNo);
    }
    else if (USART_GetITStatus(USARTx, USART_IT_TC) != RESET)
    {
            USART_ClearITPendingBit(USARTx, USART_IT_TC);
                
            // Slanje poruke            
            if (sending[IRQNo].iter < sending[IRQNo].message_length)
            {
                    USART_SendData(USARTx,  sending[IRQNo].data[sending[IRQNo].iter]);
                    sending[IRQNo].iter++;
            }
            else
            {
                    sending[IRQNo].iter = 0;
                    //GPIO_ResetBits(GPIOC,GPIO_Pin_12);	// Disabluje RS485
            }
    }
    else{
            // Sprecava overrun ili underrun gresku, proveriti
            USART_ITConfig(USARTx, USART_IT_TC, DISABLE);
            received_byte = USART_ReceiveData(USARTx); // ?
            USART_ReceiveData(USARTx);
            USARTx->SR = 0x00000000;	// D8
            USART_ClearITPendingBit(USARTx, USART_IT_TXE);
            USART_ITConfig(USARTx, USART_IT_TC, ENABLE);
    }
}

/**
*   @brief: Slanje poruke preko USART/RS485
*   @param: Adresa uredjaja na koju se salju podaci
*   @param: Poruka koja se salje na uredjaj. Potrebno je da bude u formatu stringa
*           tj da se iza kraja poruke postavi znak '\0' odnosno 0x0
*   @param: Broj USART kanala preko kog se salje poruka
*   @return: Nema povratnu vrednost
*
*/
void SendMessage(unsigned char address, unsigned char* message, unsigned int usartNo){
  usartNo--;
  
  unsigned int length = strlen(message);           // Duzina korisnog dela poruke
  sending[usartNo].address = address;                   
  sending[usartNo].message_length = length + 4;    // Ukupna duzina poruke      
  sending[usartNo].iter = 0;
  sending[usartNo].check_sum = 0;
  
  // Cuvanje Start bajta, adrese na koju se salje poruka
  sending[usartNo].data[0] = START_BYTE;
  sending[usartNo].data[1] = address;
  // Duzina poruke na sending[usartNo].data[2]
  
  // Posto najvisi bit svakog bajta u poruci mora biti 0 da ne bi greskom doslo
  // do start bita, poruka se deli na grupe po 7 bajtova. Prvo se salju najvisi 
  // biti svakog od tih 7 bajtova, potom se salju ostali biti svakog bajta
  // Primer:
  //             Poruka:     F0 F1 F2 F3 F4 F5 F6
  //     Prosledjuje se:  7F 70 71 72 73 74 75 76
  
  // bajt podataka sa najvisim bitima narednih 7 bajtova
  uint8_t high_bits = 0;
  //iterator za modifikovani niz, 3 pocetna vrednost, jer su prva 3 podatka vec postavljena u poruku
  int j = 3;
  // trenutni bajt koji se modifikuje u high_bits bajtu
  char bit = 7;
  for(int i = 0; i < length; i++){
    
    // Ako sledi pocetak segmenta od 7 bajtova ostaviti prazno mesto za bajt sa visim bitima
    if(bit == 7){
        j++;
        bit = 0;
        sending[usartNo].message_length++;
    } 

    // azuriranje high_bits bajta i maskiranje najviseg bita kod ostalih bajtova
    high_bits |= ((~CHECHSUM_MASK) & message[i]) ? (1 << bit) : 0 ;
    sending[usartNo].data[j] = message[i] & CHECHSUM_MASK;
    
    // Azuriranje check sume
    sending[usartNo].check_sum += sending[usartNo].data[j];

    // Ako smo stigli do kraja jedne sekvence od 7 bajtova, sacuvati high_bits u
    // niz koji se prosledjuje, i poceti ispocetka.
    if (bit == 6){
        high_bits = (!high_bits) ? 0x80 : high_bits;
        sending[usartNo].data[j - 7] =  high_bits;        
        //Azuriranje check sume
        sending[usartNo].check_sum += high_bits;
        high_bits = 0;     
    }
    
    j++;
    bit++;
  }
  // Ovo se izvrsava ako broj bajtova koji se prosledjuje nije deljiv sa 7
  if(bit != 7) {
    sending[usartNo].data[j - bit - 1] =  high_bits;
    sending[usartNo].check_sum += high_bits;
  }
      
  // Cuvanje duzine poruke
  sending[usartNo].data[2] = sending[usartNo].message_length - 4;
  // Dodavanje adrese i duzine poruke na check sumu
  sending[usartNo].check_sum += sending[usartNo].data[1] + sending[usartNo].data[2];
  // Uklanjanje najviseg bita sa check sume
  sending[usartNo].check_sum = sending[usartNo].check_sum & CHECHSUM_MASK;

  // Cuvanje check sume u niz koji se prosledjuje 
  sending[usartNo].data[sending[usartNo].message_length - 1] = sending[usartNo].check_sum;
  
  // Enable-uje se RS485 predaja
  //GPIO_SetBits(GPIOC,GPIO_Pin_12); 
  
  // Prvi clan niza se salje na USART, prekidna rutina salje ostatak poruke
  USART_SendData(USARTa[usartNo],  sending[usartNo].data[0]);
}

/**
*   @brief: Slanje poruke kojom se signalizira uspesan prijem poruke
*   @param: Adresa uredjaja na koji se ovaj signal salje
*   @param: Broj USART kanala preko kog se salje poruka
*   @return: Nema povratnu vrednost
*
*/
void SendACK(unsigned char address, unsigned int usartNo){
    SendMessage(address, "", usartNo);
}


/**
*   @brief: Preuzimanje poslednje validne poruke koja je stigla
*   @param: Broj USART kanala sa koje preuzimamo poslednju validnu poruku
*   @return: String koji predstavlja koristan deo pristigle poruke 
*
*/
char* GetMessage(unsigned int usartNo){
    return (char*) received[usartNo-1].prev_data;
}


/**
*   @brief: Preuzimanje adrese na koju treba da stigne poslednja validna poruka
*   @param: Broj USART kanala sa koje preuzimamo adresu
*   @return: Adresa na koju je adresiran poslednji validni podatak
*
*/
char GetAddress(unsigned int usartNo){
    return (char) received[usartNo-1].prev_address;
}















