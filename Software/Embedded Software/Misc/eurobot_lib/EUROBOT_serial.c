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

#include <string.h>
#include <stdlib.h>

#include "EUROBOT_serial.h"
#include "EUROBOT_Init.h"

// Bajt koji oznacava pocetak poruke
#define START_BYTE 0xFF

// Najvisi bit ne ulazi u check sumu da se ne bi desila situacija da je ovaj bajt
// jednak bajtu koji oznacava pocetak poruke, pa da samim tim ne dodje do greske
// u prijemu poruke.
#define CHECHSUM_MASK 0x7F

// Maksimalna duzina podatka koji se salje
#define MAX_DATA_LENGTH 0xFE // Ne sme biti 0xFF da se ne bi pomesalo sa pocetkom poruke
              
#define VER2

/**************************************************************************************/
// Strukture za slanje i prijem poruke
typedef struct {
  uint8_t data[MAX_DATA_LENGTH];        // Podatak koji se prima
  char prev_data[MAX_DATA_LENGTH];      // Poslednji validan podatak
  uint8_t address;                      // Adresa gde se šalje
  char prev_address;                    // Adresa poslednjeg validnog podatka
  int message_length;                   // Dužina poruke
  int prev_length;
  uint8_t check_sum;                    // Chech suma - potvrda validnosti poruke
  unsigned int iter;                    // Iterator za prenos podataka
} data_package;

data_package received;


typedef struct {
  uint8_t *data;        // Podatak koji se šalje
  int message_length;                   // Dužina poruke
  unsigned int iter;                    // Iterator za prenos podataka
} send_package;

#ifndef VER2
send_package sending;
#endif
/**************************************************************************************/

// Poslednji primljeni bajt
uint8_t received_byte;

/**************************************************************************************/
// Masina stanja za prijem poruke
typedef enum{ 
  START,
  ADDRESS,
  LENGTH,
  MESSAGE,
  CHECK
} ReceivedStateType;

// Trenutno stanje
ReceivedStateType message_state = START;

/**************************************************************************************/
// Struktura koja sadrzi informacije o uredjaju
typedef struct {
  uint8_t master_addr;        // Adresa mastera na magistrali
  uint8_t this_addr;          // Adresa ovog uredjaja
  int is_master;              // Da li je uredjaj master na magistrali
} DeviceType;

DeviceType ThisDevice;

/**************************************************************************************/
// Deklaracije za red
int is_emptyQ();
void insertQ(send_package* data);
void delete_lastQ();
send_package* peekQ();

/**************************************************************************************/

/**
*   @brief: Pomocna funkcija koja niz pristiglih bajtova konvertuje u validnu poruku
*   @param: String u koji se smesta poruka dobijena iz niza pristiglih bajtova
*   @param: String koji predstavlja niz pristiglih bajtova
*   @return: Nema povratnu vrednost
*
*/
void ExtractMessage(char* dest, char* src){
    unsigned int length = received.message_length;
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
*   @return: Nema povratnu vrednost
*
*/
void ProcessByte(uint8_t received_byte){
  
  // Masina stanja za prijem poruke
  switch(message_state){
    
    // Nije u toku prenos poruke, ceka se start bajt
    case START:
      if(received_byte == START_BYTE){
          message_state = ADDRESS;
      }
      else{
          message_state = START;
      }
      break;
      
    // Ucitani bajt je adresa, osim ako nije start bajt
    case ADDRESS:
      received.data[0] = 0x00;
      received.check_sum = 0;
      received.iter      = 0;
      received.message_length = 0;

      if(received_byte == START_BYTE){
          message_state = ADDRESS;
      }
      else{
          received.address = received_byte;
          received.check_sum = received_byte;
          message_state = LENGTH;
      }
      break;
      
    // Sledeci bajt je duzina poruke
    case LENGTH:      
      if(received_byte == START_BYTE){
          message_state = ADDRESS;
      }
      else{
          received.message_length = (int)received_byte;
          received.check_sum += received_byte;
          
          if(received.message_length == 0){
            message_state = CHECK;
          }
          else{
            message_state = MESSAGE;
          }
      }
      break;
      
    // Bajtovi poruke
    case MESSAGE:
      if(received_byte == START_BYTE){
          message_state = ADDRESS;
      }
      else{
          received.data[received.iter] = received_byte;   
          if(received.iter <= MAX_DATA_LENGTH) received.data[received.iter + 1] = '\0';  
          received.check_sum += received_byte;
          received.iter++;
          
          if(received.iter == received.message_length){
              message_state = CHECK;
          }
          else{
              message_state = MESSAGE;
          }
      }
      break;
      
    // Kraj poruke, provera check sume
    case CHECK:
      if(received_byte == START_BYTE){
          message_state = ADDRESS;
      }
      else{
        if ((received.check_sum & CHECHSUM_MASK) == received_byte){              
            ExtractMessage(received.prev_data,(char*)received.data);          
            received.prev_length = received.message_length - (received.message_length/8 + (received.message_length % 8 != 0));
            received.prev_address = (char) received.address;
            
        // Ako je ovo SLAVE vrati ACK            
        if(!ThisDevice.is_master) {
          if(!IsAck() && ThisDevice.this_addr == received.address) SendACK(ThisDevice.master_addr);
        }

            DecodeCommand(); 
        }
        message_state = START;
      }
      break;
  
  }
}

/**
*   @brief: Prekidna rutina za USART3
*   @param: Nema
*   @return: Nema
*
*/
void USART3_IRQHandler(){   
    if ((USART_GetITStatus(RS485, USART_IT_RXNE) != RESET))
    {
            USART_ClearITPendingBit(RS485, USART_IT_RXNE);
                
            // Prijem poruke
            received_byte = (char)USART_ReceiveData(RS485);               
            ProcessByte(received_byte);
    }
    else if (USART_GetITStatus(RS485, USART_IT_TC) != RESET)
    {
            USART_ClearITPendingBit(RS485, USART_IT_TC);
            
            // Slanje poruke            
            if (peekQ()->iter < peekQ()->message_length && !is_emptyQ())
            {
                    USART_SendData(RS485,  peekQ()->data[peekQ()->iter]);
                    peekQ()->iter++;
                    if(peekQ()->iter == 2){
                      int dummy = 2;
                    }
            }
            else
            {       
                    delete_lastQ();
                    if(!is_emptyQ())
                    {
                        USART_SendData(RS485,  peekQ()->data[peekQ()->iter]);
                    }
                    else
                    {
                        delete_lastQ();
                        DisableRS485();
                    }
            }          

    }
    else{
            // Sprecava overrun ili underrun gresku, proveriti
            USART_ITConfig(RS485, USART_IT_TC, DISABLE);
            received_byte = USART_ReceiveData(RS485); // ?
            USART_ReceiveData(RS485);
            RS485->SR = 0x00000000;	// D8
            USART_ClearITPendingBit(RS485, USART_IT_TXE);
            USART_ITConfig(RS485, USART_IT_TC, ENABLE);
    }
}

/**
*   @brief: Slanje stringa preko USART/RS485
*   @param: Adresa uredjaja na koju se salju podaci
*   @param: Poruka koja se salje na uredjaj. Potrebno je da bude u formatu stringa
*           tj da se iza kraja poruke postavi znak '\0' odnosno 0x0
*   @return: Nema povratnu vrednost
*
*/
void SendString(unsigned char address, unsigned char* message){
    SendMessage(address,message,strlen(message));
}

/**
*   @brief: Slanje poruke preko USART/RS485
*   @param: Adresa uredjaja na koju se salju podaci
*   @param: Poruka koja se salje na uredjaj u obliku niza bajtova
*   @param: Duzina poruke u bajtovima
*   @return: Nema povratnu vrednost
*
*/
void SendMessage(unsigned char address, unsigned char* message, char length){
  
  send_package* sending = (send_package*)malloc(sizeof(send_package));
  
  //sending->message_length = length + 4;    // Ukupna duzina poruke  12 + 1 + 1 + 4
  sending->message_length = 4 + length + (length/7) + (length % 7 != 0);
  sending->data = (uint8_t*)malloc((sending->message_length + 1) * sizeof(uint8_t));
  sending->iter = 0;    
  char check_sum = 0;
  
  // Cuvanje Start bajta, adrese na koju se salje poruka
  sending->data[0] = START_BYTE;
  sending->data[1] = address;
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
    //    sending->message_length++;
    } 

    // azuriranje high_bits bajta i maskiranje najviseg bita kod ostalih bajtova
    high_bits |= ((~CHECHSUM_MASK) & message[i]) ? (1 << bit) : 0 ;
    sending->data[j] = message[i] & CHECHSUM_MASK;
    
    // Azuriranje check sume
    check_sum += sending->data[j];

    // Ako smo stigli do kraja jedne sekvence od 7 bajtova, sacuvati high_bits u
    // niz koji se prosledjuje, i poceti ispocetka.
    if (bit == 6){
        // Izbegavamo slanje 0x00, menjamo sa 0x80
        high_bits = (!high_bits) ? 0x80 : high_bits;
        sending->data[j - 7] =  high_bits;        
        //Azuriranje check sume
        check_sum += high_bits;
        high_bits = 0;     
    }
    
    j++;
    bit++;
  }
  // Ovo se izvrsava ako broj bajtova koji se prosledjuje nije deljiv sa 7
  if(bit != 7) {
    // Izbegavamo slanje 0x00, menjamo sa 0x80
    high_bits = (!high_bits) ? 0x80 : high_bits;
    sending->data[j - bit - 1] =  high_bits;
    //Azuriranje check sume
    check_sum += high_bits;
  }
      
  // Cuvanje duzine poruke
  sending->data[2] = sending->message_length - 4;
  // Dodavanje adrese i duzine poruke na check sumu
  check_sum += sending->data[1] + sending->data[2];
  // Uklanjanje najviseg bita sa check sume
  check_sum = check_sum & CHECHSUM_MASK;

  // Cuvanje check sume u niz koji se prosledjuje 
  sending->data[sending->message_length - 1] = check_sum;
  
  // Enable-uje se RS485 predaja
  EnableRS485();
  
  // Prvi clan niza se salje na USART, prekidna rutina salje ostatak poruke
  if(is_emptyQ()){
    insertQ(sending);
    sending->iter = 1;
    USART_SendData(RS485,  sending->data[0]);
  }
  else {
    insertQ(sending);
  }
}

/**
*   @brief: Slanje poruke kojom se signalizira uspesan prijem poruke
*   @param: Adresa uredjaja na koji se ovaj signal salje
*   @return: Nema povratnu vrednost
*
*/
void SendACK(unsigned char address){
    SendString(address, "");
}

/**
*   @brief: Preuzimanje poslednje validne poruke koja je stigla
*   @param: Nema
*   @return: String koji predstavlja koristan deo pristigle poruke 
*
*/
char* GetMessage(){
    return (char*) received.prev_data;
}

/**
*   @brief: Preuzimanje adrese na koju treba da stigne poslednja validna poruka
*   @param: Nema
*   @return: Adresa na koju je adresiran poslednji validni podatak
*
*/
char GetAddress(){
    return (char) received.prev_address;
}

/**
*   @brief: Dohvatanje duzine posledenje validne poruke
*   @param: Nema
*   @return: Duzina poslednje validne poruke
*
*/
char GetMessageLength(){
    return received.prev_length;
}

/**
*   @brief: Proveravanje da li je poslednja validna poruka ACK
*   @param: Nema
*   @return: Informacija da li je ACK poslednja poruka
*
*/
int IsAck(){
   if(GetMessage() == "" && GetMessageLength() == 0) return 1;
   return 0;
}

/**
*   @brief: Iniciranje RS485 komunikacije
*   @param: Baud Rate transfera
*   @param: Adresa mastera na magistrali
*   @param: Adresa ovog uredjaja
*   @param: Da li je ovaj uredjaj master na magistrali
*   @return: Nema
*
*/
void initEurobotRS485(uint32_t BaudRate, uint8_t master_addr, uint8_t this_addr, int is_master){
    
    ThisDevice.master_addr = master_addr;
    ThisDevice.this_addr = this_addr;
    ThisDevice.is_master = is_master;
    
    InitGPIOPin(GPIOC, GPIO_Pin_10, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);         // Linija Tx za USART3
    InitGPIOPin(GPIOC, GPIO_Pin_11, GPIO_Mode_IN_FLOATING, GPIO_Speed_50MHz);   // Linija Rx za USART3
    InitGPIOPin(GPIOC, GPIO_Pin_12, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);        // Linija Te za RS485
       
    GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE); // Remapirati Rx: PB10 -> PC10 i Tx: PB11 -> PC11
    
    // Inicijalizacija USART3
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    
    InitDefaultUSART(USART3, USART_Mode_Rx | USART_Mode_Tx, BaudRate);
    USART_ITConfig(USART3, USART_IT_TC, ENABLE);
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    
    // Da li oba ova trebaju?
    NVIC_EnableIRQ(USART3_IRQn);
    InitNVICChannel(USART3_IRQn, 0, 0, ENABLE);
}

/**
*   @brief: Pocetak RS485 komunikacije
*   @param: Nema
*   @return: Nema
*
*/
void EnableRS485(){
  GPIO_SetBits(GPIOC,GPIO_Pin_12);
}

/**
*   @brief: Kraj RS485 komunikacije
*   @param: Nema
*   @return: Nema
*
*/
void DisableRS485(){
  GPIO_ResetBits(GPIOC,GPIO_Pin_12);
}


/**************************************************************************************/
/*****************************  RED PORUKAMA ******************************************/
/**************************************************************************************/

typedef struct node {
  send_package *data;
  struct node *next;
} list;

list *first = 0, *last = 0;

// Provera da li je red prazan
int is_emptyQ(){
  if(first == 0 && last == 0) return 1;
  return 0;
}

// Ubacivanje elementa u red
void insertQ(send_package* data){
  list *tmp = (list*)malloc(sizeof(list));
  tmp->next = 0;
  tmp->data = data;
  
  if(is_emptyQ()){
    first = last = tmp;
    return;
  }
  
  last->next = tmp;
  last = last->next;
}

// Brise prvu poruku u redu
void delete_lastQ(){
  if(is_emptyQ()) return;
 
  if(first == last) last = 0;
  list *tmp = first;
  first = first->next;
  
  free(tmp->data->data);
  free(tmp->data);
  free(tmp);
  
}

// Procitati prvu poruku u redu
send_package* peekQ(){
  if(is_emptyQ()) return 0;  
  return first->data;
}
