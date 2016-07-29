/* Includes ------------------------------------------------------------------*/

#include "stm32f10x_conf.h"
#include "EUROBOT_Init.h"
#include "Communication.h"

/* Private define ------------------------------------------------------------*/

#define MAX_TRANS_SIZE 255
#define MOTION_DEVICE_ADDRESS 0x0A
#define LENGTH_CONST 120.48
#define ANGLE_CONST 16.05

/* Private variables ---------------------------------------------------------*/

char sending_array[ MAX_TRANS_SIZE ];
int temp_address;
int sending_iterator = 0;
int sending_length = 0;
char receive_array[ MAX_TRANS_SIZE ];
bool FLAG_ackReceived = FALSE;
bool FLAG_arriveOnDest = FALSE;
uint16_t command_ID = 1;

/* Private function prototypes -----------------------------------------------*/


/**
  * @brief  Slanje komande zeljenom uredjaju.
  * @param  command predstavlja kod naredbe.
  * @param  receiver_address predstavlja adresu uredjaja kojem se salje poruka.
  * @param  data predstavlja podatak koji se salje u sklopu komande.
  */
void issueCommand( CommandNameType command, int receiver_address, uint16_t data )
{
  temp_address = receiver_address;
  sending_array[ 0 ] = 0xFF;
  sending_array[ 1 ] = temp_address;
  
  switch( command )
  {
    case STOP:
      issueSimpleCommand( 0x0A );
      break;
    case CHECK_ARRIVE:
      issueSimpleCommand( 0xFC );
      break;
    case ULTRASOUND_ON:
      issueSimpleCommand( 0x11 );
      break;
    case ULTRASOUND_OFF:
      issueSimpleCommand( 0x12 );
      break;
    case PRESCALER:
      issueComplexCommand( 0xFB, data );
      break;
    case MOVE_FORWARD:
      issueComplexCommand( 0x04, (uint16_t)(data * LENGTH_CONST) );
      break;
    case MOVE_BACKWARD:
      issueComplexCommand( 0x05, (uint16_t)(data * LENGTH_CONST) );
      break;
    case ROTATE_RIGHT:
      issueComplexCommand( 0x06, (uint16_t)(data * ANGLE_CONST) );
      break;
    case ROTATE_LEFT:
      issueComplexCommand( 0x07, (uint16_t)(data * ANGLE_CONST) );
      break;
    case START_RUNNING:
      issueSimpleCommand( 0xFA );
      break;
    default:
      issueSimpleCommand( 0xFC );
      break;
  }
}
/*----------------------------------------------------------------------------*/


/**
  * @brief  Izdavanje naredbe u kojoj se salje samo kod komande.
  * @param  command predstavlja kod komande.
  * @retval Nema.
  * @author praetorian ( archmarko92@gmail.com )
  */
void issueSimpleCommand( char command )
{
  sending_array[ 2 ] = 2;
  sending_array[ 3 ] = command;
  checkAndSend();
}
/*----------------------------------------------------------------------------*/


/**
  * @brief  Izdavanje naredbe u kojoj se, pored same komande, salje i neki podatak.
  * @param  command predstavlja kod komande.
  * @param  data predstavlja podatak koji se salje uredjaju.
  * @retval Nema.
  * @author praetorian ( archmarko92@gmail.com )
  */
void issueComplexCommand( char command, uint16_t data )
{
  sending_array[ 2 ] = 8;
  sending_array[ 3 ] = command;
  sending_array[ 4 ] = ( char )( data & 0x000F );
  sending_array[ 5 ] = ( char )( ( data >> 4 ) & 0x000F );
  sending_array[ 6 ] = ( char )( ( data >> 8 ) & 0x000F );
  sending_array[ 7 ] = ( char )( ( data >> 12 ) & 0x000F );
  sending_array[ 8 ] = ( char )( command_ID & 0x000F );
  sending_array[ 9 ] = ( char )( ( command_ID >> 4 ) & 0x000F );
  checkAndSend();
}
/*----------------------------------------------------------------------------*/


/**
  * @brief  Generisanje check sume i zapocinjanje slanja poruke.
  * @param  Nema.
  * @retval Nema.
  * @author praetorian ( archmarko92@gmail.com )
  */
void checkAndSend( void )
{
  int n = sending_array[ 2 ] + 2; // Bajtovi poruke plus duzina poruke plus adresa uredjaja.
  int check_sum = 0;
  for( int i = 1; i < n; i++ ) check_sum += ( sending_array[ i ] & 0xFF );                                               
  sending_array[ n ] = ( check_sum & 0x7F );
  
  /* Zapocni slanje poruke. */
  GPIO_SetBits( GPIOC, GPIO_Pin_12 ); // Otvaranje magistrale za slanje pomocu RS485.
  sending_iterator = 0;
  sending_length = n + 1;
  USART_SendData( USART3,  sending_array[ sending_iterator++ ] );
}
/*----------------------------------------------------------------------------*/


/**
  * @brief  Prijem poruke.
  * @param  received_byte predstavlja bajt koji je pristigao putem USART-a.
  * @retval Nema.
  * @author praetorian ( archmarko92@gmail.com )
  */
void receiveByte( uint16_t received_byte )
{
  static ReceiveStateType state_receive = FIRST_BYTE;
  static int check_sum = 0;
  static int data_cnt = 1;
  receive_array[ 0 ] = 0xFF;
  
  /* Masina stanja za prijem poruke. */
  switch( state_receive )
  {
    case FIRST_BYTE:
      if( received_byte == 0xFF ) state_receive = ADDRESS;
      else state_receive = FIRST_BYTE;
      break;
    case ADDRESS:
      if( received_byte == 0xFF ) state_receive = ADDRESS;
      else 
      {
        check_sum = 0;
        data_cnt = 1;
        state_receive = LENGTH;
        receive_array[ 1 ] = received_byte;
        check_sum += receive_array[ 1 ];
      }
      break;
    case LENGTH:
      if( received_byte == 0xFF ) state_receive = ADDRESS;
      else
      {
        state_receive = DATA;
        receive_array[ 2 ] = received_byte;
        check_sum += receive_array[ 2 ];
      }
      break;
    case DATA:
      if( received_byte == 0xFF ) state_receive = ADDRESS;
      else
      {
        if( data_cnt == receive_array[ 2 ] )
        {
          if( ( check_sum & 0x7F ) == received_byte )
          {
            receive_array[data_cnt+2]=received_byte;
            decodeMessage();
            state_receive = FIRST_BYTE;
          }
          else state_receive = FIRST_BYTE;
        }
        else
        {
          receive_array[ data_cnt + 2 ] = received_byte;
          check_sum += receive_array[ data_cnt + 2 ];
          data_cnt++;
          state_receive = DATA;
        }
      }
      break;
    default:     
      break;
  }
}
/*----------------------------------------------------------------------------*/


/**
  * @brief  Dekodovanje primljene poruke.
  * @param  Nema
  * @retval Nema.
  * @author praetorian ( archmarko92@gmail.com )
  */
void decodeMessage( void )
{
  /* Provera da li je podatak stigao sa ploce za kretanje. */
  if(receive_array[ 1 ] == MOTION_DEVICE_ADDRESS | 0x40 )
  {
    /* Provera da li je pristigla poruka acknowledge signal. */
    if(receive_array[ 2 ] == 1)
    {
      FLAG_ackReceived = TRUE;
    }
    
    /* Provera da li je pristigla poruka informacija o tome da li je robot stigao u poziciju ili ne. */
    else if( receive_array[ 2 ] == 3 )
    {
      int temp_flag = receive_array[ 3 ] | ( receive_array[ 4 ] << 4 );
      FLAG_arriveOnDest = (bool)temp_flag;
    }
    
    /* Slucaj ako nije nijedna od vazecih poruka. Ovde ne bi trebao da ulazi. */
    else
    {
    }
  }
  
  /* Ovde ne bi trebao da ulazi. */
  else
  {
  }
    
}