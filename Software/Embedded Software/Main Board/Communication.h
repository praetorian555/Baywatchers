#ifndef __COMMUNICATION_H__
#define __COMMUNICATION_H__

/* Moguce komande. */
typedef enum
{
  ULTRASOUND_ON,
  ULTRASOUND_OFF,
  PRESCALER,
  MOVE_FORWARD,
  MOVE_BACKWARD,
  ROTATE_LEFT,
  ROTATE_RIGHT,
  CHECK_ARRIVE,
  STOP,
  START_RUNNING
} CommandNameType;

typedef enum
{
  FIRST_BYTE,
  ADDRESS,
  LENGTH,
  DATA,
  CHECK_SUM
} ReceiveStateType;
  
/* Slanje komande zeljenom uredjaju. */
void issueCommand(CommandNameType command, int receiver_address, uint16_t data );
/* Izdavanje naredbe u kojoj se salje samo kod komande. */
void issueSimpleCommand( char command );
/* Izdavanje naredbe u kojoj se, pored same komande, salje i neki podatak. */
void issueComplexCommand( char command, uint16_t data );
/* Generisanje check sume i zapocinjanje slanja poruke. */
void checkAndSend( void );
/* Prijem poruke. */
void receiveByte( uint16_t received_byte );
/* Dekodovanje primljene poruke. */
void decodeMessage( void );


#endif