struct {
	   unsigned char status;
	   unsigned char state;
} task_pid_X, task_pid_Y;

volatile int ENC1=32768, ENC1_old = 32768;
volatile int ENC2 = 32768, ENC2_old = 32768;
unsigned char motion_type_X=1;
unsigned char speed_current_X;
unsigned char speed_req_X;
unsigned char maximum_speed_X=60;
unsigned int position_inc_X;
unsigned char motion_type_Y=1;
unsigned char speed_current_Y;
unsigned char speed_req_Y;
unsigned char maximum_speed_Y=60;
unsigned int position_inc_Y;
int zadata_pozicija_X=32767, zadata_pozicija_Y=32767;
int trenutna_pozicija_X=32767, trenutna_pozicija_Y=32767; 
int greska_pracenja_X, greska_pracenja_Y;
unsigned char command_ID=255;     
unsigned char ENC1A_edge=0, ENC1B_edge=0; 
unsigned char ENC2A_edge=0, ENC2B_edge=0;
int data_log[512];
int speed_correction_Y=0;
float angularConstant=0.0375;
long abs_X=0,abs_Y=0;
float abs_Theta=0;




int test=0;
  

//logovanje kretanja...
//trebalo bi logovati u vremenu zadatu poziciju, kao i poziciju po enkoderima



//stari sistem: TIM1-CCR1 pwm1
//TIM1-CCR1 pwm2
//TIM2 radi proracun odometrije
//TIM3 ce da radi kao loger
//tim4 i tim7 implementiraju pozicione kontrolere za ruke
