#define INP_TOLERANCE 30

extern const unsigned char speed_table_acc[];
extern const unsigned char speed_table_decc[];
extern const unsigned int acc_table[];

extern struct {
	   unsigned char status;
	   unsigned char state;
} task_pid_X, task_pid_Y;

extern volatile int ENC1, ENC1_old;
extern volatile int ENC2, ENC2_old;

extern unsigned char motion_type_X;
extern unsigned char speed_current_X;
extern unsigned char speed_req_X;
extern unsigned char maximum_speed_X;
extern unsigned int position_inc_X;
extern unsigned char motion_type_Y;
extern unsigned char speed_current_Y;
extern unsigned char speed_req_Y;
extern unsigned char maximum_speed_Y;
extern unsigned int position_inc_Y;
extern int zadata_pozicija_X, zadata_pozicija_Y;
extern int trenutna_pozicija_X, trenutna_pozicija_Y;
extern int greska_pracenja_X, greska_pracenja_Y;
extern unsigned char command_ID;
extern unsigned char ENC1A_edge, ENC1B_edge; 
extern unsigned char ENC2A_edge, ENC2B_edge;
extern int data_log[512];
extern int speed_correction_Y;
extern float angularConstant;
extern long abs_X,abs_Y;
extern float abs_Theta;
           
extern int test;