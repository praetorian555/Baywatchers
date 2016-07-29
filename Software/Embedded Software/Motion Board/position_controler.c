#include "variables.h"
#include "functions.h"
#include "stm32f10x.h"


void PositionControllerInit(void){
  
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
    
    //  Konfiguracija Timera 2 za implementaciju pozicionog kontrolera
    //  Period = 1ms
    TIM_TimeBaseInitStruct.TIM_Period = 100;
    TIM_TimeBaseInitStruct.TIM_Prescaler = 600;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
    TIM_ITConfig(TIM2,TIM_IT_Update, ENABLE);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);  //Startovanje TIMER4 periferije
    
    //DOZVOLA GENERISANJA PREKIDA
    //Dozvola generisanja prekida od strane TIM4 periferije
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //Inicijalizacija NVIC periferije
    NVIC_Init(&NVIC_InitStructure);
    
    //  Konfiguracija Timera 7 za implementaciju pozicionog kontrolera
    //  Period = 1ms
    TIM_TimeBaseInitStruct.TIM_Period = 100;
    TIM_TimeBaseInitStruct.TIM_Prescaler = 600;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseInitStruct);
    TIM_ITConfig(TIM7,TIM_IT_Update, ENABLE);
    TIM_ARRPreloadConfig(TIM7, ENABLE);
    TIM_Cmd(TIM7, ENABLE);  //Startovanje TIMER4 periferije
    
    //DOZVOLA GENERISANJA PREKIDA
    //Dozvola generisanja prekida od strane TIM7 periferije
    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    task_pid_X.status=1;
    task_pid_X.state=99;  
    task_pid_Y.status=1;
    task_pid_Y.state=99;
    
    motion_type_X=1;
    motion_type_Y=1;
}

void TIM2_IRQHandler(void)
{
  greska_pracenja_X=abs(zadata_pozicija_X-ENC1);
  position_controler_X();  
  TIM_ClearFlag(TIM2, TIM_FLAG_Update); 
}

void TIM7_IRQHandler(void)
{  
  greska_pracenja_Y=abs(zadata_pozicija_Y-ENC2);
  if (greska_pracenja_Y>(greska_pracenja_X+10)) {
    speed_correction_Y=-1;
  }
  else if (greska_pracenja_Y<(greska_pracenja_X-10)) {
    speed_correction_Y=1;
  }
  else speed_correction_Y=0;
  speed_correction_Y=0;
  position_controler_Y();
  TIM_ClearFlag(TIM7, TIM_FLAG_Update); 
}

//pracenje
/*
ENC1 je vodeci, a ENC2 prati
razlika izmedju zadate pozicije servo regulatora i ENC1 pozicije je neka greska pracenja koja jeste referentna
pracenje1=trenutna_pozicija_X-ENC1
ista tolika greska treba da bude kod motora 2
Medjutim ta greska kod motora 2 je
pracenje2=trenutna_pozicija_Y-ENC2

ako je pracenje2>pracenje1 onda treba malo ubrzati M2 i obrnuto
e sad, ubraznje se svodi samo na to da se malo ubrza posicioni kontroler, a to znaci da se malo poveca ucestanost tog regulatora
uvescemo speed_correction_Y koja ce biti biti samo +1 ili -1...dakle jednostavno ako je pracenje >0 onda je speed_correction_Y=1
u suprotnom je speed_correction=-1

*/

void position_controler_X (void) {
long temp1;
switch(task_pid_X.status){
	  case 1://{	
		switch (task_pid_X.state) {
	 	case 1:{//odredjivanje smera rotacije
		 	  if (zadata_pozicija_X>trenutna_pozicija_X) {
 			    //command_servoX_forward;				
    		  }
    		  else if (zadata_pozicija_X<trenutna_pozicija_X) {
 			    //command_servoX_reverse;
    		  }
			  task_pid_X.state=2;
			  speed_current_X=0;
                          position_inc_X=0;
                          motion_type_X=1;
		};break; 
		case 2: {	
		   if (zadata_pozicija_X>trenutna_pozicija_X) {
 			    temp1=zadata_pozicija_X-trenutna_pozicija_X;
				//command_servoX_forward;
			  	//if (!input_limit_switch_X1){
				   //command_servoX_pulse;
			  	   trenutna_pozicija_X++;
				//}
				//else task_pid_X.state=99;
    		  }
                  else if (zadata_pozicija_X<trenutna_pozicija_X) {
 			    temp1=trenutna_pozicija_X-zadata_pozicija_X;
				//command_servoX_reverse;
				//if (!input_limit_switch_X0){
			  	   //command_servoX_pulse;
			  	   trenutna_pozicija_X--;
				//}
				//else {
				//	 trenutna_pozicija_X=0;
				//	 task_pid_X.state=99;
				//};
    		  }
		  else if (zadata_pozicija_X==trenutna_pozicija_X) task_pid_X.state=99;
		   
		   //odredjivanje brzine/////////////////////		   
		   if (temp1>0){
 		   	  if (temp1>=1999) speed_req_X=speed_table_decc[1999];//maksimalna brzina
 		   	  else speed_req_X=speed_table_decc[temp1];
 		   	  position_inc_X++;//ovo je isto sto i pulse_table_position u negreconu
 		   	  if (motion_type_X==0) {//mod usporenja prilikom priblizavanja cilju
	   	   	  	 if (speed_current_X>=speed_req_X) speed_current_X=speed_req_X;//zaista usporava
	   	   	  	 else {//u toku usporavanja je dobio komandu da ide prema sledecem cilju tj. da ubrzava
		   	  	 	motion_type_X=1;//ocigledno pocinje da ubrzava pa se menja tip kretanja		 
		   	  	 	position_inc_X=acc_table[speed_current_X];//sada treba da nadje position_inc u tabeli za ubrzanja
	   	   	  	 };
 		   	  }
 		   	  else {//mod ubrzanja 	
	   	   	  	 if (speed_current_X>=speed_req_X) {
	   	   	  	 	motion_type_X=0;//prebacuje se na usporenje
	   	   	  	 	speed_current_X=speed_req_X;//zaista usporava
	   	   	  	 }
	   	   	  	 else {//ubrzava prema tabeli za ubrzanje
	   	   	  	 	speed_current_X=speed_table_acc[position_inc_X]; 
	   	   	  	 };
 		   	  };
 		   	  if (speed_current_X<maximum_speed_X){	   	   	  	 
 	   	   	  	 //OCR1AH=OCR_high[speed_current_X];
				// OCR1AL=OCR_low[speed_current_X];//uzuriranje OCR vrednosti
                                TIM2->ARR = 1+(int)(100/speed_current_X);  
		   	  }
		   	  else {
			     //OCR1AH=OCR_high[maximum_speed_X];
	   	   	  	 //OCR1AL=OCR_low[maximum_speed_X];//uzuriranje OCR vrednosti  
                                TIM2->ARR = 1+(int)(100/maximum_speed_X);              
		   	  };
 		   }
 		   else {
 		   		//ako je brzina nula???
 		   };   
		}; break;
		case 99:{
		 	  //zavrseno pozicioniranje
                  if (zadata_pozicija_X!=trenutna_pozicija_X) task_pid_X.state=1; 
                  TIM2->ARR=1;
		}; break; 		
		default: break;
	   }; break;//case ON
	  case 0 : {
		//sta kad se ugasi???

	  };break;
     };	//switch status
}

void position_controler_Y (void) {
long temp1;
switch(task_pid_Y.status){
	  case 1://{	
		switch (task_pid_Y.state) {
	 	case 1:{//odredjivanje smera rotacije
		 	  if (zadata_pozicija_Y>trenutna_pozicija_Y) {
 			    //command_servoX_forward;				
    		  }
    		  else if (zadata_pozicija_Y<trenutna_pozicija_Y) {
 			    //command_servoX_reverse;
    		  }
			  task_pid_Y.state=2;
			  speed_current_Y=0;
                          position_inc_Y=0;
                          motion_type_Y=1;
		};break; 
		case 2: {	
		   if (zadata_pozicija_Y>trenutna_pozicija_Y) {
 			    temp1=zadata_pozicija_Y-trenutna_pozicija_Y;
				//command_servoX_forward;
			  	//if (!input_limit_switch_Y1){
				   //command_servoX_pulse;
			  	   trenutna_pozicija_Y++;
				//}
				//else task_pid_Y.state=99;
    		  }
    	   else if (zadata_pozicija_Y<trenutna_pozicija_Y) {
 			    temp1=trenutna_pozicija_Y-zadata_pozicija_Y;
				//command_servoX_reverse;
				//if (!input_limit_switch_Y0){
			  	   //command_servoX_pulse;
			  	   trenutna_pozicija_Y--;
				//}
				//else {
				//	 trenutna_pozicija_Y=0;
				//	 task_pid_Y.state=99;
				//};
    		  }
		   else if (zadata_pozicija_Y==trenutna_pozicija_Y) task_pid_Y.state=99;
		   
		   //odredjivanje brzine/////////////////////		   
		   if (temp1>0){
 		   	  if (temp1>=1999) speed_req_Y=speed_table_decc[1999];//maksimalna brzina
 		   	  else speed_req_Y=speed_table_decc[temp1];
 		   	  position_inc_Y++;//ovo je isto sto i pulse_table_position u negreconu
 		   	  if (motion_type_Y==0) {//mod usporenja prilikom priblizavanja cilju
	   	   	  	 if (speed_current_Y>=speed_req_Y) speed_current_Y=speed_req_Y;//zaista usporava
	   	   	  	 else {//u toku usporavanja je dobio komandu da ide prema sledecem cilju tj. da ubrzava
		   	  	 	motion_type_Y=1;//ocigledno pocinje da ubrzava pa se menja tip kretanja		 
		   	  	 	position_inc_Y=acc_table[speed_current_Y];//sada treba da nadje position_inc u tabeli za ubrzanja
	   	   	  	 };
 		   	  }
 		   	  else {//mod ubrzanja 	
	   	   	  	 if (speed_current_Y>=speed_req_Y) {
	   	   	  	 	motion_type_Y=0;//prebacuje se na usporenje
	   	   	  	 	speed_current_Y=speed_req_Y;//zaista usporava
	   	   	  	 }
	   	   	  	 else {//ubrzava prema tabeli za ubrzanje
	   	   	  	 	speed_current_Y=speed_table_acc[position_inc_Y]; 
	   	   	  	 };
 		   	  };
 		   	  if (speed_current_Y<maximum_speed_Y){	   	   	  	 
 	   	   	  	 //OCR1AH=OCR_high[speed_current_Y];
				// OCR1AL=OCR_low[speed_current_Y];//uzuriranje OCR vrednosti
                                speed_current_Y=speed_current_Y+speed_correction_Y;
                                TIM7->ARR = 1+(int)(100/speed_current_Y);  
		   	  }
		   	  else {
			     //OCR1AH=OCR_high[maximum_speed_Y];
	   	   	  	 //OCR1AL=OCR_low[maximum_speed_Y];//uzuriranje OCR vrednosti  
                                TIM7->ARR = 1+(int)(100/maximum_speed_Y);              
		   	  };
 		   }
 		   else {
 		   		//ako je brzina nula???
 		   };   
		}; break;
		case 99:{
		 	  //zavrseno pozicioniranje
                  if (zadata_pozicija_Y!=trenutna_pozicija_Y) task_pid_Y.state=1;
                  TIM7->ARR=1; 
		}; break; 		
		default: break;
	   }; break;//case ON
	  case 0 : {
		//sta kad se ugasi???

	  };break;
     };	//switch status
}



int Speed_profile_X (int zadata_pozicija_X, int trenutna_pozicija_X) {
long temp1;
switch(task_pid_X.status){
	  case 1://{	
		switch (task_pid_X.state) {
	 	case 1:{//odredjivanje smera rotacije
		 	  if (zadata_pozicija_X>trenutna_pozicija_X) {
 			    //command_servoX_forward;				
    		  }
    		  else if (zadata_pozicija_X<trenutna_pozicija_X) {
 			    //command_servoX_reverse;
    		  }
			  task_pid_X.state=2;
			  speed_current_X=0;
                          position_inc_X=0;
                          motion_type_X=1;
		};break; 
		case 2: {	
		   if (zadata_pozicija_X>trenutna_pozicija_X) {
 			    temp1=zadata_pozicija_X-trenutna_pozicija_X;
				//command_servoX_forward;
			  	//if (!input_limit_switch_X1){
				   //command_servoX_pulse;
			  	   trenutna_pozicija_X++;
				//}
				//else task_pid_X.state=99;
    		  }
                  else if (zadata_pozicija_X<trenutna_pozicija_X) {
 			    temp1=trenutna_pozicija_X-zadata_pozicija_X;
				//command_servoX_reverse;
				//if (!input_limit_switch_X0){
			  	   //command_servoX_pulse;
			  	   trenutna_pozicija_X--;
				//}
				//else {
				//	 trenutna_pozicija_X=0;
				//	 task_pid_X.state=99;
				//};
    		  }
		  else if (zadata_pozicija_X==trenutna_pozicija_X) task_pid_X.state=99;
		   
		   //odredjivanje brzine/////////////////////		   
		   if (temp1>0){
 		   	  if (temp1>=1999) speed_req_X=speed_table_decc[1999];//maksimalna brzina
 		   	  else speed_req_X=speed_table_decc[temp1];
 		   	  position_inc_X++;//ovo je isto sto i pulse_table_position u negreconu
 		   	  if (motion_type_X==0) {//mod usporenja prilikom priblizavanja cilju
	   	   	  	 if (speed_current_X>=speed_req_X) speed_current_X=speed_req_X;//zaista usporava
	   	   	  	 else {//u toku usporavanja je dobio komandu da ide prema sledecem cilju tj. da ubrzava
		   	  	 	motion_type_X=1;//ocigledno pocinje da ubrzava pa se menja tip kretanja		 
		   	  	 	position_inc_X=acc_table[speed_current_X];//sada treba da nadje position_inc u tabeli za ubrzanja
	   	   	  	 };
 		   	  }
 		   	  else {//mod ubrzanja 	
	   	   	  	 if (speed_current_X>=speed_req_X) {
	   	   	  	 	motion_type_X=0;//prebacuje se na usporenje
	   	   	  	 	speed_current_X=speed_req_X;//zaista usporava
	   	   	  	 }
	   	   	  	 else {//ubrzava prema tabeli za ubrzanje
	   	   	  	 	speed_current_X=speed_table_acc[position_inc_X]; 
	   	   	  	 };
 		   	  };
 		   	  if (speed_current_X<maximum_speed_X){	   	   	  	 
 	   	   	  	 //OCR1AH=OCR_high[speed_current_X];
				// OCR1AL=OCR_low[speed_current_X];//uzuriranje OCR vrednosti
                                TIM4->ARR = 1+(int)(100/speed_current_X);  
		   	  }
		   	  else {
			     //OCR1AH=OCR_high[maximum_speed_X];
	   	   	  	 //OCR1AL=OCR_low[maximum_speed_X];//uzuriranje OCR vrednosti  
                                TIM4->ARR = 1+(int)(100/maximum_speed_X);              
		   	  };
 		   }
 		   else {
 		   		//ako je brzina nula???
 		   };   
		}; break;
		case 99:{
		 	  //zavrseno pozicioniranje
                  if (zadata_pozicija_X!=trenutna_pozicija_X) task_pid_X.state=1; 
		}; break; 		
		default: break;
	   }; break;//case ON
	  case 0 : {
		//sta kad se ugasi???

	  };break;
     };	//switch status

      if (speed_current_X>maximum_speed_X) return maximum_speed_X;
      else return speed_current_X;
}

int Speed_profile_Y (int zadata_pozicija_Y, int trenutna_pozicija_Y) {
long temp1;
unsigned char znak;
speed_current_Y=0;
switch(task_pid_Y.status){
	  case 1://{	
		switch (task_pid_Y.state) {
	 	case 1:{//odredjivanje smera rotacije
		 	  if (zadata_pozicija_Y>trenutna_pozicija_Y) {
 			    //command_servoX_forward;				
    		  }
    		  else if (zadata_pozicija_Y<trenutna_pozicija_Y) {
 			    //command_servoX_reverse;
    		  }
			  task_pid_Y.state=2;
			  speed_current_Y=0;
                          position_inc_Y=0;
                          motion_type_Y=1;
		};break; 
		case 2: {	
		   if (zadata_pozicija_Y>trenutna_pozicija_Y) {
 			    temp1=zadata_pozicija_Y-trenutna_pozicija_Y;
                                znak=1;
    		  }
                  else if (zadata_pozicija_Y<trenutna_pozicija_Y) {
 			    temp1=trenutna_pozicija_Y-zadata_pozicija_Y;
                                znak=0;
    		  }
		  else if (zadata_pozicija_Y==trenutna_pozicija_Y) {
                    task_pid_Y.state=99;                    
                  }
		   
		   //odredjivanje brzine/////////////////////		   
		   if (temp1>0){
 		   	  if (temp1>=1999) speed_req_Y=speed_table_decc[1999];//maksimalna brzina
 		   	  else speed_req_Y=speed_table_decc[temp1];
 		   	  //position_inc_Y++;//ovo je isto sto i pulse_table_position u negreconu
 		   	  if (motion_type_Y==0) {//mod usporenja prilikom priblizavanja cilju
	   	   	  	 if (speed_current_Y>=speed_req_Y) speed_current_Y=speed_req_Y;//zaista usporava
	   	   	  	 else {//u toku usporavanja je dobio komandu da ide prema sledecem cilju tj. da ubrzava
		   	  	 	motion_type_Y=1;//ocigledno pocinje da ubrzava pa se menja tip kretanja		 
		   	  	 	position_inc_Y=acc_table[speed_current_Y];//sada treba da nadje position_inc u tabeli za ubrzanja
	   	   	  	 };
 		   	  }
 		   	  else {//mod ubrzanja 	
	   	   	  	 if (speed_current_Y>=speed_req_Y) {
	   	   	  	 	motion_type_Y=0;//prebacuje se na usporenje
	   	   	  	 	speed_current_Y=speed_req_Y;//zaista usporava
	   	   	  	 }
	   	   	  	 else {//ubrzava prema tabeli za ubrzanje
	   	   	  	 	speed_current_Y=speed_table_acc[position_inc_Y]; 
	   	   	  	 };
 		   	  };
 		   	 /* if (speed_current_Y<maximum_speed_Y){	   	   	  	 
                                TIM4->ARR = 1+(int)(100/speed_current_Y);  
		   	  }
		   	  else {
                                TIM4->ARR = 1+(int)(100/maximum_speed_Y);              
		   	  };*/
 		   }
 		   else {
 		   		//ako je brzina nula???
                     speed_current_Y=0;
 		   };   
		}; break;
		case 99:{
		 	  //zavrseno pozicioniranje
                  if (zadata_pozicija_Y!=trenutna_pozicija_Y) task_pid_Y.state=1; 
		}; break; 		
		default: break;
	   }; break;//case ON
	  case 0 : {
		//sta kad se ugasi???

	  };break;
     };	//switch status

    if (speed_current_Y>maximum_speed_Y){ 
        if (znak==1) return maximum_speed_Y;
        else return -maximum_speed_Y;
    }
    else {
        if (znak==1) return speed_current_Y;
        else return -speed_current_Y;
    };
}



void LogerInit(void){
  
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    
    //  Konfiguracija Timera 3 za implementaciju logera
    //  Period = 1ms
    TIM_TimeBaseInitStruct.TIM_Period = 100;
    TIM_TimeBaseInitStruct.TIM_Prescaler = 600;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);
    TIM_ITConfig(TIM3,TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM3, ENABLE);  //Startovanje TIMER4 periferije
    
    //DOZVOLA GENERISANJA PREKIDA
    //Dozvola generisanja prekida od strane TIM4 periferije
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //Inicijalizacija NVIC periferije
    NVIC_Init(&NVIC_InitStructure);   
}


void TIM3_IRQHandler(void)
{
  //funkcija za logovanje
  TIM_ClearFlag(TIM3, TIM_FLAG_Update); 
}
