#ifndef _EUROBOT_MSM_
#define _EUROBOT_MSM_


// typedefs

typedef struct Pos {  // pozicija robota na terenu
   int x;  // koordinata x-ose, duže dimenzije terena   KORAK 1? OPSEG 0 300?
   int y;  // koordinata y-ose, kraæe dimenzije terena  KORAK 1?  OPSEG 0 200?
   int Q;  // ugao koji zaklapa pravac kretanja (okrenutosti) robota sa x-osom  KORAK 1?  OPSEG -180:180?
} Position;

typedef enum { // glavna masina stanja i njena stanja tipa robotState
  idle,       // 0
  goForward,  // 1
  goBackward, // 2
  rotate,     // 3 
  stop,       // 4
  doNothing,  // 5 
  shade,      // 6
  fullstop    // 7
} robotState;
 
 extern robotState nextState = idle;

 
// defines INICIJALIZOVATI INICIJALIZOVATI INICIJALIZOVATI INICIJALIZOVATI 
 
#define PI (3.14159265)
 
#define slaveAddress (0xAA)

#define x0left (0)  // ukoliko smo levi takmicar
#define y0left (0)
#define Q0left (0)

#define x0right (0)  // ukoliko smo desni takmicar
#define y0right (0)
#define Q0right (0)

#define maskFront (0x03)  // maska za prednje UV senzore 
#define maskRear (0x0C)   // maska za prednje UV senzore
#define maskAll (0x00)    // maska za sve UV senzore
#define maskNone (0x0F)   // maska za nijedan UV senzor (nema maske)

#define waitAck (5)         // koliko puta se ponovo salje poruka i ceka ACK ukoliko nije stigao
#define maxAckTime (20)     // max vrednost za koju bi trebalo da stigne ACK po profesoru
#define messageLength (10)  // duzina standardne poruke u bajtovima (char(1B), char(1B), int(4B), int(4B))

// ako se robot nadje u pravougaoniku stranica minUVdistance i (xDoorMax-xDoorMin) ne treba posmatrati senzore jer je u pitanju zatvaranje vrata 
#define minUVdistance (15)  // maksimalna razdaljina od prepreke a da se sigurno mozemo zaustaviti 
#define xDoorMin (50)       
#define xDoorMax (50)

 
// promenljive 

 int leftRight;  // prekidac koji govori da li smo levi (1) ili desni (0) takmicar; POVEZATI HARDVERSKI!
 
// niz od tri poruke koje treba poslati u slucaju da iz nekog razloga treba da popravimo poziciju
unsigned char* repairMessages[3] = { "" }; 
unsigned char* messages[12] = { "" }; // korisceno umesto positions[12] -> problem: daje apsolutne a ne relativne komande

// trenutna pozicija robota na terenu (extern stoji zbog linker greske u suprotnom)
extern Position currentPosition; 
// takticke pozicije u koje treba stici tokom meca; OBAVEZNO SRACUNATI I INICIJALIZOVATI ZA LEVOG/DESNOG TAKMICARA
Position positions[12]; 
// brojac za gornju promenljivu, bitna stvarcica koja izmedju omogucava sinhroni prelazak iz stanja u stanje
int cnt = 0; 

// kretanje robota smoze biti u nekoliko stanja:
// 'i' - initial, 'f' - forward, 'b' - backward, 'r' - rotate, 'w' - wait(UV),'s' - stop(finished), 'x' - repaired
char movementState = 'i';
// flegovi koje ploca za kretanje salje kao potencijalnu signalizaciju da je na jednom od 4 senzora detektovana prepreka
// gledaju se samo 4 donja, dakle binarno: 0 0 0 0 (zadnjiLevi) (zadnjiDesni) (prednjiLevi) (prednjiDesni)
char flegoviUV = 0x00;  // int?
// maska za UV senzore, opet se gledaju samo donja 4 bita, dakle binarno: 0 0 0 0 (zadnjiLevi) (zadnjiDesni) (prednjiLevi) (prednjiDesni)
char maskUV = 0x00;  // int?


  // flegovi

extern int gimme = 0;  // fleg koji javlja da treba da se traze novi podaci o kretanju i senzorima; POSTAVLJA PREKIDNA RUTINA
extern int end = 0;    // fleg koji kaze da je isteklo regularnih 90 sekundi i da treba otvoriti kisobran i ugasiti sve zivo; POSTAVLJA PREKIDNA RUTINA

int start = 0;       // fleg koji kaze da treba da krenemo sa igrom
int wait = 0;        // fleg koji kaze da je instrukcija za kretanje vec poslata drugoj ploci, to jest da li treba da cekamo da kretanje dovede robota u zeljeno mesto; mozda bi trebalo da se zove firstTime
int ack = 0;         // fleg koji kaze da li je stigao ACK signal; POSTAVLJA DecodeCommand()
int already = 0;     // fleg koji kaze da je instrukcija za pomeranje servoe vec poslata
int update = 0;      // fleg koji kaze da li treba azurirati poziciju
int repair = 0;      // fleg koji kaze da treba da se popravi pozicija robota; nije binarni, moze imati vrednosti od 0 do 3
int considerUV = 1;  // fleg koji kaze da li treba da vrednujemo signaliziranje UV-a
int dangerUV = 0;    // fleg koji kaze da treba uci u stop stanje i sacekati da se prepreka skloni


// definicije funkcija

// funkcija koja postavlja pocetne uslove, prema tome da li smo levi ili desni takmicar
void initLeftRight(int leftRight) {
  // NA OVOM MESTU TREBA POSTAVITI SVE POCETNE USLOVE KOJI ZAVISE OD TOGA DA LI SMO LEVI/DESNI TAKMICAR
  // TO MOGU BITI I DRUGE KONSTANTE, NPR. NIZ POSITIONS[], A JEDAN PRIMER JE DAT ISPOD
  if(leftRight) {
          currentPosition.x = x0left;
          currentPosition.y = y0left;
          currentPosition.Q = Q0left;
  }
  else {
          currentPosition.x = x0right;
          currentPosition.y = y0right;
          currentPosition.Q = Q0right;
  }
}

// funkcija koja inicijalizuje TIM4 i pin B9 za kontrolu servomotora za otvaranje kisobrana
void initParasolServo() {
  // Inicijalizacija pina na koji se salje PWM signal za kontrolu serva.
  InitGPIOPin(GPIOB, GPIO_Pin_9, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
        
  // Perioda 10 ms sa koracima od 10 us.
  InitTIM_TimeBase(TIM4, 240 - 1, 1000 - 1, TIM_CounterMode_Up, TIM_CKD_DIV1, 0);
        
  // Kanal koji salje PWM signal za kontrolu serva.
  InitTIM_OC(TIM4, TIM_Channel_4, TIM_OutputState_Enable, TIM_OCMode_PWM1, 0, TIM_OCPolarity_High);

  // Kanal koji generise prekide za funkciju delay. Ovde se ne poziva funkcija za inicijalizaciju
  // posto izlaz kanala nije otvoren. Zato se tajmer mora posebno pokrenuti!!!
  
};

// inicijalizacija tajmera TIM3 koji odbrojava trajanje meca; trebalo bi da je dobro podeseno, na 90s
void initTimer90() {
  InitTIM_TimeBase(TIM3, 45000 - 1, 48000 - 1, TIM_CounterMode_Up, TIM_CKD_DIV1, 0);
  TIM_Cmd(TIM3, ENABLE);
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
  InitNVICChannel(TIM3_IRQn, 0, 0, ENABLE);
};

// inicijalizacija funkcije koja omogucava delay potreban za ACK i za servomotor
void initDelayTimer () {
  InitTIM_TimeBase(TIM2, 24 - 1, 1000 - 1, TIM_CounterMode_Up, TIM_CKD_DIV1, 0);  // 1ms
  TIM_Cmd(TIM2, ENABLE);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  InitNVICChannel(TIM2_IRQn, 0, 0, ENABLE);  // bilo 0, 1
}

#include <math.h>

// funkcije za konvertovanje poruke u UV flegove, stanje kretanja i broj otkucaja 2 enkodera nisu napisane 
// isto vazi za konvertovanje otkucaja enkodera u poziciju ili u centimetre koje se koriste posebno ili kao delovi gornjih funkcija
// te funkcije se koriste u ostatku koda, a parametri i povratne vrednosti dati su u njihovim potpisima
// POTREBNO JE NAPISATI TE FUNKCIJE
// napomena: radi jednostavnosti funkcije koriste samo jednu vrednost za otkucaje enkodera; TO TREBA PROMENITI
Position message2position(unsigned char* m) {
  
};
unsigned char* encoder2message(int i) { // napomena: koristi vrednosti za 2 enkodera 

};
int angle2encoder(float f) {  // napomena: vraca vrednosti za 2 enkodera 

};
int distance2encoder(float f) {

};
char message2UV(unsigned char* m) { // recimo, to moze biti 1. bajt poruke

};
char mesage2movementState(unsigned char* m) {  // recimo, to moze biti 2. bajt poruke

};


// NAJBITNIJA I NAJKOMPLIKOVANIJA F-JA
// funkcija koja azurira poziciju robota na terenu, tj. racuna koordinate x, y i Q na osnovu 
// takodje, funkcija obradjuje informaciju koja p
void updatePosition() {
  
  // PORUKA: UV flegovi (1 char), stanje kretanja (1 char), otkucaji enkodera (2 x int)
  
  unsigned char* message = (unsigned char *) GetMessage();  // može li sa druge ploèe da stigne druga poruka u meðuvremenu? verovatno ne
  
  currentPosition = message2position(message);    // azuriranje trenutne pozicije
  movementState = mesage2movementState(message);  // azuriranje stanja kretanja
  flegoviUV = message2UV(message);                // azuriranje UV flegova
 
  // deo za UV
  if((currentPosition.y < minUVdistance) && (xDoorMin < currentPosition.x) && (currentPosition.x < xDoorMax) && (currentPosition.Q > 0) && (currentPosition.Q < 180)) {
        // najpre, provera da li uopste treba vrednovati flegove UV-a: npr., ako zatvaramo vrata ne treba
        // ovakav slican deo moze se dodati za vise oblasti na terenu, trenutno je odradjeno samo za vrata na jednoj strani
        // TREBA SMISLITI GDE TACNO NE TREBA GLEDATI SENZORE I UBACITI NEKOLIKO OVAKVIH DELOVA KODA
        // VODIITI RACUNA O TOME STA SU X I Y I GDE JE KOORDINATNI POCETAK (kao kad se racunaju pocetni uslovi)
    considerUV = 0;
    dangerUV = 0;
  }
  else {  // ako senzore treba vrednovati
    
    if( movementState == 'f') // ako idemo napred, maskiramo zadnje senzore
       maskUV = maskRear;
    else if( movementState == 'b')   // ako idemo nazad, maskiramo prednje senzore
       maskUV = maskFront;
    else if ( movementState == 'w')  // ako smo stali zbog prepreke
       maskUV = maskNone;
    else  // ako ne idemo ni napred ni nazad, maskiramo sve senzore
       maskUV = maskAll;
    if( (considerUV == 1) /* prvi uslov nije neophodan */ && ( (flegoviUV & maskUV) != 0) ) { // ako nisu svi flegovi resetovani i ako uopšte treba da se vrednuju senzori
      dangerUV = 1;
      for(int i=0; i < waitAck; i++) {
        SendMessage(slaveAddress, "wait", 4);  // ako ima neki brzi nacin moze
        delay(maxAckTime);
        if (ack) {
            ack = 0;
            break;
        }
      }
      movementState = 'w';
    }
    else 
      dangerUV = 0;
  }
  
  if(!dangerUV) {
    // deo za kretanje
    // ako je robot stao ('w','s') a treba da se popravi pozicija(2. uslov) i vec nije popravljana('x')
    // BILO BI LEPO DA SE KRETANJE VEC POSTAVLJENO U 'S' ILI 'W' AKO SE DOSLO DOVDE
    if( (movementState == 's' || movementState == 'w' && movementState != 'x') /* ako smo stali iz nekog razloga a vec nije odradjen repair*/ 
     && ( ( currentPosition.x != positions[cnt].x ) || ( currentPosition.y != positions[cnt].y ) || ( currentPosition.Q != positions[cnt].Q ) ) /* ako treba da se popravi pozicija */ ) {
      // projekcije vektora od trenunte pozicije do zeljene pozicije na x i y-osu
      int xProj = positions[cnt].x - currentPosition.x; // a ako je nula? trebalo bi da radi atan f-ja 
      int yProj = positions[cnt].y - currentPosition.y;
      // rastojanje izmeðu trenutne i željene pozicije, racuna se preko vektora 
      float distance = sqrt ( xProj^2 + yProj^2 );  // MODULE = SQRT( x*x + y*y)
      // ugao zbira gorepomenutog vektora i vektora kojim smo prvobitno pokusali doci do zeljene pozicije
      float angle = atan(yProj/xProj) * 180 / PI;  // bolje je koristiti atan2 (y,x) * 180 / PI;
      // popravka ugla u zavisnosti od kvadranta
      if (yProj < 0 && xProj > 0)  // 4. kvadrant
        angle = angle + 360;
      else if (xProj < 0) // 2. i 3. kvadrant
        angle = angle + 180;
      // pravljenje poruka za ispravljanje pozicije: 1) rotiramo ka zeljenoj tacki, 2) dodjemo do nje, 3) rotiramo ka sledecem zeljenom uglu
      repairMessages[2] = encoder2message(angle2encoder(angle)); // koliko treba da se zarotira ka zeljenoj tacki
      repairMessages[1] =  encoder2message(distance2encoder(distance));  // koliko treba da se ide ka zeljenoj tacki
      repairMessages[0] = encoder2message(distance2encoder(positions[cnt].Q));  // ugao u koji smo hteli doci
      repair = 3;
      wait = 0; // zavrsilo se bilo kakvo kretanje pa se opet dozvoljava slanje poruke drugoj ploci
      ++cnt; // predji na sledecu poruku; koristi se tek kad se zavrsi repair; PRETPOSTAVKA JE DA SE NECE UCI U STOP STANJE (jer se najcesce popravljaju 'sitnice')
    }
    else  if( (movementState == 's') ||(movementState == 'w') || (movementState == 'x')) {
      /*ne treba da se popravlja*/
      if (movementState == 's')  // ako se stalo iz najnormalnijeg razloga - dosli smo u odgovarajucu poziciju  // && != 'x' ?
         ++cnt;
      wait = 0;
    }
    else if (movementState == 'f' || movementState == 'r' || movementState == 'b')
      wait = 1;
  }    
}


// funkcija iz EUROBOT_serial.h, samo setuje ack ukoliko je dosao ACK ili update ukoliko je dosla obicna poruka
void DecodeCommand(){
    // po Blakiju, poruka ne mora da se èuva, nalaziæe se u GetMessage-u i ne bi trebalo da ce je nesto prebrisati!
  if(GetMessage()=="")
    ack = 1;
  else
    update = 1;
}

// koja na osnovu trenutne pozicije i sledece zeljene pozicije koja zavisi od cnt pravi poruku za kretanje
// u prinicipu, trebalo bi da se koristi samo za kretanje napred, verovanto i za nazad, a ne za rotaciju
unsigned char * nextMessage(int cnt) {
  return encoder2message( distance2encoder ( sqrt ( (positions[cnt].x - currentPosition.x )^2 + (positions[cnt].y - currentPosition.y )^2 )));
};


// funkcija koja na osnovu sinhronog brojaca 'cnt' koji kaze dokle smo stigli sa planiranim izvrsavanjem masine stanja
// i eksternih, 'asinhronih' signala odreðuje sledece stanje
// kako se ide ka kraju funkcije, prioriteti asinhronih signala su sve veci
void setNextState() {
  
  if(start) { // ako je uopste pokrenuta masina stanja
    
    // normalni prelasci u stanja
    
    switch(cnt) {
	case 0:
	case 3:
	case 6:
	case 8:
	case 11:
          nextState = goForward;
          break;
	case 1:
	case 4:
	case 9:
          nextState = goForward; 
          break;
	case 2:
	case 5:
	case 7:
	case 10:
          nextState = rotate;
          break;
	default:
          nextState = doNothing;  // ukoliko eventualno nesto podje po zlu, najbolje je da ne radimo nista  
    }
  
  // 'vanredni' prelasci u stanja
  
  if(repair) { // popravljanje pozicije robota
    if(repair == 2)
      nextState = goForward; 
     else // 1, 3
      nextState = rotate; 
  }
    
  if(dangerUV) // zaustavljanje robota zbog signaliziranja UV-a (prepreka)
    nextState = stop;
  
  if(end)  // najprioritetnije na kraju: zaustavljanje robota nakon isteka devedeset (i nešto) sekundi
    nextState = fullstop;
 
  }
  
}


#endif
