#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>
#include<avr/wdt.h>

#define sbi(X,Y) X|=(1<<Y)
#define cbi(X,Y) X&=~(1<<Y)
#define tog(X,Y) X^=(1<<Y)

#define TOP 59926

uint8_t n1=0;
uint16_t reset_count=0;

void rf_pwm_init()
{
  sbi(DDRD,5);
  TCCR0A|=(1<<WGM00); //Set on down counting, Clear on up counting (COM bit activated in start_rf() )      
  sbi(TCCR0B,WGM02);              //Phase correct PWM with OCR0A as top
  OCR0A=200;
  OCR0B=100;
  TCNT0=200;
//  sei();
  sbi(TCCR0B,CS01);       //prescaler 8
  sbi(PORTD,5);
}
inline void start_rf()
{
  TCNT0=200;
  sbi(TCCR0A,COM0B1);
}
inline void high_stop_rf()
{
  sbi(DDRB,5);
  cbi(TCCR0A,COM0B1);
}
#define low_rf() DDRB&=0b11011111
void ultrasound_init()
{//Trig part
  DDRB|=(1<<3)|(1<<4);
  OCR2A=160;
  sbi(TIMSK2,1);    //OCIEA
  TCNT2=0;
}
inline void trig()
{
  if(n1)
    sbi(PORTB,4);
  else
    sbi(PORTB,3);
  sbi(TCCR2B,CS20);
}
void TCNT1_init()
{
  ICR1=TOP;
  OCR1A=TOP-9900;
  OCR1B=100;
  TCCR1B|=(1<<WGM12)|(1<<WGM13); //CTC with ICR1 as TOP
  TIMSK1|=(1<<1)|(1<<2)|(1<<5); //OCIEA , OCIEB & ICIE Interrupt Enable
  TCNT1=0;
  sei();
  sbi(TCCR1B,CS11);   //Prescaler 8
}

int main()
{
//  Serial.begin(2000000);
  rf_pwm_init();
  ultrasound_init();
  TCNT1_init();

 /* MCUSR
  WDTCSR|=(1<<WDCE);
  WDTCSR|=(1<<WDP3)|(1<<WDP0);
  WDTCSR&=~( (1<<WDP1)|(1<<WDP2) );
  WDTCSR|=(1<<WDIE); */
  
  while(1);
  return 0;
}
/*
ISR(INT0_vect)
{
  if(flag_TC1B)
  {
    low_rf();
    OCR1B=TCNT1+1000;
    flag_TC1B=0;
  }
}
*/
ISR(TIMER1_CAPT_vect)
{
  n1=n1?0:1;
  OCR1A=n1?50026:54026;
  reset_count++;
  if(reset_count>=200)
    sbi(DDRC,5);
}

ISR(TIMER1_COMPA_vect)
{
  high_stop_rf();
}

ISR(TIMER1_COMPB_vect)
{
//  if(flag_TC1B){
    if(OCR1B==100){
      low_rf();
      trig();
      OCR1B=1100;
    }
    else if(OCR1B==1100){
      start_rf();
      sbi(TCCR1B,CS10); //Prescaler 64
      OCR1B=18600;
    }
    else{
      cbi(TCCR1B,CS10);
      OCR1B=100;
    }
//    Serial.println("T");
/*  }
  else{
    start_rf();
    flag_TC1B=1;
  }
  trig();*/
//  Serial.println(TCNT1);
}

ISR(TIMER2_COMPA_vect)
{
  if(n1)
    cbi(PORTB,4);
  else
    cbi(PORTB,3);
  cbi(TCCR2B,CS20);
  TCNT2=0;
}
/*
ISR(WDT_vect)
{
  Z=0;
}
*/
