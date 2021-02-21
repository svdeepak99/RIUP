#include<avr/io.h>
//#include<avr/interrupt.h>
//#include<util/delay.h>

#define sbi(X,Y) X|=(1<<Y)
#define cbi(X,Y) X&=~(1<<Y)
#define tog(X,Y) X^=(1<<Y)

long int i=0,t;
uint16_t TCNT0H=0;

#define TCNT0T (TCNT0H<<8)|TCNT0;

void rf_receive_init()
{
  Serial.begin(2000000);
  sbi(EIMSK,INT0);
  EICRA|=(1<<ISC00)|(1<<ISC01);     //Trigger at rise
  TIMSK0|=1;  //TOV interrut enable
  sei();
  sbi(TCCR0B,CS01);       //prescaler 8
}

int main()
{
  rf_receive_init();
  //Timer 1 PWM code
  sbi(DDRB,1);
  ICR1=40000;
  OCR1A=20000;
  TCCR1A|=(1<<WGM11)|(1<<COM1A1); //set at BOTTOM, clear on Compare Match
  TCCR1B|=(1<<WGM12)|(1<<WGM13)|(1<<CS11);  //Fast PWM with ICR1 as TOP , Prescaler 8
  while(1);
  return 0;
}

ISR(INT0_vect)
{
  if(i==5)
      TCNT1=0;
  t=TCNT0T;
  TCNT0=0;
  TCNT0H=0;
  if(t>270&&t<430){
    if(i==5)
      i=0;
    else
      i++;
  }
  else
    i=0;
}

ISR(TIMER0_OVF_vect)
{
  TCNT0H++;
}

