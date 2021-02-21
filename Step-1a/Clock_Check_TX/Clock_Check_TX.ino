#include<avr/io.h>
//#include<avr/interrupt.h>
//#include<util/delay.h>

#define sbi(X,Y) X|=(1<<Y)
#define cbi(X,Y) X&=~(1<<Y)
#define tog(X,Y) X^=(1<<Y)

int main()
{
  //Timer 1 PWM code
  sbi(DDRB,1);
  ICR1=40000;
  OCR1A=20000;
  TCCR1A|=(1<<WGM11)|(1<<COM1A1); //set at BOTTOM, clear on Compare Match
  TCCR1B|=(1<<WGM12)|(1<<WGM13)|(1<<CS11);  //Fast PWM with ICR1 as TOP , Prescaler 8
  while(1);
  return 0;
}

