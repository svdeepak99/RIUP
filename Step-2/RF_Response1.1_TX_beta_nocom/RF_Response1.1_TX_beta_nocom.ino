#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>

#define sbi(X,Y) X|=(1<<Y)
#define cbi(X,Y) X&=~(1<<Y)
#define tog(X,Y) X^=(1<<Y)

int n;

void rf_pwm_init()
{
  sbi(DDRD,5);
  TCCR0A|=(1<<WGM00)|(1<<COM0B1); //Set on down counting, Clear on up counting      
  sbi(TCCR0B,WGM02);              //Phase correct PWM with OCR0A as top
  OCR0A=100;
  OCR0B=50;
  TCNT0=100;
  sei();
  sbi(TCCR0B,CS01);       //prescaler 8
}
inline void on_pwm()
{
//  sbi(TCCR0B,CS01);
  OCR0A=170;
  OCR0B=85;
  TCNT0=0;
  TIMSK0|=1;      //Enable TOV (at 0) interrupt
}
inline void off_pwm()
{
  cbi(TIMSK0,0);
  OCR0A=100;
  OCR0B=50;
  n=0;
}
int main()
{
  rf_pwm_init();
  while(1)
  {
    on_pwm();
    _delay_ms(5000);
  }
  return 0;
}

ISR(TIMER0_OVF_vect)
{
  n++;
  if(n==70)
    off_pwm();
}

