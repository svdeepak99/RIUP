#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>

#define sbi(X,Y) X|=(1<<Y)
#define cbi(X,Y) X&=~(1<<Y)
#define tog(X,Y) X^=(1<<Y)

#define TOP 39951

int n1=25,t2,i,flag;
unsigned int t1;
int n=0;

void rf_pwm_init()
{
  sbi(DDRD,5);
  TCCR0A|=(1<<WGM00)|(1<<COM0B1); //Set on down counting, Clear on up counting      
  sbi(TCCR0B,WGM02);              //Phase correct PWM with OCR0A as top
  OCR0A=100;
  OCR0B=50;
  TCNT0=100;
//  sei();
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
  Serial.begin(2000000);
  //Timer 1 CTC code
  ICR1=TOP;
  TCCR1B|=(1<<WGM12)|(1<<WGM13)|(1<<CS11); //CTC with ICR1 as TOP , Prescaler 8
  TIMSK1|=(1<<1); //OCIEA Interrupt Enable
  OCR1A=TOP-100;
  //INT1 code
  sbi(EIMSK,INT1);
  EICRA|=(1<<ISC10)|(1<<ISC11); //Rising Edge
  rf_pwm_init();
  sei();
  while(1){
    while(flag)
      _delay_ms(3);
    Serial.print(n1);Serial.print(") ");Serial.println(t2);
    flag=1;
  }
  return 0;
}

ISR(INT1_vect)
{
  if(n1==25){
    on_pwm();
    n1=0;
  }
  t1=TCNT1;
  t2=(t1>30000)?t1-TOP:t1;
  if(flag)
    flag=0;
  n1++;
}

ISR(TIMER0_OVF_vect)
{
  if(n==6){
    TCNT1=-82;
    off_pwm();
  }
  else
    n++;
}

ISR(TIMER1_COMPA_vect)
{//The exact required TOP is 39950.5 .So is this simple technique :-)
  ICR1=(ICR1==39950)?39951:39950;
}

