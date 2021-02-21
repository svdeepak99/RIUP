#include<avr/io.h>
#include<avr/interrupt.h>
//#include<util/delay.h>

#define sbi(X,Y) X|=(1<<Y)
#define cbi(X,Y) X&=~(1<<Y)
#define tog(X,Y) X^=(1<<Y)

int n=100;
unsigned int t;

int main()
{
  Serial.begin(1000000);
  //Timer 1 CTC code
  ICR1=39953;
  TCCR1B|=(1<<WGM12)|(1<<WGM13)|(1<<CS11); //CTC with ICR1 as TOP , Prescaler 8
  //INT1 code
  sbi(EIMSK,INT1);
  EICRA|=(1<<ISC10)|(1<<ISC11); //Rising Edge
  sei();
  while(1);
  return 0;
}

ISR(INT1_vect)
{
  if(n==100){
    TCNT1=0;
    t=TCNT1;
    n=0;
  }
  else
    t=TCNT1;
  Serial.print(n);Serial.print(") ");Serial.println(t);
  n++;
}

