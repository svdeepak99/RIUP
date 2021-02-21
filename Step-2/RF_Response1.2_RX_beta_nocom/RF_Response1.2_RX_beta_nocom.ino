#include<avr/io.h>
#include<avr/interrupt.h>
//#include<util/delay.h>

#define sbi(X,Y) X|=(1<<Y)
#define cbi(X,Y) X&=~(1<<Y)
#define tog(X,Y) X^=(1<<Y)

unsigned long int n=0,i=0,t;

inline void usart_send(unsigned int ch )
{
  while (UCSR0A != (UCSR0A | (1 << UDRE0))); //waiting for UDRE to become high - which means data is ready to be written
  UDR0 = ch;
}

void rf_receive_init()
{
  Serial.begin(2000000);
  sbi(EIMSK,INT0);
  EICRA|=(1<<ISC00)|(1<<ISC01);     //Trigger at rise
  sei();
  sbi(TCCR1B,CS11);       //prescaler 8
}

int main()
{//This code has best accuracy with antennas +/- 1 us max when measured at close range
 //Best results are obtained when Transmitter has an antenna and receiver does not have an antenna
  rf_receive_init();
  while(1);
  return 0;
}

ISR(INT0_vect)
{
  t=TCNT1;
  if(t>270&&t<430){
  TCNT1=0;
  n+=t;
  if(i==4)
    n=0;
  i++;
  if(i==69){
    n=n>>6;
    Serial.println(n);
    n=0;
    i=0;    
  }
  }
  else{
    TCNT1=0;
    i=0;
    n=0;
  }
}

