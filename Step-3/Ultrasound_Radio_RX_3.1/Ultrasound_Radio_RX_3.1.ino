#include<avr/io.h>
#include<avr/interrupt.h>
//#include<util/delay.h>

#define sbi(X,Y) X|=(1<<Y)
#define cbi(X,Y) X&=~(1<<Y)
#define tog(X,Y) X^=(1<<Y)

#define TOP 60000

uint16_t TCNT0H=0,tu,t1,ti,tf;
uint8_t flag=2;
float f,f1;

#define TCNT0T (TCNT0H<<8)|TCNT0;

void rf_receive_init()
{
  sbi(EIMSK,INT1);
  EICRA|=(1<<ISC11);     //Trigger at fall
  TIMSK0|=1;  //TOV interrut enable
  sei();
  sbi(TCCR0B,CS01);       //prescaler 8
}
void ultrasound_init()
{
  //Trig Part
  sbi(DDRB,3);  //OC2A
  TCCR2A|=(1<<WGM20)|(1<<WGM21);    //Fast PWM
  OCR2B=235;
  OCR2A=160;
  sbi(TIMSK2,2);    //OCIEB
  TCNT2=0;
  //Echo Part
  sbi(EICRA,ISC00);   //Trigger at Rise & Fall
  sbi(EIMSK,INT0);
}
inline void trig()
{
  sbi(TCCR2A,COM2A1);             //OCR2A - non inverting mode
  sbi(TCCR2B,CS20);
}
void TCNT1_init()
{
  ICR1=TOP;
  TCCR1B|=(1<<WGM12)|(1<<WGM13); //CTC with ICR1 as TOP
  TIMSK1|=(1<<5); //ICIE Interrupt Enable
  sbi(TCCR1B,CS11);   //Prescaler 8
}

int main()
{
  Serial.begin(2000000);
  ultrasound_init();
  TCNT1_init();
  rf_receive_init();
  while(1)
  {
    f=0.01478*tu+10;    //0.017
//    if(f>140&&f<260)
      f1=f;
      Serial.println((int)f1);
    _delay_ms(30);
  }
  return 0;
}

ISR(INT0_vect)
{
  if(flag==0)
  {
    ti=TCNT1;
    flag=1;
  }
  else if(flag==1)
  {
    tf=TCNT1;
    flag=2;
    tu=tf-ti+635;
  }
 // Serial.println("Echo");
}

ISR(INT1_vect)
{// rf time start
  t1=TCNT0T;
  if(t1>5000)
  {
//    TCNT1=635;
    trig();
    flag=0;
//    Serial.println("Trig");
  }
  TCNT0=0;
  TCNT0H=0;
}

ISR(TIMER0_OVF_vect)
{
  TCNT0H++;
}

ISR(TIMER1_CAPT_vect)
{
//  if(flag!=2){
  flag=2;
//  tu=0;
//  }
}

ISR(TIMER2_COMPB_vect)
{
  cbi(TCCR2B,CS20);
  cbi(TCCR2A,COM2A1);
  TCNT2=0;
}

