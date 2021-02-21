#include<avr/io.h>
#include<avr/interrupt.h>
#include<math.h>

//#include<util/delay.h>

#define sbi(X,Y) X|=(1<<Y)
#define cbi(X,Y) X&=~(1<<Y)
#define tog(X,Y) X^=(1<<Y)

class KalmanFilter
{
  public:
  
  float err_mea;
  float err_est;
  float KG;
  float est;
  float est_prev;
  float q;

  KalmanFilter(float em,float es,float qi,float esti)
  {
    err_mea = em;
    err_est = es;
    q = qi;
    est_prev=esti;
  }

  float updateEstimate(float mea)
  {
    KG = err_est / (err_est + err_mea);
    est= est_prev + KG * (mea - est_prev);
    err_est= (1.0 - KG) * err_est + q * fabs(est - est_prev);
    est_prev= est;

    return est;
  }
};

#define TOP 60000

uint16_t TCNT0H=0,t1,tu1,ti1,tf1,tu2,ti2,tf2;
uint8_t flag=2,n1=0;
float f1,f2,f1f,f2f,x,y,c,X,Y;//,AC;

#define TCNT0T (TCNT0H<<8)|TCNT0;

KalmanFilter t1f(1,1,0.01,195);
KalmanFilter t2f(1,1,0.01,190);

void rf_receive_init()
{
  cbi(DDRB,4);
  sbi(PCICR,PCIE0);
  sbi(PCMSK0,PCINT4);
  TIMSK0|=1;  //TOV interrut enable
  sei();
  sbi(TCCR0B,CS01);       //prescaler 8
}
void ultrasound_init()
{
  //Trig Part
  sbi(DDRB,3);  //pin 11
  sbi(TIMSK1,1);      //OCIEA
  //Echo Part
  sbi(EICRA,ISC00);   //Trigger at Rise & Fall
  sbi(EIMSK,INT0);
}
inline void trig()
{
  sbi(PORTB,3);
  OCR1A=TCNT1+20;
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
    f1=tu1*0.176*1.02;  //0.01478
    f2=tu2*0.176*1.02;
    if(f1<4500)
      f1f=f1;
    if(f2<4500)
      f2f=f2;
      
    f1f=t1f.updateEstimate(f1f);
    f2f=t2f.updateEstimate(f2f);

    c= (f1f+f2f) * (f1f-f2f); //f1f^2 - f2f^2
 //   BC= f2f*f2f - 2371600;
 //   AC= f1f*f1f-2592100;
 //   x= (219950 + c) * 0.000320821;
    x= (2640500 - c) * 0.000320821;
 //   y= sqrt( AC - x*x ) ;
 //   y= sqrt( BC*BC - ( BC - x )*( BC - x) );
 //   y= (220500 + c + 2200 * x) * 0.000454545;
    y= sqrt( f2f*f2f - 2371600 - x*x );

    X= (x-y)*0.7071 + 35;
    Y= (x+y)*0.7071 - 1100 +10;

    Serial.print(X);Serial.print('\t');Serial.println(Y);//Serial.print('\t');Serial.print(f1f);Serial.print('\t');Serial.println(f2f);
    _delay_ms(40);
  }
  return 0;
}

ISR(INT0_vect)
{
  if(n1){
    if(flag==0)
    {
      ti2=TCNT1;
      flag=1;
    }
    else if(flag==1)
    {
      tf2=TCNT1;
      flag=2;
      tu2=tf2-ti2+635;
    //    Serial.println(n1);
    }
    //  Serial.println(n1);
  }
  else{
    if(flag==0)
    {
      ti1=TCNT1;
      flag=1;
    }
    else if(flag==1)
    {
      tf1=TCNT1;
      flag=2;
      tu1=tf1-ti1+635;
    //    Serial.println(n1);
    }
  }
 // Serial.println("Echo");
}

ISR(PCINT0_vect)
{// rf time start
  if( (PINB&0b00010000) == 0){
    
  t1=TCNT0T;
  if(t1>4000)
  {
    TCNT1=0;
    flag=0;
    n1=(t1>8000)?0:1;
    trig();
//    Serial.println("Trig");
  }
  TCNT0=0;
  TCNT0H=0;
  
  }
}

ISR(TIMER0_OVF_vect)
{
  TCNT0H++;
}

ISR(TIMER1_CAPT_vect)
{
//  if(flag!=2){
  flag=2;
//  tu1=0;
//  }
}

ISR(TIMER1_COMPA_vect)
{
  cbi(PORTB,3);
}

