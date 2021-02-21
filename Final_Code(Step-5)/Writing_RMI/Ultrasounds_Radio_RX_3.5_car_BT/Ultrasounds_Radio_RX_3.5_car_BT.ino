#include<avr/io.h>
#include<avr/interrupt.h>
#include<math.h>

#include<util/delay.h>

#define sbi(X,Y) X|=(1<<Y)
#define cbi(X,Y) X&=~(1<<Y)
#define tog(X,Y) X^=(1<<Y)

class Car
{
  public:

  Car()
  {
    sbi(DDRD,4);  //IN1
    sbi(DDRD,7);  //IN2
    sbi(DDRB,0);  //IN3
    sbi(DDRB,2);  //IN4
    sbi(DDRD,6);  //EN1 - OC0A
    sbi(DDRD,5);  //EN2 - OC0B
    TCCR0A|=(1<<WGM01)|(1<<WGM00)|(1<<COM0A1)|(1<<COM0B1);//    //Fast PWM TOP - 0xFF
    TCCR0B|=(1<<CS01)|(1<<CS00);                 //Prescaler 256
  }
  
  void forward(uint8_t n)
  {
    OCR0A=n;
    OCR0B=n;
    sbi(PORTD,4);
    cbi(PORTD,7);
    sbi(PORTB,0);
    cbi(PORTB,2);
  }

  void backward(uint8_t n)
  {
    OCR0A=n;
    OCR0B=n;
    cbi(PORTD,4);
    sbi(PORTD,7);
    cbi(PORTB,0);
    sbi(PORTB,2);
  }

  void right(uint8_t n)
  {
    OCR0A=n;
    OCR0B=n;
    sbi(PORTD,4);
    cbi(PORTD,7);
    cbi(PORTB,0);
    sbi(PORTB,2);
  }

  void left(uint8_t n)
  {
    OCR0A=n;
    OCR0B=n;
    cbi(PORTD,4);
    sbi(PORTD,7);
    sbi(PORTB,0);
    cbi(PORTB,2);
  }

  void fr(uint8_t n)
  {
    OCR0A=n;
    sbi(PORTD,4);
    cbi(PORTD,7);
    cbi(PORTB,0);
    cbi(PORTB,2);
  }

  void fl(uint8_t n)
  {
    OCR0B=n;
    cbi(PORTD,4);
    cbi(PORTD,7);
    sbi(PORTB,0);
    cbi(PORTB,2);
  }

  void br(uint8_t n)
  {
    OCR0B=n;
    cbi(PORTD,4);
    cbi(PORTD,7);
    cbi(PORTB,0);
    sbi(PORTB,2);
  }

  void bl(uint8_t n)
  {
    OCR0A=n;
    cbi(PORTD,4);
    sbi(PORTD,7);
    cbi(PORTB,0);
    cbi(PORTB,2);
  }
  
  void halt()
  {
    cbi(PORTD,4);
    cbi(PORTD,7);
    cbi(PORTB,0);
    cbi(PORTB,2);
  }
};
Car car;

class KalmanFilter
{
  public:
  
  float err_mea;
  float err_est;
  float KG;
  float est;
  float est_prev;
  float q;
  float x;
  int n=0,nr=0;

  KalmanFilter(float em,float es,float qi,float esti)
  {
    err_mea = em;
    err_est = es;
    q = qi;
    est_prev=esti;
  }

  float updateEstimate(float mea)
  {
    x=abs(est-mea);
    if(n<10)
      n++;
    if(x<50||n<10||nr==10){
    nr=0;
    KG = err_est / (err_est + err_mea);
    est= est_prev + KG * (mea - est_prev);
    err_est= (1.0 - KG) * err_est + q * fabs(est-est_prev);
    est_prev= est;
    }
    else
      nr++;
    return est;
  }
};

#define TOP 60000

uint16_t TCNT2H=0,t1,tu1,ti1,tf1,tu2,ti2,tf2,_tu1,_tu2,_ti1,_ti2,_tf1,_tf2,n_init=0;
uint8_t flag=2,n1=0,_flag=2;
float f1,f2,f1f,f2f,x,y,c,X,Y,_f1f,_f2f,_X,_Y;//,AC;

class trackmoves
{
  private:
  
  enum direction {FRONT,BACK};
  enum processrunning {FREE,ROTATING,GOTO,HALT};
  
  float returnangle,angdiff,m,c,den,dsign;
  direction dir=FRONT;
  processrunning process=FREE;
  
  public:
  
  float Xavg,Yavg;
  float Xd,Yd;
  float targetangle;
  float currentangle;
  float distance;
  float points[15][3]={ {-200,-200,FRONT} , {-200,200,FRONT} , {-100,100,FRONT} , {-200,0,FRONT} , {-100,-200,FRONT} , {-50,-200,FRONT} , {-50,200,FRONT} , {0,0,FRONT} , {50,200,FRONT} , {50,-200,FRONT} , {250,-200,FRONT} , {175,-200,BACK} , {175,200,FRONT} , {75,200,FRONT} , {250,200,BACK} };
  int n=15,i=0;

  float angle(float x1,float y1,float x2,float y2)
  {
    if((x2-x1)!=0){
      returnangle= atan( (y2-y1)/(x2-x1) );
      
      if(returnangle>=0&&y2<y1)
        returnangle+=3.145926;
      else if(returnangle<0)
        returnangle+=(y2<y1)?6.283185:3.141592;
    }
    else
      returnangle= (y2-y1)>=0?1.570796:4.712389;

    return returnangle;
  }

  void rotate()
  {
 //   Serial.print('\t');Serial.println(angdiff);
    if( angdiff < 0.04 )
    {
      car.halt();
      process=GOTO;
      if(Yavg!=Yd){
        m=(Xd-Xavg)/(Yavg-Yd);
        c=m*Xd - Yd;
        den=sqrt( m*m + 1 );

        dsign=( (Yavg - m*Xavg + c)>=0 )?1:-1;
      }
      else{
        c=Xd;
        den=0;
        
        dsign= ( (Xavg - c)>=0)?1:-1;
      }
    }
    else if( (targetangle>currentangle && angdiff<3.141592) || (currentangle>targetangle && angdiff>3.141592) ){
      car.left(80);
    }
    else{
      car.right(80);
    }
  }

  void GoTo()
  {
 //   distance=sqrt( (Xd-Xavg)*(Xd-Xavg) + (Yd-Yavg)*(Yd-Yavg) );
    //Perpendicular distance from end perpendicular line
    if(den!=0)
      distance= ((Yavg - m*Xavg + c)*dsign) / den;
    else
      distance= (Xavg - c)*dsign;
//    Serial.println(distance);
    if(distance>5){
      if(dir==FRONT){
       if(angdiff<0.02)
          car.forward(80);
        else if( (targetangle>currentangle && angdiff<3.141592) || (currentangle>targetangle && angdiff>3.141592) )
          car.fl(80);
        else
          car.fr(80);
      }
      else{
        if(angdiff<0.02)
          car.backward(80);
        else if( (targetangle>currentangle && angdiff<3.141592) || (currentangle>targetangle && angdiff>3.141592) )
          car.bl(80);
        else
          car.br(80);
      }
    }
    else{
      car.halt();
//      Serial.println(i);
      process=(i<n)?FREE:HALT;
    }
  }
  
  void update()
  {
    if(process!=HALT){
      
      Xavg=(X+_X)/2;
      Yavg=(Y+_Y)/2;
      if(dir==FRONT)
        currentangle= angle(X, Y, _X, _Y);
      else
        currentangle= angle(_X, _Y, X, Y);
      angdiff= abs(targetangle-currentangle);
      
      if(process==FREE){
        //target angle calc
        Xd=points[i][0];
        Yd=points[i][1];
        dir=(int)points[i][2];
        i+=1;
   //     Serial.print('\t');Serial.print(Xd);Serial.print('\t');Serial.println(Yd);
        targetangle= angle(Xavg , Yavg, Xd, Yd);
        angdiff= abs(targetangle-currentangle);
        process=ROTATING;
      }

      if(process==ROTATING)
        rotate();
      if(process==GOTO)
        GoTo();
    }
  }
};

trackmoves track;

#define TCNT2T (TCNT2H<<8)|TCNT2;

KalmanFilter t1f(1,1,0.01,195),t2f(1,1,0.01,190),_t1f(1,1,0.01,190),_t2f(1,1,0.01,190);

void rf_receive_init()
{
  cbi(DDRB,4);
  sbi(PCICR,PCIE0);
  sbi(PCMSK0,PCINT4);
  TIMSK2|=1;  //TOV interrut enable
  sei();
  sbi(TCCR2B,CS21);       //prescaler 8
}
void ultrasound_init()
{
  //Trig Part
  sbi(DDRB,1);  //pin 9
  sbi(DDRB,3);  //pin 11
  sbi(TIMSK1,1);      //OCIEA
  
  //Echo Part
  sbi(EICRA,ISC00);   //Trigger at Rise & Fall
  sbi(EIMSK,INT0);

  sbi(EICRA,ISC10);
  sbi(EIMSK,INT1);
}
inline void trig()
{
  sbi(PORTB,1);
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
  Serial.begin(115200);
  ultrasound_init();
  TCNT1_init();
  rf_receive_init();
  
//  car.right(100);
  while(1)
  {//Receiver 1 code
    f1=tu1*0.176*1.02;  //0.01478
    f2=tu2*0.176*1.02;
    if(f1<4500)
      f1f=f1;
    if(f2<4500)
      f2f=f2;
      
    f1f=t1f.updateEstimate(f1f);
    f2f=t2f.updateEstimate(f2f);

    c= (f1f+f2f) * (f1f-f2f); //f1f^2 - f2f^2
    x= (2640500 - c) * 0.000320821;
    y= sqrt( f2f*f2f - 2371600 - x*x );

    X= (x-y)*0.7071 + 35 -20 +20;
    Y= (x+y)*0.7071 - 1100 +10 +55 -20;

    //Receiver 2 Code
    f1=_tu1*0.176*1.02;  //0.01478
    f2=_tu2*0.176*1.02;
    if(f1<4500)
      _f1f=f1;
    if(f2<4500)
      _f2f=f2;
      
    _f1f=_t1f.updateEstimate(_f1f);
    _f2f=_t2f.updateEstimate(_f2f);

    c= (_f1f+_f2f) * (_f1f-_f2f); //_f1f^2 - _f2f^2
    x= (2640500 - c) * 0.000320821;
    y= sqrt( _f2f*_f2f - 2371600 - x*x );

    _X= (x-y)*0.7071 + 35 -20 +20;
    _Y= (x+y)*0.7071 - 1100 +10 +55 -20;
    
    Serial.print(X);Serial.print('\t');Serial.println(Y);Serial.print('\t');Serial.print(_X);Serial.print('\t');Serial.println(_Y);//Serial.print('\t');Serial.println(t1f.x);//Serial.print('\t');Serial.print(f1f);Serial.print('\t');Serial.println(f2f);
    
    if(n_init>100)
      track.update();
    else
      n_init++;
    
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
    }
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

ISR(INT1_vect)
{
  if(n1){
    if(_flag==0)
    {
      _ti2=TCNT1;
      _flag=1;
    }
    else if(_flag==1)
    {
      _tf2=TCNT1;
      _flag=2;
      _tu2=_tf2-_ti2+635;
//      Serial.println(_tu2);
    }
  }
  else{
    if(_flag==0)
    {
      _ti1=TCNT1;
      _flag=1;
    }
    else if(_flag==1)
    {
      _tf1=TCNT1;
      _flag=2;
      _tu1=_tf1-_ti1+635;
    //    Serial.println(n1);
    }
  }
 // Serial.println("Echo");
}

ISR(PCINT0_vect)
{// rf time start
  if( (PINB&0b00010000) ==0){
    
  t1=TCNT2T;
  if(t1>4000)
  {
    TCNT1=0;
    flag=0;
    _flag=0;
    n1=(t1>8000)?0:1;
    trig();
  }
  TCNT2=0;
  TCNT2H=0;
  
  }
}

ISR(TIMER2_OVF_vect)
{
  TCNT2H++;
}

ISR(TIMER1_CAPT_vect)
{
  flag=2;
  _flag=2;
}

ISR(TIMER1_COMPA_vect)
{
  cbi(PORTB,1);
  cbi(PORTB,3);
}

