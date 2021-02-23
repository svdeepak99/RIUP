import processing.serial.*;

Serial myPort;

float s1,s2,s3,s4;

void setup()
{
  size(600,600);
  background(0);
  stroke(255);
  strokeWeight(10);

//  String portName = Serial.list()[0];
  myPort = new Serial(this, "COM5" , 115200);
  myPort.bufferUntil('\n');
}
void draw()
{
    background(0);
 //   line(s1/2+300,s2/2+300,s3/2+300,s4/2+300);
    point(s1/2+300,s2/2+300);
}

void serialEvent (Serial myPort)
{
//  background(0);
  s1=float(myPort.readStringUntil('\t'));
  s2=float(myPort.readStringUntil('\n'));
 // s3=float(myPort.readStringUntil('\t'));
 // s4=float(myPort.readStringUntil('\n'));
 // myPort.bufferUntil('\n');
//  point(s1,s2);
}
