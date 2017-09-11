#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>

const int chaApin = 2;     
const int chaBpin =  3;     

volatile long int encoder = 0;
volatile int  cha=0;
volatile int  chb=0;
float angle = 0;
float dest_angle = 180.0;
float error_angle = 0;
float error,error_dt,t_0;
float error_prev = 0;
float t_m1 = 0;
float pid=0;
float Mspeed;
float kp = 11.2;
float kd = 0;
float p_der,d_der;
float time;
float start_time;

SoftwareSerial SWSerial(NOT_A_PIN, 11); // RX on no pin (unused), TX on pin 11 (to S1).
SabertoothSimplified ST(SWSerial); // Use SWSerial as the serial port.

float crnt_time()
{
  time=float (millis()-start_time)/1000;
  return(time);
  start_time = millis();
}

void motor()
{
  t_0 = crnt_time();  
  error_angle = dest_angle - angle;
 // Serial.print("angle----------------------------------------");Serial.print(angle);
 // Serial.println("t_0----------------------------------------");Serial.println(t_0);
  error_dt = (error_angle-error_prev)/(t_0-t_m1);
  error_prev=error_angle;
  t_m1 = t_0;
  p_der=kp*error_angle;
  pid = (kp*error_angle)+(kd*error_dt);
    //Serial.print("dest_angle:");Serial.print(dest_angle);
      //  Serial.print("angle:");Serial.print(angle);


 // Serial.print("error:");Serial.print(error_angle);
  //Serial.print("PID:");Serial.print(pid);

  delay(5);  
  int max_speed = 50;
  Mspeed = ((pid/90)*max_speed);


    if (Mspeed > max_speed)
  {
    Mspeed =max_speed;
  }
  else if (Mspeed < -max_speed)
  {
    Mspeed = -max_speed;
  }
  Serial.print("Mspeed");Serial.println(Mspeed);
  ST.motor(1, Mspeed);
}


void setup()
{
  
  Serial.begin(9600);
  Serial.println("CLEARDATA"); //clears up any data left from previous projects
  Serial.println("LABEL,Timer,t_0,angle,p_der,..."); //always write LABEL, so excel knows the next things will be the names of the columns (instead of Acolumn you could write Time for instance)
  Serial.println("RESETTIMER"); //resets timer to 0
  SWSerial.begin(9600);
  pinMode(chaApin, INPUT_PULLUP); 
  pinMode(chaBpin, INPUT_PULLUP); 
  attachInterrupt(0, CHA, RISING);
  attachInterrupt(1, CHB, RISING);
}
void loop()
{
  Serial.print("DATA,TIME,"); //writes the time in the first column A and the time since the measurements started in column B
  /*Serial.print(angle);
  Serial.print(t_0);
  Serial.print(error_angle);
  Serial.print(error_dt);
  Serial.print(p_der);
  Serial.println(d_der);
  delay(100); //add a delay

   */
    Serial.print(t_0);
 Serial.print(",");
 Serial.print(angle);
  Serial.print(",");
 Serial.print(p_der);
  Serial.print(",");


 Serial.println("...");
   angle = ((float(encoder)/400)*360);
//   Serial.print("encoder :   ");Serial.println(encoder);
 // Serial.println("current angle--------------");Serial.println(angle);
  motor();
}
void CHA()
{  
if(digitalRead(chaBpin) == 0)
{
  encoder++ ;
 // Serial.println(encoder);
}

}

void CHB()
{
if(digitalRead(chaApin) == 0)
{
  encoder-- ;
  //  Serial.print(encoder);

}
}

