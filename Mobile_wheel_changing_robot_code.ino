#include <Stepper.h>

const int echo_pin = 2;//echo pin for first ultrasonic sensor
const int trig_pin = 3;// trigger pin for first ultrasonic sensor
const int metalsensor= A8;//capacitive proximity sensor
int wheelnutdistance;//distance to wheel nut
int metalsensedistance;//sensed distance by capacitive proximity sensor
long duration;
long distance;//distance measured by first ultrasonic sensor
int sensednut;//logic to detect presence of nut
int sensedstud;//logic to detect presence of nut during closing
//2nd ultrasonic sensor
const int echo_pin1 = A14;
const int trig_pin1 = A15;
long duration1;
long distance1;
//3rd ultrasonic sensor
const int echo_pin2 = A12;
const int trig_pin2 = A13;
long duration2;
long distance2;

const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution

int NUTRUNNERENA= 6 ; // Motor A connections
int NUTRUNNERIN1= 46;
int NUTRUNNERIN2= 47;

//Motor driver 1 connection ( FRONT WHEELS)
int FRONTWHEELSENA = 12;  // Motor 1 connections
int FRONTWHEELSIN1 = 22;
int FRONTWHEELSIN2 = 23;

int FRONTWHEELSENB = 13;  // Motor 2 connections
int FRONTWHEELSIN3 = 24;
int FRONTWHEELSIN4 = 25;

//Motor driver 2 connection ( BACK WHEELS)
int REARWHEELSENA = 10;  // Motor 1 connections
int REARWHEELSIN1 = 26;
int REARWHEELSIN2 = 27;

int REARWHEELSENB = 11;  // Motor 2 connections
int REARWHEELSIN3 = 28;
int REARWHEELSIN4 = 29;

int yAxis_pin = A6; // Joysticks Y-axis
int xAxis_pin = A7; // Joysticks X-axis



int MotorSpeed1 = 0; //Motor speed values to start at zero
int MotorSpeed2 = 0;
int MotorSpeed3 = 0; 
int MotorSpeed4 = 0;
//force feedback parameters
int forcefeedback=A0;
int force;
//initialize the number of nuts removed or replaced
int nuts=0;
int studs=0;
//logic for checking
int done=0;//logic to ensure the X gantry remains in place
// for your motor

// initialize the stepper library on pins 
Stepper myStepperX(stepsPerRevolution, 34, 35, 36, 37);// X gantry
Stepper myStepperY(stepsPerRevolution, 38, 39, 40, 41);//Y gantry
Stepper myStepperdisc(stepsPerRevolution, 42, 43, 44, 45);//location disc for nut runner
Stepper mySteppergrip(stepsPerRevolution, 30, 31, 32, 33);//gripper location
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  myStepperX.setSpeed(200);//speed in rpm
  myStepperY.setSpeed(200);//speed in rpm
myStepperdisc.setSpeed(100);//speed in rpm
//pin definitions
  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);

  pinMode(trig_pin1, OUTPUT);
  pinMode(echo_pin1, INPUT);

  pinMode(trig_pin2, OUTPUT);
  pinMode(echo_pin2, INPUT);
  pinMode(metalsensor,INPUT);

  pinMode(yAxis_pin, INPUT);
  pinMode(xAxis_pin, INPUT);
 
  // Set all the motor control pins to outputs
  pinMode(FRONTWHEELSENA, OUTPUT);
  pinMode(FRONTWHEELSENB, OUTPUT);
  pinMode(REARWHEELSENA, OUTPUT);
  pinMode(REARWHEELSENB, OUTPUT);
  
  pinMode(FRONTWHEELSIN1, OUTPUT);
  pinMode(FRONTWHEELSIN2, OUTPUT);
  pinMode(FRONTWHEELSIN3, OUTPUT);
  pinMode(FRONTWHEELSIN4, OUTPUT);

  pinMode(REARWHEELSIN1, OUTPUT);
  pinMode(REARWHEELSIN2, OUTPUT);
  pinMode(REARWHEELSIN3, OUTPUT);
  pinMode(REARWHEELSIN4, OUTPUT);

   pinMode(NUTRUNNERENA, OUTPUT);
  
  pinMode(NUTRUNNERIN1, OUTPUT);
  pinMode(NUTRUNNERIN2, OUTPUT);
  //define forcefeedback sensor as input
  pinMode(forcefeedback,INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  int xAxis = analogRead(xAxis_pin); // Read Joysticks X-axis
   int yAxis = analogRead(yAxis_pin); // Read Joysticks Y-axis
  
   // If joystick stays in middle the motors are not moving
if(xAxis > 470 && xAxis <550 && yAxis > 470 && yAxis <550){Stop();}

if (xAxis > 470 && xAxis <550){    
// Y-axis used for left and right control
if (yAxis < 470){turnRight();
// Convert the declining Y-axis readings for going backward from 470 to 0 into 0 to 255 value for the PWM signal for increasing the motor speed  
MotorSpeed1 = map(yAxis, 470, 0, 0, 255);
MotorSpeed2 = map(yAxis, 470, 0, 0, 255); 
MotorSpeed3 = map(yAxis, 470, 0, 0, 255);
MotorSpeed4 = map(yAxis, 470, 0, 0, 255);  
}

if (yAxis > 550) {turnLeft();
// Convert the increasing Y-axis readings for going forward from 550 to 1023 into 0 to 255 value for the PWM signal for increasing the motor speed      
MotorSpeed1 = map(yAxis, 550, 1023, 0, 255);
MotorSpeed2 = map(yAxis, 550, 1023, 0, 255);
MotorSpeed3 = map(yAxis, 550, 1023, 0, 255);
MotorSpeed4 = map(yAxis, 550, 1023, 0, 255); 
}

}else{

if (yAxis > 470 && yAxis <550){   
// X-axis used for forward and backward control  
if (xAxis < 470){forward();}
if (xAxis > 550){backward();}

if (xAxis < 470){
// Convert the declining X-axis readings for going backward from 470 to 0 into 0 to 255 value for the PWM signal for increasing the motor speed  
MotorSpeed1 = map(xAxis, 470, 0, 0, 255);
MotorSpeed2 = map(xAxis, 470, 0, 0, 255); 
MotorSpeed3 = map(yAxis, 470, 0, 0, 255);
MotorSpeed4 = map(yAxis, 470, 0, 0, 255);
}

if (xAxis > 550){
// Convert the increasing X-axis readings for going forward from 550 to 1023 into 0 to 255 value for the PWM signal for increasing the motor speed      
MotorSpeed1 = map(xAxis, 550, 1023, 0, 255);
MotorSpeed2 = map(xAxis, 550, 1023, 0, 255);
MotorSpeed3 = map(yAxis, 550, 1023, 0, 255);
MotorSpeed4 = map(yAxis, 550, 1023, 0, 255); 
 }
 
}else{

if(xAxis < 470){forward();}
if(xAxis > 550){backward();}

if(yAxis < 470){
    // Convert the declining Y-axis readings from 470 to 0 into increasing 0 to 255 value
    int yMapped = map(yAxis, 470, 0, 0, 255);
    
    // Move to left - decrease left motor speed, increase right motor speed
    MotorSpeed1 = MotorSpeed1 + yMapped;
    MotorSpeed2 = MotorSpeed2 - yMapped;
    MotorSpeed3 = MotorSpeed3 + yMapped;
    MotorSpeed4 = MotorSpeed4 - yMapped;
    
    // Confine the range from 0 to 255
    if(MotorSpeed1 > 255) {MotorSpeed1 = 255;}
    if(MotorSpeed2 > 70){MotorSpeed2 = 70;}
    if(MotorSpeed3 > 255) {MotorSpeed3 = 255;}
    if(MotorSpeed4 > 70){MotorSpeed4 = 70;}
}
 
if (yAxis > 550){
    // Convert the increasing Y-axis readings from 550 to 1023 into 0 to 255 value
    int yMapped = map(yAxis, 550, 1023, 0, 255);
    
    // Move right - decrease right motor speed, increase left motor speed
    MotorSpeed1 = MotorSpeed1 - yMapped;
    MotorSpeed2 = MotorSpeed2 + yMapped;
    MotorSpeed3 = MotorSpeed3 - yMapped;
    MotorSpeed4 = MotorSpeed4 + yMapped;
    
    // Confine the range from 0 to 255
    if(MotorSpeed1 < 70){MotorSpeed1 = 70;}
    if(MotorSpeed2 > 255){MotorSpeed2 = 255;}
    if(MotorSpeed3 < 70){MotorSpeed3 = 70;}
    if(MotorSpeed4 > 255){MotorSpeed4 = 255;}
  }
 } 
}

// Prevent buzzing at low speeds (Adjust according to your motors. My motors couldn't start moving if PWM value was below value of 70)
if(MotorSpeed1 < 70){MotorSpeed1 = 0;}
if(MotorSpeed2 < 70){MotorSpeed2 = 0;}
if(MotorSpeed3 < 70){MotorSpeed3 = 0;}
if(MotorSpeed4 < 70){MotorSpeed4 = 0;}
  
analogWrite(FRONTWHEELSENA, MotorSpeed1); // Send PWM signal to motor 1
analogWrite(FRONTWHEELSENB, MotorSpeed2); // Send PWM signal to motor 2
analogWrite(REARWHEELSENA, MotorSpeed3); // Send PWM signal to motor 1
analogWrite(REARWHEELSENB, MotorSpeed4); // Send PWM signal to motor 2

delay(10);

//read map distance to wheel nut since capacitive sensor has a maximum sensing distance of 10mm
  wheelnutdistance=analogRead(metalsensor);
  wheelnutdistance=map(wheelnutdistance,0,1023,0,10);
  metalsensedistance=analogRead(metalsensor);
  metalsensedistance=map(metalsensedistance,0,1023,0,10);
  //running distance detection by sending a ping and measuring time taken to receive the ping back for the 3 ultrasonic sensors
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);

  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);


  duration = pulseIn(echo_pin, HIGH);
  distance = duration * (0.034 / 2);  //distance in cm
  digitalWrite(trig_pin1, LOW);
    delayMicroseconds(2);

    digitalWrite(trig_pin1, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_pin1, LOW);


    duration1 = pulseIn(echo_pin1, HIGH);
    distance1 = duration1 * (0.034 / 2);  //distance in cm

    digitalWrite(trig_pin2, LOW);
    delayMicroseconds(2);

    digitalWrite(trig_pin2, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_pin2, LOW);


    duration2 = pulseIn(echo_pin2, HIGH);
    distance2 = duration2 * (0.034 / 2);  //distance in mm
delay(100);
//just for computer simulation 
Serial.print("distance:");
Serial.println(distance);
Serial.print("distance1:");
Serial.println(distance1);
Serial.print("distance2:");
Serial.println(distance2);

//if gripper is far from the wheel, move it to 235mm
  if (distance >= 23.5 && done==0 ) {
    myStepperX.step(1);
    myStepperY.step(0);
    myStepperdisc.step(0);
    
  } 
//if already within 230mm
  else {
    
    done=1;//ensure X gantry doesnt move
    myStepperX.step(0);// stop stepper motor in X
    
    //if gripper is lower, raise it
  if(distance1<distance2)
    {
      myStepperY.setSpeed(100);
      myStepperY.step(1);
      myStepperdisc.step(0);
    }
    //if gripper is higher, lower it
     if(distance2<distance1)
    {
      myStepperY.setSpeed(100);
      myStepperY.step(-1);
      myStepperdisc.step(0);
    }

    if(distance1 == distance2) 
    {
      myStepperY.step(0);
     myStepperdisc.step(1);
      
      if(metalsensedistance==10)
      //if lug nut detected, stop all gantry motors, locating disc motor
      {
        myStepperdisc.step(0);
        myStepperX.step(0);
    myStepperY.step(0);
    sensednut=1;// logic for a nut has been detected
      }
      else
      {
     
       sensednut=0;
       //no nut detected
      }
      while(sensednut==0)
      {
        //as long as there is no nut detected, just keep running disc
        myStepperdisc.step(1);
      }
      restart://goto location
    if(sensednut==1)//if nut detected
    
    {
 analogWrite(NUTRUNNERENA, 255); // Set motors to maximum speed
                                   // For PWM maximum possible values are 0 to 255
  digitalWrite(NUTRUNNERIN1, HIGH);  // Turn on nut runner to extend shaft
  digitalWrite(NUTRUNNERIN2, LOW);
  delay(5000);// delay for 5s to ensure full extension
  digitalWrite(NUTRUNNERIN1, LOW);  // Turn off nutrunner on motor driver 1
  digitalWrite(NUTRUNNERIN2, LOW);
  delay(5000);
  //map and read for force
  force=analogRead(forcefeedback);
  force=map(force,0,1023,0,1200);
  // check if force feedback is upto half
if(force>>0 && force<=600)
{
  //keep unlocking
  analogWrite(NUTRUNNERENA, 100); // Set motors to fraction speed to protect threads
                                  
  digitalWrite(NUTRUNNERIN1,LOW);  
  digitalWrite(NUTRUNNERIN2, HIGH);
}
else if(force>>600)//if greater than half the force experienced
{
  analogWrite(NUTRUNNERENA, 20); // Set motors to low speed for retraction
                                 //retract shaft
  digitalWrite(NUTRUNNERIN1, LOW);  //
  digitalWrite(NUTRUNNERIN2, HIGH);
  delay(5000);//delay
  analogWrite(NUTRUNNERENA, 0); 
                                //stop motor
  digitalWrite(NUTRUNNERIN1, LOW); 
  digitalWrite(NUTRUNNERIN2, LOW);
  delay(5000);
  goto restart;//restart nutrunning process
}
if(force-- && force==0)//if force is decreasing and reaches 0 ,shows nut has been unlocked
{
  analogWrite(NUTRUNNERENA, 255); // Set motor to maximum speed
                                   // retract
  digitalWrite(NUTRUNNERIN1, LOW);  //
  digitalWrite(NUTRUNNERIN2, HIGH);
  delay(5000);
  analogWrite(NUTRUNNERENA, 0); // Stop
                                 
  digitalWrite(NUTRUNNERIN1, LOW);  // Turn off nutrunner
  digitalWrite(NUTRUNNERIN2, LOW);
  nuts++;//increment number of nuts removed
  delay(10000);//delay 10s to give time for nut removal
  sensednut=0;// logic to start disc
}
 if (nuts==5)//if all nuts have been removed, stop nut runner
 {
   analogWrite(NUTRUNNERENA, 0);
                                  
  digitalWrite(NUTRUNNERIN1, LOW); 
  digitalWrite(NUTRUNNERIN2, LOW);
delay(10000);
while(distance1>0 && distance2>0)// as long as the distance between the gripping teeth is not 0, run gripper till it grips
{
  mySteppergrip.step(1);
}
// once gripped, return X gantry backwards to remove
  myStepperdisc.step(0);
  myStepperY.step(0);
  myStepperX.step(-200);
  delay(5000);

  myStepperX.step(100);
//maybe put a pushbutton to start closing lugnut
nuts=0;//reset nuts
 // start closing operation
   if(metalsensedistance==10)
   {
     //if nut/stud sensed, put logic
     sensedstud=1;
   }
   else
   {
     sensedstud=0;
   }
    while(sensedstud==0)//if no nut/stud sensed, run disc
      {
        myStepperdisc.step(1);
      }
   restart2://goto marker
   if(sensedstud=1)// if nut has been sensed, closing nut operations
   {

     analogWrite(NUTRUNNERENA, 255); // Set nutrunner to extend shaft 
                                  
  digitalWrite(NUTRUNNERIN1, HIGH);  
  digitalWrite(NUTRUNNERIN2, LOW);
  delay(5000);
  digitalWrite(NUTRUNNERIN1, LOW);  
  digitalWrite(NUTRUNNERIN2, LOW);
  force=analogRead(forcefeedback);
  force=map(force,0,1023,0,1200);
if(force>>0 && force<=600)
{
  analogWrite(NUTRUNNERENA, 100); // Set motors to low speed
                              
  digitalWrite(NUTRUNNERIN1, HIGH);  //close
  digitalWrite(NUTRUNNERIN2, LOW);
}
else if(force>>600&&force<<1200)//maximum force is experienced when fully locked 
{
  analogWrite(NUTRUNNERENA, 20); // Set motors to very low speed
                                   
  digitalWrite(NUTRUNNERIN1, LOW);  //retract shaft
  digitalWrite(NUTRUNNERIN2, HIGH);
  delay(5000);
  analogWrite(NUTRUNNERENA, 0); // Set motors to maximum speed
                                   // For PWM maximum possible values are 0 to 255
  digitalWrite(NUTRUNNERIN1, LOW);  // Turn off nutrunner on motor driver 1
  digitalWrite(NUTRUNNERIN2, LOW);
  delay(5000);
  goto restart2;//restart closing process
}
if(force++ && force==1200)// if force has increased to maximum, the wheel nut has been fully closed
{
  analogWrite(NUTRUNNERENA, 10); // Set motors to very low speed
                                   // retract
  digitalWrite(NUTRUNNERIN1, LOW);  
  digitalWrite(NUTRUNNERIN2, HIGH);
  delay(2000);//retract for a short period to ensure it is removed from the nut

analogWrite(NUTRUNNERENA, 255); // Set motors to max speed
                                   // fully retract
  digitalWrite(NUTRUNNERIN1, LOW);  
  digitalWrite(NUTRUNNERIN2, HIGH);
  delay(2000);

  analogWrite(NUTRUNNERENA, 0);
                                
  digitalWrite(NUTRUNNERIN1, LOW);  // Turn off nutrunner 
  digitalWrite(NUTRUNNERIN2, LOW);
  studs++;//increment studs locked
  delay(10000);//wait for a while to put next nut
  sensedstud=0;// start running disc again
}
if(studs==5)// if all studs locked
{
do// ungrip until 2 gripping rods are equal distance from the tyre
{
  mySteppergrip.step(-1);
}
while( distance1!=distance2);
delay(5000);
while(distance<28)//as long as distance is less than 280mm, retract gantrt
{
myStepperX.step(200);
}
//maybe pushbutton to stop the proces
delay(60000);// stop process before next cyle
}

   }
 }
 
    }
    
      /*
      force feedback
      if no force feedback- stop nut runner
      if force feedback too high- stop nut runner. run nut runner in reverse, then reinsert and restart nut running.
      wait 15s, increment nut removed number, repeat disc process till all wheel nuts are out.
      if nut =5 or 6 retract nut runner
      run gripper motor
      if distance1==0 && distance2==0
      stop gripper motor
      return x to initial position
      move x to final position.
      detect nuts
      run in forward, check for force feedback
      if force feedback present to some extent reinsert and start running
      if force feed back maximum, retract, spin disc, and repeat for all nuts.
      */
    }
    
  }
    
  }
void forward(){ //Motor driver 1 
// Set Motor 1 forward
digitalWrite(FRONTWHEELSIN1, HIGH);
digitalWrite(FRONTWHEELSIN2, LOW);

// Set Motor 2 forward
digitalWrite(FRONTWHEELSIN3, HIGH);
digitalWrite(FRONTWHEELSIN4, LOW);

//Motor driver 2
// Set Motor 1 forward
digitalWrite(REARWHEELSIN1, HIGH);
digitalWrite(REARWHEELSIN2, LOW);

// Set Motor 2 forward
digitalWrite(REARWHEELSIN3, HIGH);
digitalWrite(REARWHEELSIN4, LOW);
}

void backward(){  //Motor driver 1  
// Set Motor 1 backward
digitalWrite(FRONTWHEELSIN1, LOW);
digitalWrite(FRONTWHEELSIN2, HIGH);
// Set Motor 2 backward
digitalWrite(FRONTWHEELSIN3, LOW);
digitalWrite(FRONTWHEELSIN4, HIGH);

//Motor driver 2
// Set Motor 1 backward
digitalWrite(REARWHEELSIN1, LOW);
digitalWrite(REARWHEELSIN2, HIGH);
// Set Motor 2 backward
digitalWrite(REARWHEELSIN3, LOW);
digitalWrite(REARWHEELSIN4, HIGH);
}

void turnRight(){ //Motor driver 1
// Set Motor 1 forward
digitalWrite(FRONTWHEELSIN1, HIGH);
digitalWrite(FRONTWHEELSIN2, LOW);
// Set Motor 2 backward 
digitalWrite(FRONTWHEELSIN3, LOW);
digitalWrite(FRONTWHEELSIN4, HIGH);

//Motor driver 2
// Set Motor 1 forward
digitalWrite(REARWHEELSIN1, HIGH);
digitalWrite(REARWHEELSIN2, LOW);
// Set Motor 2 backward 
digitalWrite(REARWHEELSIN3, LOW);
digitalWrite(REARWHEELSIN4, HIGH);
}

void turnLeft(){ //Motor driver 1
// Set Motor 1 backward 
digitalWrite(FRONTWHEELSIN1, LOW);
digitalWrite(FRONTWHEELSIN2, HIGH);
// Set Motor 2 forward 
digitalWrite(FRONTWHEELSIN3, HIGH);
digitalWrite(FRONTWHEELSIN4, LOW);

//Motor driver 2
// Set Motor 1 backward 
digitalWrite(REARWHEELSIN1, LOW);
digitalWrite(REARWHEELSIN2, HIGH);
// Set Motor 2 forward 
digitalWrite(REARWHEELSIN3, HIGH);
digitalWrite(REARWHEELSIN4, LOW);
}

void Stop(){//Motor driver 1
// Set Motor 1 stop
digitalWrite(FRONTWHEELSIN1, LOW);
digitalWrite(FRONTWHEELSIN2, LOW);
// Set Motor 2 stop
digitalWrite(FRONTWHEELSIN3, LOW);
digitalWrite(FRONTWHEELSIN4, LOW);

//Motor driver 2
// Set Motor 1 stop
digitalWrite(REARWHEELSIN1, LOW);
digitalWrite(REARWHEELSIN2, LOW);
// Set Motor 2 stop
digitalWrite(REARWHEELSIN3, LOW);
digitalWrite(REARWHEELSIN4, LOW);

}
  



