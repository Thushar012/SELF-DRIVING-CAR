#include <Servo.h>




//Self Driving(Line follower Robot)
#define enA 5//Enable1 L298 Pin enA 
#define in1 6 //Motor1  L298 Pin in1 
#define in2 7 //Motor1  L298 Pin in1 
#define in3 8 //Motor2  L298 Pin in1 
#define in4 9 //Motor2  L298 Pin in1 
#define enB 10 //Enable2 L298 Pin enB 

#define R_S A0 //ir sensor Right
#define L_S A1 //ir sensor Left

#define RightLED 3
#define LeftLED 11

#define RightInd 12
#define LeftInd 13

#define Echo A2
#define Trigger A3

#define Buzzer 2
#define servo A5
int Speed = 80;
int turn = 175;
int StopDistance = 10;
int zebra = 18;
int distance_L, distance_F, distance_R;

bool OnRightLane = true;
void setup(){
  Serial.begin(9600); // put your setup code here, to run once

pinMode(R_S, INPUT); // declare if sensor as input  
pinMode(L_S, INPUT); // declare ir sensor as input

pinMode(enA, OUTPUT); // declare as output for L298 Pin enA 
pinMode(in1, OUTPUT); // declare as output for L298 Pin in1 
pinMode(in2, OUTPUT); // declare as output for L298 Pin in2 
pinMode(in3, OUTPUT); // declare as output for L298 Pin in3   
pinMode(in4, OUTPUT); // declare as output for L298 Pin in4 
pinMode(enB, OUTPUT); // declare as output for L298 Pin enB 
pinMode(Echo, INPUT);
pinMode(Trigger,OUTPUT);

pinMode(RightLED , OUTPUT);
pinMode(LeftLED , OUTPUT);

pinMode(RightInd , OUTPUT);
pinMode(LeftInd, OUTPUT);
pinMode(Buzzer , OUTPUT);
pinMode(servo, OUTPUT);


for (int angle = 80; angle <= 140; angle += 5)  {
   servoPulse(servo, angle);  }
 for (int angle = 140; angle >= 0; angle -= 5)  {
   servoPulse(servo, angle);  }

 for (int angle = 0; angle <= 80; angle += 5)  {
   servoPulse(servo, angle);  }

   distance_F = Ultrasonic_read();
   delay(500);
   jumpStart(180);
  // TCCR0B = TCCR0B & B11111000 | B00000010 ;
 // Write The Duty Cycle 0 to 255 Enable Pin B for Motor2 Speed 
//  analogWrite(enA, 200);
//  analogWrite(enB, 200);

}
void loop(){
  distance_F = Ultrasonic_read();
  Serial.print("D F=");Serial.println(distance_F);
  
//OLD DISTANCE SENSOR
  // clear the trig pin
  // digitalWrite(Trigger, LOW);
  // delayMicroseconds(2);
  // digitalWrite(Trigger, HIGH);
  // delayMicroseconds(10);
  // digitalWrite(Trigger, LOW);

  // int duration = pulseIn(Echo,HIGH);
  // int distance = duration * 0.034 /2;
  // Serial.println(distance);
  // delay(100);//distance = time * speed (divide by two cuz bounce back time)



  if((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 1)){// 0 : 0
    if(distance_F > StopDistance){
      
      forword();
    }
    else{
      Stop();
      digitalWrite(Buzzer, HIGH);
      delay(300);
      digitalWrite(Buzzer, LOW);
      delay(300);
      digitalWrite(Buzzer, HIGH);
      delay(300);
      digitalWrite(Buzzer, LOW);
      delay(300);
      digitalWrite(Buzzer, HIGH);
      delay(300);
      digitalWrite(Buzzer, LOW);
      distance_F = Ultrasonic_read();
      delayMicroseconds(20);
      if(distance_F <= StopDistance){
        LaneChange();
      // Check_side();
      }else {
        jumpStart(200);
        delay(100);
        digitalWrite(LeftLED, LOW);
        digitalWrite(RightLED, LOW);
      }

    }
    }   //if Right Sensor and Left Sensor are at Black color then it will call forword function

  if((digitalRead(R_S) == 0)&&(digitalRead(L_S) == 1)){turnLeft();} // 1 : 0  if Right Sensor is Black and Left Sensor is White then it will call turn Right function  

  if((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 0)){turnRight();}  // 0 : 1  if Right Sensor is White and Left Sensor is Black then it will call turn Left function

  if((digitalRead(R_S) == 0)&&(digitalRead(L_S) == 0)){ZebraCrossing();} //if Right Sensor and Left Sensor are at Black color then it will call Stop function
  delay(10);
}

void servoPulse (int pin, int angle){
  int pwm = (angle*11) + 500;      // Convert angle to microseconds
 digitalWrite(pin, HIGH);
 delayMicroseconds(pwm);
 digitalWrite(pin, LOW);
 delay(50);
}
 //STOPPING THE CAR 
  // if (distance <= StopDistance) {
  // Stop();
  // analogWrite(enA, 200);
  // analogWrite(enB, 200);
  // delayMicroseconds(1000);
  // analogWrite(enA, Speed);
  // analogWrite(enB, Speed);
  // delayMicroseconds(100);
    // if(distance<= StopDistance){
    //   delay(2000);
    //   digitalWrite(Buzzer, HIGH);
    // }

  
  
  // }else {
  // digitalWrite(RightLED, LOW);
  // digitalWrite(LeftLED, LOW);
  // digitalWrite(Buzzer, LOW);
  // objectdetected = false;
  // }
 
// }

long Ultrasonic_read(){
  digitalWrite(Trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(Trigger, HIGH);
  delayMicroseconds(10);
  long time = pulseIn (Echo, HIGH);
  //return time / 29 / 2;
  long distance = time * 0.034 /2;
  return distance;
}

void Check_side(){
    Stop();
    delay(100);
 for (int angle = 70; angle <= 140; angle += 5)  {
   servoPulse(servo, angle);  }
    delay(300);
    distance_L = Ultrasonic_read();
    Serial.print("D R=");Serial.println(distance_R);
    delay(100);
  for (int angle = 140; angle >= 0; angle -= 5)  {
   servoPulse(servo, angle);  }
    delay(500);
    distance_R = Ultrasonic_read();
    Serial.print("D L=");Serial.println(distance_L);
    delay(100);
 for (int angle = 0; angle <= 70; angle += 5)  {
   servoPulse(servo, angle);  }
    delay(300);
    digitalWrite(LeftLED, LOW);
    digitalWrite(RightLED, LOW);
    compareDistance();
}

void compareDistance(){
    if(distance_L > distance_R){
  digitalWrite(LeftInd, HIGH);
  turnLeft();
  delay(500);//1000
  forword();
  delay(1100);
  turnRight();
  delay(1000);
  forword();
  delay(1100);
  turnRight();
  delay(900);
  turnLeft();
  delay(1000);
  digitalWrite(LeftInd, LOW);
  }
  else{
  digitalWrite(RightInd, HIGH);
  turnRight();
  delay(500);//1000
  forword();
  delay(1100);
  turnLeft();
  delay(1000);
  forword();
  delay(1100);  
  turnLeft();
  delay(900);
  turnRight();
  delay(1000);
  digitalWrite(RightInd, LOW);
  }
}

void forword(){  //forword
digitalWrite(in1, HIGH); //Right Motor forword Pin 
digitalWrite(in2, LOW);  //Right Motor backword Pin 
digitalWrite(in3, LOW);  //Left Motor backword Pin 
digitalWrite(in4, HIGH); //Left Motor forword Pin 
analogWrite(enA, Speed);
analogWrite(enB, Speed);
}

void jumpStart( int Jspeed){  //forword
digitalWrite(in1, HIGH); //Right Motor forword Pin 
digitalWrite(in2, LOW);  //Right Motor backword Pin 
digitalWrite(in3, LOW);  //Left Motor backword Pin 
digitalWrite(in4, HIGH); //Left Motor forword Pin 
analogWrite(enA, Jspeed);
analogWrite(enB, Jspeed);
}

void Backword(){  //forword
digitalWrite(in1, LOW); //Right Motor forword Pin 
digitalWrite(in2, HIGH);  //Right Motor backword Pin 
digitalWrite(in3, HIGH);  //Left Motor backword Pin 
digitalWrite(in4, LOW); //Left Motor forword Pin 
analogWrite(enA, Speed);
analogWrite(enB, Speed);
}

void turnRight(){
//turnRight
digitalWrite(in1, LOW);  //Right Motor forword Pin 
digitalWrite(in2, HIGH); //Right Motor backword Pin  
digitalWrite(in3, LOW);  //Left Motor backword Pin 
digitalWrite(in4, HIGH); //Left Motor forword Pin 

analogWrite(enA, turn);
analogWrite(enB, turn);
}

void turnLeft(){ 
//turnLeft
digitalWrite(in1, HIGH); //Right Motor forword Pin 
digitalWrite(in2, LOW);  //Right Motor backword Pin 
digitalWrite(in3, HIGH); //Left Motor backword Pin 
digitalWrite(in4, LOW);  //Left Motor forword Pin 
analogWrite(enA, turn);
analogWrite(enB, turn);
}

void Stop(){ //stop
digitalWrite(in1, LOW); //Right Motor forword Pin 
digitalWrite(in2, LOW); //Right Motor backword Pin 
digitalWrite(in3, LOW); //Left Motor backword Pin 
digitalWrite(in4, LOW); //Left Motor forword Pin 
analogWrite(enA, Speed);
analogWrite(enB, Speed);
digitalWrite(RightLED, HIGH);
digitalWrite(LeftLED, HIGH);

}
void ZebraCrossing(){
  Stop();
  delay(1000);
  distance_F = Ultrasonic_read();
  if(distance_F>=zebra && (digitalRead(R_S) == 0)&&(digitalRead(L_S) == 0)){
    digitalWrite(RightLED, LOW);
    digitalWrite(LeftLED, LOW);
    jumpStart(180);
    delay(200);
    
  }
  
}

void LaneChange(){
  digitalWrite(LeftLED, LOW);
  digitalWrite(RightLED, LOW);
   
    distance_F = Ultrasonic_read();
    
    if(OnRightLane && distance_F <= StopDistance){
     
      digitalWrite(LeftInd, HIGH);
      turnLeft();
      delay(400);
      while ((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 1)) {
        forword();
        OnRightLane = false;
      }
      digitalWrite(LeftInd, LOW);
    }
  
   
    distance_F = Ultrasonic_read();
    
  if(!OnRightLane && distance_F <= StopDistance){
    digitalWrite(RightInd, HIGH);
    turnRight();
    delay(400);
    while ((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 1)) {
      forword();
      OnRightLane = true;
    }
    digitalWrite(RightInd, LOW);
  }
}
