#include <Servo.h>

Servo x;
Servo y;
bool pause1 = 0;
int AllTheAngles[16];
int delayTime = 20;
int low[2] = {72, 5};
int high[2] = {111, 33};
void setup() {
  // put your setup code here, to run once:
  x.attach(3);
  y.attach(2);
  pinMode(42,INPUT_PULLUP);
  pinMode(44,OUTPUT);
  digitalWrite(44,HIGH);
  y.write(90);
  Serial.begin(9600);
}

void loop() {
  for(int j = low[1]; j < high[1]; j++){
    for(int i = low[0]; i < high[0]; i++){
      x.write(i);
      delay(delayTime);
      temp(i, j);
    }
    for(int i = high[0]; i > low[0]; i--){
      x.write(i);
      delay(delayTime);
      temp(i, j);
    }
    y.write(j);
  }
  
  for(int j = high[1]; j > low[1]; j--){
    for(int i = low[0]; i < high[0]; i++){
      x.write(i);
      delay(delayTime);
      temp(i, j);
    }
    for(int i = high[0]; i > low[0]; i--){
      x.write(i);
      delay(delayTime);
      temp(i, j);
    }
    y.write(j);
  }


}

void temp(int currentPosx, int currentPosy){
  
  while(digitalRead(42)){
    delay(10);
    Serial.print("y Servo: ");
  Serial.println(currentPosy);
  Serial.print("x Servo: ");
  Serial.println(currentPosx);
  }
}
//void moveSERVOS(void){
//  x.write(findAngles(boardDistance,returnedPOS[0],returnedPOS[1], posofLaser[0], posofLaser[1], space, 90));
//  y.write(findAngles(boardDistance,returnedPOS[1],returnedPOS[0], posofLaser[1], posofLaser[0], space, 90));
//}
//
////find the angle of sensor
//double findAngles(double distBoard, double POS_laser_A, double POS_laser_B, double POS_A, double POS_B, double sensors_space,double offsets){
//  return (offsets - atan2(POS_laser_A - sensors_space*POS_A, sqrt( pow(distBoard,2) + pow(POS_laser_B - sensors_space*POS_B,2) ))*180/PI);
//}
