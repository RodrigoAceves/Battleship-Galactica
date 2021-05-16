//Sensors: MPU, Tilt, FSR, PIR, Ultrasonic, Sound
//Triggers: Fan, RGB LED, BAR LED

//GND, POWER, CONTRAST GND 5.1k in parallel then series with 100kand10k in parallel, 12, GND, 11, --, --, --, --, 10, 9, 8, 7, 220 OHM connected to POWER,GND
#include <LiquidCrystal.h>
#include "I2Cdev.h"
#include "MPU6050.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

#define OUTPUT_READABLE_ACCELGYRO

#define rowPot    A0
#define columnPot A1
#define button    22
#define FSR       A15 //800 Ohms for reasonably firm press -- we will have in series with a 10kOhm resistor
#define Motor     4
#define trig      6
#define echo      5
#define PIR       31
#define ASM       A2
#define TILT      32
#define SDA       20
#define SCL       21

byte RGB[3];
byte barLED[5]; 

int goPOS[2]; //{column,row}
int targetNUM = -1;
int pot[2];

bool r = LOW;

LiquidCrystal lcd(12, 11, 10, 9, 8, 7);

String ESPData; 

void setup() {
  //Initialize the UART Config
  Serial.begin(9600);   //UART 0 communication with the computer for testing
  Serial1.begin(9600);  //UART 1 communication to the ESP32

  //button initialized as input with pullup resistor
  pinMode(button, INPUT_PULLUP);
  
  //Motor initialized as output
  pinMode(Motor, OUTPUT);

  //initialize the RGB LEDs as outputs define the pins
  for(int i = 0; i<3; i++){
    RGB[i] = 23 + i;
    pinMode(RGB[i],OUTPUT);
  }

  //initialize BARleds
  for(int i =0; i<5; i++){
    barLED[i] = 26 + i;
    pinMode(barLED[i],OUTPUT);
  }

  //initialize the LCD
  lcd.begin(16,2);
  lcd.print("Row:");
  lcd.setCursor(7, 0);
  lcd.print("Column:");

  //FSR as digital
  pinMode(FSR,INPUT_PULLUP);


  //initialize the Ultrasonic Sensor Pins
  pinMode(trig,OUTPUT);
  pinMode(echo, INPUT);

  //initialize PIR sensor
  pinMode(PIR, INPUT);

  //Initialize Motor
  pinMode(Motor, OUTPUT);

  //Initialize the Tilt sensor
  pinMode(TILT, INPUT_PULLUP);

  //Initialize the MPU
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  accelgyro.initialize();

  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}

void loop() {

  //Read data from UART1
  if((Serial1.available() > 0)){
    ESPData = Serial1.readString();
    int lengthstring = ESPData.length() + 1;
    char temp[lengthstring]; //stores the characters instead of the whole string
    temp[1] = -1;
    ESPData.toCharArray(temp,lengthstring);
    if((temp[0] >= 65) && (temp[0] <= 80)){
      targetNUM = temp[1] - 48;
  
      goPOS[0] = (temp[0] - 65)%4;
      goPOS[1] = ((temp[0] - 65) - goPOS[0])/4;
  
      Serial.print("Column: ");
      Serial.println(goPOS[0]); //Column 
      Serial.print("Row: ");
      Serial.println(goPOS[1]); //Row
      lcd.setCursor(5,0);
      lcd.print(goPOS[1]);
      lcd.setCursor(15,0);
      lcd.print(goPOS[0]);
      Serial.println(ESPData);
      if(temp[1]>-1){
        r = HIGH;
      }
    }
  }

//Check for FSR
  if((analogRead(FSR)>800) && (targetNUM == 0) && (r == HIGH)){
    Serial.println((int) (pot[0]*4+pot[1]));
    Serial1.println((int) (pot[0]*4+pot[1]));
    Serial.println("Position Sent");
    barLEDfunc();
    r = LOW;
  }


//Check for Ultrasonic sensor
  digitalWrite(trig,LOW);
  delayMicroseconds(2);
  digitalWrite(trig,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig,LOW);
  long duration = pulseIn(echo,HIGH);
  if((duration < 1000) && (targetNUM == 1) && (r == HIGH)){
    Serial.println((int) (pot[0]*4+pot[1]));
    Serial1.println((int) (pot[0]*4+pot[1]));
    Serial.println("Position Sent");
    rgbLEDs(); 
    r = LOW;
  }

//Check for PIR Sensor
  if((digitalRead(PIR) == 1) && (targetNUM == 2) && (r == HIGH)){
    Serial.println((int) (pot[0]*4+pot[1]));
    Serial1.println((int) (pot[0]*4+pot[1]));
    Serial.println("Position Sent");

    int motorSpeed = 200;
    for(int i = 0; (i < 1000); i++){
      motorSpeed+= 3*(((digitalRead(PIR)?1:-1)) + (((motorSpeed + (digitalRead(PIR)?1:-1)) >= 254 )?-1:0) + (((motorSpeed + (digitalRead(PIR)?1:-1)) <=2)?1:0));
      analogWrite(Motor,motorSpeed);
      delay(10);
      Serial.println(motorSpeed);
    }
    analogWrite(Motor,0);
    r = LOW;
  }

  //Check for Audio Sensor Module Sensor
  if((analogRead(ASM) > 50) && (targetNUM == 3) && (r == HIGH)){
    Serial.println((int) (pot[0]*4+pot[1]));
    Serial1.println((int) (pot[0]*4+pot[1]));
    Serial.println("Position Sent");
    barLEDfunc();
    r = LOW;
  }

  //Check for Tilt Sensor
  if((!digitalRead(TILT)) && (targetNUM == 4) && (r == HIGH)){
    Serial.println((int) (pot[0]*4+pot[1]));
    Serial1.println((int) (pot[0]*4+pot[1]));
    Serial.println("Position Sent");
    barLEDfunc();
    r = LOW;
  }



  //Check for MPU5060 Sensor
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    #ifdef OUTPUT_BINARY_ACCELGYRO
        Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
        Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
        Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
        Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
        Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
        Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
    #endif
  if((az > 20000) && (targetNUM == 5) && (r == HIGH)){
    Serial.println((int) (pot[0]*4+pot[1]));
    Serial1.println((int) (pot[0]*4+pot[1]));
    Serial.println("Position Sent");
    int positioncurrent = gy;
    for(int j = 0; j< 100; j++){
        //Check for MPU5060 Sensor
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
        #ifdef OUTPUT_BINARY_ACCELGYRO
           Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
        #endif
        positioncurrent = (positioncurrent + gy)/2;
        delay(10);
      }

    int counter = millis();
    int motorSpeed = 128;
    for(int i = 0; (i < 100); i++){
        //Check for MPU5060 Sensor
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
        #ifdef OUTPUT_BINARY_ACCELGYRO
           Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
        #endif
      if(positioncurrent + 100 < gy){
        motorSpeed += 20;
        if (motorSpeed >= 255) motorSpeed = 255;
        Serial.println("Increased");
      }else if(positioncurrent - 100 > gy){
        motorSpeed -= 20;
        if (motorSpeed <= 50) motorSpeed = 50;
        Serial.println("Decreased");
      }
      analogWrite(Motor,motorSpeed);
      Serial.print("Average: "); Serial.print(positioncurrent); Serial.print(" Current Value: "); Serial.println(gy);
      Serial.println(motorSpeed);
      delay(100);
    }
    Serial.print("Time (ms): ");
    Serial.println(millis() - counter);
    analogWrite(Motor,0);
    r = LOW;
    Serial.println("End MPU");
    GameOver();
  }
//-----------------------------------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------------------------------
  lcd.setCursor(0,1);
  lcd.print("R:");
  lcd.print(fourRange(0,analogRead(rowPot)));
  lcd.print(" C:");
  lcd.print(fourRange(1,analogRead(columnPot)));
  lcd.print(" Target:");
  lcd.print(targetNUM + 1);
  
//  Serial.print("\n\n\n\n\nROW: ");
//  Serial.println(fourRange(0,analogRead(rowPot)));
//  Serial.print("Column: ");
//  Serial.println(fourRange(1,analogRead(columnPot)));
//  Serial.print("FSR Value: ");
//  Serial.println(analogRead(FSR));
//  Serial.print("Ultrasonic Feedback: ");
//  Serial.println(duration);
//  Serial.print("PIR Sensor: ");
//  Serial.println(digitalRead(PIR));
//  Serial.print("Sound Sensor: ");
//  Serial.println(analogRead(ASM)>50);
//  Serial.print("Tilt sensor: ");
//  Serial.println(digitalRead(TILT));
//  Serial.print("MPU Trigger Value: ");
//  Serial.println(az);
 delay(10);
}


int fourRange(bool tem,int range){
  if(range < 256){
    pot[tem] = 0;
  } else if(range< 512){
    pot[tem] = 1;
  } else if(range < 768){
    pot[tem] = 2;
  } else {
    pot[tem] = 3;
  }
  return pot[tem];
}

void barLEDfunc(void){
  for(int j = 0; j < 2; j++){
    for(int i = 0; i<5; i++){
      digitalWrite(barLED[i],HIGH);
      delay(526);
    }
    for(int i = 4; i > -1; i--){
      digitalWrite(barLED[i],LOW);
      delay(526);
    }//6 milliseconds off
  }
}

//RGB LEDs need to turn on one at a time for 10 seconds
void rgbLEDs(void){
  int counter = millis();
  for(int j =0; j < 4; j++){
    digitalWrite(23, HIGH);
    delay(666);
    for(int i = 0; i<2; i++){
      digitalWrite(23+i, LOW);
      digitalWrite(24+i, HIGH);
      delay(666);
    }
    digitalWrite(25, LOW);
    delay(666);
  }
  Serial.print("Time Elapsed (ms): ");
  Serial.println(millis()-counter-656);
}

//Motor Trigger
void MotorFunc(){
  int motorSpeed = 200;
  for(int i = 0; (i < 1000); i++){
    motorSpeed+= 3*(((digitalRead(PIR)?1:-1)) + (((motorSpeed + (digitalRead(PIR)?1:-1)) >= 254 )?-1:0) + (((motorSpeed + (digitalRead(PIR)?1:-1)) <=2)?1:0));
    analogWrite(Motor,motorSpeed);
    delay(10);
    Serial.println(motorSpeed);
  }
  analogWrite(Motor,0);
}


void GameOver(void){
  digitalWrite(44, LOW);              //Turn off the Laser
  Serial.println("Gameover");         //print Gameover to the serial monitor
  lcd.clear();                        //clear the LCD display
  while(1){                           //lock the function in this loop
    for(int i = 0; i < 5; i++){       //This for loop simplay flashes the 'Game Over!' text in the middle of the LCD display
      lcd.setCursor(3,0);             //Set the cursor to a position so Game Over could be centered
      lcd.print("Game Over!");        //Print Game Over on the display
      delay(500);                     //wait
      lcd.clear();                    //Clear the screen
      delay(500);                     //wait
    }


//just an animation... 
    for(int j =0; j < 4; j++){
      for(int i = 0; i < 12; i++){
        lcd.setCursor(i,j%2);
        lcd.print("--->");
        delay(50);
      }
      lcd.clear();
      for(int i = 12; i > 0; i--){
        lcd.setCursor(i,j%2);
        lcd.print("<---");
        delay(50);
      }
    }
  }
}
