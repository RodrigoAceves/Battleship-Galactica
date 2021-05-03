//include the Servo library, include the Liquid Crystal library
#include <Servo.h>
#include <LiquidCrystal.h>

//Define the number of targets and the 
//maximum number of places that they could be
#define TARGETS 6
#define maxPOS 16

//Create two servo objects
Servo xAxis; //This servo will control the motion in one direction
Servo yAxis; //This servo will control the motion in the other direction
LiquidCrystal lcd(9, 8, 7, 6, 5, 4);

//---------------------------------------------------------------------------------------------------------------------------------

//Store the return data
String ESPData;

//Keep track of the servo positions and step size
int pos[2];
int returnedPOS[2];
int stepSize = 5;
double boardDistance = 4.25;      //inches
int space = 1;                    //Space from one photoresistor to another
double posofLaser[2] = {1.5, 1.5}; //physical position of the laser from the edges {x, y}

//initialize a function that contains All of the possible positions
int positions[TARGETS];

//Check values
bool targetsHit[TARGETS]; //checks if a target has been hit before
int allTargetsHit; //if this is equal to the number of targets all targets are hit

//7 Segment display: thanks to https://create.arduino.cc/projecthub/meljr/7-segment-led-displays-102-using-a-shift-register-6b6976 
const int dataPin = 10;  // blue wire to 74HC595 pin 14
const int latchPin = 11; // green to 74HC595 pin 12
const int clockPin = 12; // yellow to 74HC595 pin 11
const char common = 'c';    // common cathode
bool decPt = false;  // decimal point display flag
int checkVal = 1; //checks if the 7 segment display needs updating

//----------------------------------------------------------------------------------------------------------------------------------------------------
 
void setup() { // set pins to output 
  
  //Begin Serial Monitor 
  Serial.begin(9600);   //UART 0 communication with the computer for testing
  Serial1.begin(9600);  //UART 1 communication to the ESP32
  
  //Initialize the servos to pin 2 and 3 on the PWM pins
  xAxis.attach(3);
  yAxis.attach(2);

  //Find the preset random positions
  setRandom();
  
  Serial.println("\n\nRandom Values Set!");

  //Initialize all of the buttons as inputs
  initBUTTONS();

  //initialize the LEDs
  initLEDs();

  //initialize the LCD
  initLCD();

  //Turn on the laser
  pinMode(44, OUTPUT);
  digitalWrite(44, HIGH);

  //initialize 7 segment display
  init7SEGDISP();
  
  //Tell the ESP32 to be ready on initialization
  Serial1.print("Program Start");
} 

void init7SEGDISP(void){
  pinMode(dataPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
}

//----------------------------------------------------------------------------------------------------------------------------------------------------

void initBUTTONS(void){
  //Enable the 5 buttons as inputs with pull up resistors for negative logic input switches
  for(int i = 0; i < 5; i++) pinMode(i + 38, INPUT_PULLUP);
}

//----------------------------------------------------------------------------------------------------------------------------------------------------

//Find the preset random values out of the preset range 
//into an array and make sure the random inputs are all different
void setRandom(void){

  //Set the random seed so that it would not be sudorandom
  randomSeed(random(analogRead(A1)/2,analogRead(A0)));
  
  //initialize repeat variable that will only be used in this function
  int repeat = 1;
  
  //set random values to the array
  Serial.println("Finding Random Values...");
  
  //Set all of the values in the array as random
  for(int i = 0; i < TARGETS; i++){
    positions[i] = random(0,maxPOS);
  }
  
  //Make sure none of those random values are repeating
  while(repeat > 0){
    //set repeat to 0
    repeat = 0;
    //Loop through every value
    for(int i = 0; i < TARGETS; i++){
      //Loop through every value multiple times
      for(int j = 0; j < TARGETS; j++){
        //if the values should not be the same then randomize 
        //one of them and make sure the function repeats again
        if((j != i) && (positions[j] == positions[i])){
          positions[j] = random(0,maxPOS);
          repeat++;
        }
      }
    }
  }
  
  //Sort the selected values
  Serial.println("\nSorting...");
  while(repeat < TARGETS){
    repeat = 1;
    //loop for all index values for the targets - 1 since the function compares with the next value
    for(int i = 0; i < TARGETS - 1; i++){
      //if the current position is greater than the following position
      if(positions[i] > positions [i+1]){
        //swap the values
        int temp = positions[i];
        positions[i] = positions[i+1];
        positions[i+1] = temp;
      }else{
        //increase the value of repeat.
        //if the value of repeat is triggered enough times, then we know that the values are sorted
        repeat++;
      }
    }
  }
  
  //Print the final values that were sorted
  printarray();


  Serial.println("Done.");
  Serial.print("Time Elapsed (ms): ");
  Serial.println(millis());
}

//----------------------------------------------------------------------------------------------------------------------------------------------------

//Print the whole array and the time elapsed on the serial monitor
void printarray(void){
  // Print those random values on the Serial Monitor
  for(int i = 0; i < TARGETS; i++){
    Serial.print("Target ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(positions[i]);
  }
}

//----------------------------------------------------------------------------------------------------------------------------------------------------

void initLEDs(void){
  //Initialize all 16 LEDs as outputs
  for(int i = 0; i < maxPOS; i++) pinMode(i + 22, OUTPUT);
  
  //Set all LED's to LOW (unncessesary, but precautionary)
  for(int i = 0; i < maxPOS; i++) digitalWrite(i,LOW);

  //Set the random LEDs that we found to the LEDs
  for(int i = 0; i < TARGETS; i++) digitalWrite(positions[i] + 22,HIGH);
}

//----------------------------------------------------------------------------------------------------------------------------------------------------

void initLCD(void){
  lcd.begin(16, 2);
  lcd.print("R:");
  lcd.setCursor(4, 0);
  lcd.print("C:");
  lcd.setCursor(8, 0);
  lcd.print("TRGS:");
}

//----------------------------------------------------------------------------------------------------------------------------------------------------

void lcdPRINT(int x_position, int y_position){
  lcd.setCursor(2,0);
  lcd.print((char) (x_position + 65));
  lcd.setCursor(6,0);
  lcd.print(y_position);
  for(int i = 0; i < TARGETS; i++){
    lcd.setCursor((14 + 3*i)%16,(14 + 3*i)/16);
    if(targetsHit[i]){
      lcd.print("  ");
      if(allTargetsHit == TARGETS){ 
        GameOver();
      }
    } else{
      lcd.print((char) ((positions[i] - positions[i]%4)/4+65));
      lcd.setCursor((15 + 3*i)%16,(15 + 3*i)/16);
      lcd.print(positions[i]%4);
    }
  }
}

//----------------------------------------------------------------------------------------------------------------------------------------------------

void GameOver(void){
  digitalWrite(44, LOW);
  Serial.println("Gameover");
  lcd.clear();
  while(1){
    for(int i = 0; i < 5; i++){
      lcd.setCursor(3,0);
      lcd.print("Game Over!");
      delay(500);
      lcd.clear();
      delay(500);
    }
    //just a fun animation
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

//----------------------------------------------------------------------------------------------------------------------------------------------------

void moveSERVOS(void){
  xAxis.write(findAngles(boardDistance,returnedPOS[0],returnedPOS[1], posofLaser[0], posofLaser[1], space));
  yAxis.write(findAngles(boardDistance,returnedPOS[1],returnedPOS[0], posofLaser[1], posofLaser[0], space));
}

//----------------------------------------------------------------------------------------------------------------------------------------------------

//find the angle of sensor
double findAngles(double distBoard, double POS_laser_A, double POS_laser_B, double POS_A, double POS_B, double sensors_space){
  return (90 - atan2(POS_laser_A - sensors_space*POS_A, sqrt( pow(distBoard,2) + pow(POS_laser_B - sensors_space*POS_B,2) ))*180/PI);
}

//----------------------------------------------------------------------------------------------------------------------------------------------------

//Determine where the laser should be moved
void selectPosition(void){
    if(!digitalRead(38) || !digitalRead(39) || !digitalRead(40) || !digitalRead(41)){
      delay(10); //debounce on press
      
      //change the respective value depending on the button pressed
      for(int i = 0; i < 2; i++) pos[i] += (digitalRead(39 + 2*i) == 0)*(pos[i] < 3) - (digitalRead(38 + 2*i) == 0)*(pos[i] > 0) + 3*((pos[i] == 0) && (digitalRead(38 + 2*i) == 0)) - 3*((pos[i] == 3) && (digitalRead(39 + 2*i) == 0));
      
      //While the button is still pressed loop in here
      while(!digitalRead(38) || !digitalRead(39) || !digitalRead(40) || !digitalRead(41)) delay(10);
    } //end if statement
    
    if(!digitalRead(42)){
      delay(10); //debounce on press

      //Position|TargetsAlreadyHit
      //Cast the integers to a character from A to P, then cast that character to a string, concatenate strings with casted integer onto respective ASCII integer
      Serial1.println((String) ((char) (pos[0]*4 + pos[1] + 65)) + (String) ((char) (allTargetsHit + 48))); //send the position data to the ESP32
      Serial.println((String) ((char) (pos[0]*4 + pos[1] + 65)) + (String) ((char) (allTargetsHit + 48))); //print to serial monitor
      
      //stay here so long as the button is still being pressed
      while(!digitalRead(42)) delay(10); 
      
    }
}

//----------------------------------------------------------------------------------------------------------------------------------------------------

void checkHIT(void){
  for(int i = 0; i < TARGETS; i++) if((analogRead(positions[i] + 54) > 900) && (targetsHit[i] == 0) ) {
    //if the target is hit, then the LED turns off and the value that keeps track of the target that was hit
    targetsHit[i] = 1;
    digitalWrite(positions[i] + 22, LOW);
    allTargetsHit++;
  }
}

//----------------------------------------------------------------------------------------------------------------------------------------------------

void set7SEGDISP(void){
  if(checkVal != allTargetsHit) {
    byte bits = myfnNumToBits(allTargetsHit) ; //Display the desired digit
    if (decPt) {
      bits = bits | B00000001;  // add decimal point if needed
    }
    myfnUpdateDisplay(bits);    // display alphanumeric digit
  }
}

//----------------------------------------------------------------------------------------------------------------------------------------------------

void myfnUpdateDisplay(byte eightBits) {
  digitalWrite(latchPin, LOW);  // prepare shift register for data
  shiftOut(dataPin, clockPin, LSBFIRST, eightBits); // send data
  digitalWrite(latchPin, HIGH); // update display
}

//----------------------------------------------------------------------------------------------------------------------------------------------------

byte myfnNumToBits(int someNumber) {
  switch (someNumber) {
    case 0:
      return B11111100;
      break;
    case 1:
      return B01100000;
      break;
    case 2:
      return B11011010;
      break;
    case 3:
      return B11110010;
      break;
    case 4:
      return B01100110;
      break;
    case 5:
      return B10110110;
      break;
    case 6:
      return B10111110;
      break;
    case 7:
      return B11100000;
      break;
    case 8:
      return B11111110;
      break;
    case 9:
      return B11110110;
      break;
    case 10:
      return B11101110; // Hexidecimal A
      break;
    case 11:
      return B00111110; // Hexidecimal b
      break;
    case 12:
      return B10011100; // Hexidecimal C
      break;
    case 13:
      return B01111010; // Hexidecimal d
      break;
    case 14:
      return B10011110; // Hexidecimal E
      break;
    case 15:
      return B10001110; // Hexidecimal F
      break;  
    default:
      return B10010010; // Error condition, displays three horizontal bars
      break;   
  }
}

//----------------------------------------------------------------------------------------------------------------------------------------------------

//MAIN LOOP
void loop() { 
  selectPosition();

  lcdPRINT(pos[0], pos[1]);
  
  if((Serial1.available() > 0)){
    
    //reads what we recieved from the 
    ESPData = Serial1.readString();
    int temp = (int) ESPData.toInt();
    Serial.println(ESPData);
    
    //returnedPOS will be the returned positions from the master operator
    returnedPOS[0] = (temp)%4;
    returnedPOS[1] = (temp - returnedPOS[1])/4;
    Serial.print("Row: ");
    Serial.println(returnedPOS[1]);
    Serial.print("Colunm: ");
    Serial.println(returnedPOS[0]);
  }
  
  

  //The servos only listen to returned POS
  moveSERVOS();
  
  //check if one of the targets were hit
  checkHIT();

  set7SEGDISP();

  checkVal = allTargetsHit;
  
} //End MAIN loop
