
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
double boardDistance = 4.25;            //inches
int space = 1;                    //Space from one photoresistor to another
float posofLaser[2] = {1.5, 1.5}; //physical position of the laser from the edges

//initialize a function that contains All of the possible positions
int positions[TARGETS];

//Check values
bool checkVal = 0;

//----------------------------------------------------------------------------------------------------------------------------------------------------
 
void setup() { // set pins to output 
  
  //Begin Serial Monitor 
  Serial.begin(9600);   //UART 0
  Serial1.begin(9600);  //UART 1

  //Initialize the servos to pin 2 and 3 on the PWM pins
  xAxis.attach(3);
  yAxis.attach(2);

  Serial.println("\n\n\nProgram Start!");

  //Find the preset random positions
  setRandom();
  
  Serial.println("\n\n Random Values Set! ");

  //Initialize all of the buttons as inputs
  initBUTTONS();

  //initialize the LEDs
  initLEDs();

  //initialize the LCD
  initLCD();
  
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
  
  //Print the final values that were selected
  printarray();
  
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
}

//----------------------------------------------------------------------------------------------------------------------------------------------------

//Print the whole array and the time elapsed on the serial monitor
void printarray(void){
  Serial.print("\nTime Elapsed (ms): ");
  Serial.println(millis());
  // Print those random values on the Serial Monitor
  for(int i = 0; i < TARGETS; i++){
    Serial.print("Target Number ");
    Serial.print(i);
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
  lcd.print("Row: ");
  lcd.setCursor(7, 0);
  lcd.print("Column: ");
}

//----------------------------------------------------------------------------------------------------------------------------------------------------

void lcdPRINT(int x_position, int y_position){
  lcd.setCursor(5,0);
  lcd.print((char) (x_position + 65));
  lcd.setCursor(15,0);
  lcd.print(y_position);
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
      for(int i = 0; i < 2; i++){
        pos[i] += (digitalRead(39 + 2*i) == 0)*(pos[i] < 3) - (digitalRead(38 + 2*i) == 0)*(pos[i] > 0) + 3*((pos[i] == 0) && (digitalRead(38 + 2*i) == 0)) - 3*((pos[i] == 3) && (digitalRead(39 + 2*i) == 0));
      }
      //While the button is still pressed loop in here
      while(!digitalRead(38) || !digitalRead(39) || !digitalRead(40) || !digitalRead(41)){
        delay(10);
      } // End while loop
    } //end if statement
    
    if(!digitalRead(42)){
      //Cast the integers to a character from A to D, then cast that character to a string
      Serial.println((String) ((char) (pos[0]*4 + 65 + pos[1]))); //send the position data to the ESP32
      //stay here so long as the button is still being pressed
      while(!digitalRead(42)){
        delay(10);
      }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------------------------

void loop() { 
  selectPosition();

  lcdPRINT(pos[0], pos[1]);
  
  returnedPOS[0] = pos[0];
  returnedPOS[1] = pos[1];

  moveSERVOS();

  int checker = 0;
  for(int i = 0; i < 16; i++){
    checker += (analogRead(54 + i) > 900)*(i);
  }
  checker += 1;
  lcd.setCursor(0,1);
  lcd.print("  ");
  lcd.setCursor(0,1);
  lcd.print(checker);

  if(Serial1.available()){      //Checks if there is data in UART1
    ESPData = Serial1.readString();
    Serial.print(ESPData);
    Serial1.print(ESPData);
  }
  
  
} //End MAIN loop
