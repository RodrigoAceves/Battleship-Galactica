//included libraries:
#include <Servo.h>          //include the Servo library
#include <LiquidCrystal.h>  //Include the LCD Display library

//this will be constant throughtout the program:
#define TARGETS 6 //Define the number of targets and the 
#define maxPOS 16 //maximum number of places that they could be

//Create two servo objects and one lcd object
Servo xAxis; //This servo will control the motion in one direction
Servo yAxis; //This servo will control the motion in the other direction
LiquidCrystal lcd(9, 8, 7, 6, 5, 4); //Pins 4 through 9 are for the LCD display



//----------------------------------------------------------------------------------------------------------------------------------------------------
/*
 * In the following lines are the stored data that will likely be used throught the program.
*/
//----------------------------------------------------------------------------------------------------------------------------------------------------
//Store ESP32 data
String ESPData;                     //This string will store the informaiton coming in from the ESP32

//Keep track of the servo positions and step size
int pos[2];                         //This will store the position that we want the master operator to go to
int x[17] =    {0,103, 89, 76, 64, 
                102, 89, 76, 64, 
                103, 89, 76, 65, 
                103, 89, 77, 65};       //Hardcoded x-y positions
int y[17] =    {0, 14,14,15, 16, 
                24, 23, 25, 24, 
                35, 34, 35, 35,
                45,44, 45,45};
//int x[4] = {111, 99, 87, 75};       //Hardcoded x-y positions
//int y[4] = {5,15,25,33};
int returnedPOS[2];                 //This section stores the position that is returned by the master operator

//Learn the position of the laser from the x, y, and z position and the distance of each photoresistor from one another
// int stepSize = 5; // I dont think this line is necessary i was probably messing with a particulat funciton
double boardDistance = 4.25;        //inches
int space = 1;                      //Space from one photoresistor to another
double posofLaser[2] = {1, 1.5}; //physical position of the laser from the edges

//initialize a function that contains All of the possible positions
int positions[TARGETS];

//Check values
bool targetsHit[TARGETS]; //checks if a target has been hit before
int allTargetsHit;        //if this is equal to the number of targets all targets are hit

//7 Segment display: thanks to https://create.arduino.cc/projecthub/meljr/7-segment-led-displays-102-using-a-shift-register-6b6976 
const int dataPin = 10;  // blue wire to 74HC595 pin 14
const int latchPin = 11; // green to 74HC595 pin 12
const int clockPin = 12; // yellow to 74HC595 pin 11
const char common = 'c'; // common cathode
bool decPt = false;      // decimal point display flag
int checkVal = 1;        //checks if the 7 segment display needs updating

//----------------------------------------------------------------------------------------------------------------------------------------------------
/*
 * In the Setup function, all of the initializations and most redundant calculations occur.
 */
//----------------------------------------------------------------------------------------------------------------------------------------------------
void setup() { // set pins to output 
  
  //Begin Serial Monitor 
  Serial.begin(9600);   //UART 0 communication with the computer for testing
  Serial1.begin(9600);  //UART 1 communication to the ESP32
  
  //Initialize the servos to pin 2 and 3 on the PWM pins
  xAxis.attach(3); //x-axis servo
  yAxis.attach(2); //y-axis Servo

  //Find the preset random positions
  setRandom(); 

  //State on UART0 that all of the random values are set
  Serial.println("\n\nRandom Values Set!");

  //Initialize all of the buttons as inputs
  initBUTTONS();

  //initialize the LEDs
  initLEDs();

  //initialize the LCD
  initLCD();

  //Turn on the laser
  pinMode(44, OUTPUT);    //Laser is plugged into pin 44 for more control
  digitalWrite(44, HIGH); //Laser turns on

  //initialize 7 segment display
  init7SEGDISP();
} 

//----------------------------------------------------------------------------------------------------------------------------------------------------
/*
 * The following program will initialize the data pin, latch pin, and clock pin with respect to the preconfigured values as outputs.
 */
//----------------------------------------------------------------------------------------------------------------------------------------------------
void init7SEGDISP(void){
  pinMode(dataPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
}

//----------------------------------------------------------------------------------------------------------------------------------------------------
/*
 * The following program will initialize the five buttons starting on pin 38 with the internal pull-up resistor of the microcontroller.
 */
//----------------------------------------------------------------------------------------------------------------------------------------------------
void initBUTTONS(void){
  //Enable the 5 buttons as inputs with pull up resistors for negative logic input switches
  for(int i = 0; i < 5; i++) pinMode(i + 38, INPUT_PULLUP);
}

//----------------------------------------------------------------------------------------------------------------------------------------------------
/*
 * The following program will use the internal sudorandom program of the microcontroller, but make it random by setting the random seed as some random 
 * analog based value. It will then generate a random array that is sized 'TARGETS' as previously defined. It will then regenerate random values if there
 * are any repeating values until all of the randomly generated values are different. Afterwards they will be sorted and ready for use in the remainder 
 * of the program. At this point, sorting it is unnecessary but it may be of use in future projects.
 */
//----------------------------------------------------------------------------------------------------------------------------------------------------
void setRandom(void){

  //Set the random seed so that it would not be sudorandom
  randomSeed(random(analogRead(A1)/2,analogRead(A0)));
  
  //initialize repeat variable that will only be used in this function
  int repeat = 1;
  
  //set random values to the array
  Serial.println("Finding Random Values...");
  
  //Set all of the values in the array as random
  for(int i = 0; i < TARGETS; i++){
    positions[i] = random(1,maxPOS);
  }
  
  //Make sure none of those random values are not repeating
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
          positions[j] = random(1,maxPOS);
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

  //General code to check the timing of my program
  Serial.println("Done.");
  Serial.print("Time Elapsed (ms): ");
  Serial.println(millis());
}

//----------------------------------------------------------------------------------------------------------------------------------------------------
/*
 * In the printarray program, I will print the whole array on the serial monitor.
 */
//----------------------------------------------------------------------------------------------------------------------------------------------------

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
/*
 * This initLEDs program initializes the LEDs that are attatched from pin 22 to pin 37 of the Arduino MEGA. It will then assuire that the output is
 * which is redundant and likely can be removed altogether. Finally, it sets the respective LEDs for the random targets that the program finds using 
 * the random function generator.
 */
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
/*
 * In the initLCD function, the strings 'R:', 'C:', and 'TRGS:' are set so that they only need to be set once in the program.
 */
//----------------------------------------------------------------------------------------------------------------------------------------------------

void initLCD(void){
  lcd.begin(16, 2); //initialize LCD
  lcd.print("R:");  //Print on the first row and column
  lcd.setCursor(4, 0);  //Move Cursor to the fifth column
  lcd.print("C:");      //print on the fifth column first row
  lcd.setCursor(8, 0);  //Mover cursor to the eighth position
  lcd.print("TRGS:");   //Print TRGS: on the eighth position first row
}

//----------------------------------------------------------------------------------------------------------------------------------------------------
/*
 * In this lcdPRINT program, the x position and y position are provided in the function then the function displays the corresponding value on the LCD 
 * display. Afterwards, the program checks which targets are hit then turns on and off the respective targets. Here is where the function checks if 
 * all the targets have been hit. If they all are, the program goes into the Game Over function which will be shown on another portion of the program.
 * For the complicated looking calculation within the setCursor function of the lcd object, the program is simply processing how to space out each
 * target so that they can all be shown neatly on the LCD display.
 */
//----------------------------------------------------------------------------------------------------------------------------------------------------

void lcdPRINT(int x_position, int y_position){
  lcd.setCursor(2,0);                   //Move Cursor to second third column first row
  lcd.print((char) (x_position + 65));  //print the x position converted to the respective character w/ ASCII table
  lcd.setCursor(6,0);                   //Move cursor to the seventh position
  lcd.print(y_position);                //Print the y position as an integer
  for(int i = 0; i < TARGETS; i++){     //Loop through all of the targets
    lcd.setCursor((14 + 3*i)%16,(14 + 3*i)/16); //Print into the LCD cursor the targets 
    if(targetsHit[i]){                  //if the target 
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
/*
 * For the GameOver function, the laser is turned off, the LCD is cleared, and the program is locked into a while loop. Inside of this loop, are the 
 * calculations for a tiny animation showing us that the game has ended.
 */
//----------------------------------------------------------------------------------------------------------------------------------------------------

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

//----------------------------------------------------------------------------------------------------------------------------------------------------
/*
 * This moveSERVOS function positions the servos with respect to the positions that we want to move. The positions are calculated on the findAngles
 * function.
 */
//----------------------------------------------------------------------------------------------------------------------------------------------------

void moveSERVOS(void){
  int square = 15;                        //move through all of the positions
  for(double i = 0; i < square; i++){     
      //Write the hardcoded positions then oscillate around the target in both
      //directions
      xAxis.write((x[returnedPOS[0]+returnedPOS[1]*4 + 1] + i*pow(-1,i)/5.00));
      yAxis.write((y[returnedPOS[0]+returnedPOS[1]*4 + 1] + i*pow(-1,i)/5.00));
      delay(100);//delay for a 10th of a second
      //check if one of the targets were hit
      checkHIT();
      set7SEGDISP();
      checkVal = allTargetsHit; //Update the check value

  }
  delay(20);
  Serial.print("Going to :\t");
  Serial.println(returnedPOS[0]+returnedPOS[1]*4); //Show the positions
  xAxis.write(x[returnedPOS[0]+returnedPOS[1]*4+1]);  //move the the respectiove positions
  yAxis.write(y[returnedPOS[0]+returnedPOS[1]*4+1]);

  //Removed autocalculated positions
//  xAxis.write(findAngles(boardDistance,returnedPOS[0],returnedPOS[1], posofLaser[0], posofLaser[1], space, 90));
//  yAxis.write(findAngles(boardDistance,returnedPOS[1],returnedPOS[0], posofLaser[1], posofLaser[0], space, 90));
}

//----------------------------------------------------------------------------------------------------------------------------------------------------
/*
 * This findAngles function finds the angles in degrees that the servo needs to go to so that the laser can point at the photoresistors.
 */
//----------------------------------------------------------------------------------------------------------------------------------------------------

//find the angle of sensor
double findAngles(double distBoard, double POS_laser_A, double POS_laser_B, double POS_A, double POS_B, double sensors_space,double offsets){
  return (offsets - atan2(POS_laser_A - sensors_space*POS_A, sqrt( pow(distBoard,2) + pow(POS_laser_B - sensors_space*POS_B,2) ))*180/PI);
}

//----------------------------------------------------------------------------------------------------------------------------------------------------
/*
 * This selectPosition function simply allows the Standby Operator (this program) to increase, decrease, or send the position depending on the button
 * that is being pressed at the moment. It only supports having one button pressed at a time. In other words, if we press a button then keep holding
 * that button down and press another button, then the program will not read the other button that is being pressed. This function could have been 
 * more efficient if we convert each digital value with a number containing its binary representation but this function works fine. Two buttons that 
 * may be pressed and registered are the position buttons and the send button. If the position button and the send button are pressed together, then
 * the position button is released while the send button is still being pressed, then the program should send over the information to the Master 
 * Operator within this function. Whether it happens on this loop or in another loop would not make a significant difference to our program.
 */
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
/*
 * In the checkHIT function, the program checks each position that we have randomly generated then sets the bool array of 'targetsHIT' to high 
 * depending on if a target has been hit. If it has not been processed previously, this function may also keep track of the total number
 * of targets hit. That number is stored on the allTargetsHit integer. 
 */
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
/*
 * This set7SEGDISP function was obtained from the arduino website. It is a very efficient use of the seven segment display and is therefore used
 * in this project. A link to this function along with the other corresponding functions is provided towards the global variable initializations 
 * section.
 */
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
/*
 * in myfnUpdateDisplay the user obtains the eight bits that are required for the seven segment display as an input. It does not return any value, 
 * but it does set the respective value onto the seven segment display. Notice how the function required turning off the latchPin then setting the 
 * new value and turning on the latchPin again after the value has been set. This will update the value to the new output.
 */
//----------------------------------------------------------------------------------------------------------------------------------------------------

void myfnUpdateDisplay(byte eightBits) {
  digitalWrite(latchPin, LOW);  // prepare shift register for data
  shiftOut(dataPin, clockPin, LSBFIRST, eightBits); // send data
  digitalWrite(latchPin, HIGH); // update display
}

//----------------------------------------------------------------------------------------------------------------------------------------------------
/*
 * In the myfnNumToBits function, depending on the integer number from 0 to 15 in hex, the function will return the respective number in bits 
 * to be set to the 7 Segment Display.
 */
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
/*
 * This is the main function. This function loops rapidly through all of the programs so that out input will be seemingly instant and the input of the
 * master operator will mostly depend on the UART communication between the ESP32 and the Arduino MEGA. The first function selectPosition simply 
 * allows this program to listen to any inputs from the buttons to check if we would like the Master Operator to change the position of the laser. The
 * lcdPRINT function prints those values onto the LCD. The if statement checks if the Master Operator has returned anything, and if the Master Operator
 * has, then the returnedPOS or returned positions are calculated depending on the number that was recieved. That output is then displayed onto the 
 * Serial Monitor for debugging purposes. In the moveSERVOS function, we use the positions that were gathered from the master operator. Then we check 
 * if any target was hit, then update the 7 Segment Display to display the number of targets that are down.
 */
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
    returnedPOS[1] = (temp - returnedPOS[0])/4;
//    Serial.print("Row: ");
//    Serial.println(returnedPOS[1]);
//    Serial.print("Colunm: ");
//    Serial.println(returnedPOS[0]);

    //The servos only listen to returned POS
    moveSERVOS();
  }
  
  //check if one of the targets were hit
  checkHIT();

  set7SEGDISP();

  checkVal = allTargetsHit;
} //End MAIN loop
