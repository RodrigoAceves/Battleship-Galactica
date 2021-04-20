// This code is to be uploaded to the Arduino Mega
// This code receives the output from the server via the ESP32 and to the Mega.

String ESPData;

void setup() {
 Serial.begin(9600);
 Serial1.begin(9600);
 for(int i = 0; i < 4; i++){
  pinMode(i+10,OUTPUT);
 }
}

void loop() {

  if(Serial1.available()){      //Checks if there is data in UART1
    ESPData= Serial1.readString();
    Serial.print(ESPData);

    if(ESPData.indexOf("AA")>=0){
      //Makes sure only LED 10 is on
        digitalWrite(10,HIGH);
        for(int i = 0; i < 3; i++){
          digitalWrite(i+11,LOW);
        }
    }
    
    if(ESPData.indexOf("BB")>=0){
      //Makes sure only LED 10 and 11 is on
        for(int i = 0; i < 2; i++){
          digitalWrite(i+10,HIGH);
          digitalWrite(i+12,LOW);
        }
    }
    if(ESPData.indexOf("CC")>=0){
        //Makes sure only LED 10, 11, and 12 is on
        for(int i = 0; i < 3; i++){
          digitalWrite(i+10,HIGH);
        }
        digitalWrite(13,LOW);
    }
    if(ESPData.indexOf("DD")>=0){
      //Makes sure all LEDs are on
      for(int i = 0; i < 4; i++){
          digitalWrite(i+10,HIGH);
        }
    }
  } 
}
