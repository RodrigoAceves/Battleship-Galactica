



//This code is to be uploaded to the ESP32.
// This code receives data from the Arduino Mega onto the ESP32, and sends it to the MQTT Server.


#include <WiFi.h>
#include <PubSubClient.h>

#define RXD2 16    //ESP32 UART2 pins
#define TXD2 17


// Add Wi-Fi and server settings
const char* ssid ="Unknown Host";   // Add your Wi-Fi ssid
const char* pass= "00000000";   // Add Wi-Fi PW
const char* brokerUser= ("RODRIGOACEVES");   // Add the user name you selected from the server page
const char* brokerPass= ("00000000");   // Add the PW you selected from the server page
const char* broker= ("broker.emqx.io");
const char* outTopic = ("thisisatest");    //Add the name you selected for your subscription
const char* inTopic= ("thisisatest");      // Add the name you selected for your subscription

WiFiClient espClient;
PubSubClient client(espClient);


char messages[100];
String MegaData;

void setupwifi()
{
  delay(100);
  Serial.println("\nConnecting to");
  Serial.println(ssid);

  WiFi.begin(ssid, pass);

  while(WiFi.status() != WL_CONNECTED)
  {
    delay(100);
    Serial.print("-");
  }
  Serial.print("\nConnected to");
  Serial.println(ssid);
}

void reconnect(){
  while(!client.connected())
  {
    Serial.print("\nConnecting to");
    Serial.println(broker);
    if(client.connect("Add client ID", brokerUser, brokerPass)){   // Add client ID or Name
      Serial.println("\nConnected to");
      Serial.println(broker);
      client.subscribe(inTopic);
    }else{
      Serial.println("\nTrying connect again");
      delay(5000);
    }
    }
  }

  
void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);  // ESP32 UART 2 pins
  Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2); //Mega UART 1 pins
  setupwifi();
  client.setServer(broker, 1883);
  
}

void loop() 
{
 
  if (!client.connected()){
    reconnect();
  }
  client.loop();
  if(Serial2.available()){      // This portion of the code reads the Arduino Mega serial monitor output through the ESP32 UART2 pins. It then sends the data to the MQTT server.
   String MegaData=Serial2.readString();  // Changes the string to a char and publishes to the server.
   MegaData.toCharArray(messages,10);
   client.publish(outTopic,messages);
   delay(500); 
}

}
