/*
 * Hub to translate MQTT messages into IR commands for Ulisse Argo DCI AC
 * Also PubSubClient for MQTT relay
 * BME280 for P,H,T
 * Copyright K.Schmolders 08/2017
 */

#include "wifi_credentials.h"
#include <IRrecv.h>
#include <IRutils.h>
#include <IRsend.h>
#include <ir_Argo.h> 
//required for MQTT
#include <ESP8266WiFi.h>
//required for OTA updater
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
//end OTA requirements
#include <PubSubClient.h>
#include <Adafruit_BME280.h>


// An IR detector/demodulator is connected to GPIO pin 14 on Sonoff
uint16_t IR_SEND_PIN = 14;
uint16_t BME_SCL = 1;
uint16_t BME_SDA = 3;
IRArgoESP argoir(IR_SEND_PIN);
//IRrecv irrecv(RECV_PIN); //Receiver 
decode_results results;  // Somewhere to store the results
irparams_t save;         // A place to copy the interrupt state while decoding.

//timer
int timer_update_state_count;
int timer_update_state = 60000; //update status via MQTT every minute
int timer_delay_send_count;
int timer_delay_send = 1000; //wait one second before sending
bool delay_send = false;

//MQTT
WiFiClient espClient;
PubSubClient client(espClient);
//all wifi credential and MQTT Server importet through wifi_credential.h

const char* inTopic = "cmnd/ac_argo/#";
const char* outTopic = "stat/ac_argo/";
const char* mqtt_id = "ac_argo";

//BME280
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme280; // I2C
float bme280_temperature, bme280_pressure, bme280_humidity, bme280_height;
float bme280_temp_offset = 1.5;

//OTA
ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
    
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  httpUpdater.setup(&httpServer);
  httpServer.begin();
}


//callback function for MQTT client
void callback(char* topic, byte* payload, unsigned int length) {
  payload[length]='\0'; // Null terminator used to terminate the char array
  String message = (char*)payload;

  Serial.print("Message arrived on topic: [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(message);
  
  //get last part of topic 
  char* cmnd = "test";
  char* cmnd_tmp=strtok(topic, "/");

  while(cmnd_tmp !=NULL) {
    cmnd=cmnd_tmp; //take over last not NULL string
    cmnd_tmp=strtok(NULL, "/"); //passing Null continues on string
    //Serial.println(cmnd_tmp);    
  }
  
  

  if (!strcmp(cmnd, "tset")) {
    Serial.print("Received new temperature setpoint: ");
    Serial.println(message);
    int t_set=message.toInt();
    argoir.setTemp(t_set);
    delay_send=true;
    timer_delay_send_count=millis();
    //argoir.send();
  }
  else if (!strcmp(cmnd, "power")) {
    Serial.print("Received new power command: ");
    Serial.println(message);
    if (message == "ON") {
      
      argoir.on();
      argoir.send();
    }
    else if (message == "OFF") {
      
      argoir.off();
      argoir.send();
    }
    else Serial.println("unknown power state");
  }
  else if (!strcmp(cmnd, "fan")) {
    Serial.print("Received new fan mode: ");
    Serial.println(message);
    argoir.setFan(message.toInt());
    argoir.send();

  }
  else if (!strcmp(cmnd,"coolmode")) {
    Serial.print("Received new cool mode: ");
    Serial.println(message);
    argoir.setCoolMode(message.toInt());
    argoir.send();

  }
  else if (!strcmp(cmnd,"heatmode")) {
    Serial.print("Received new heat mode: ");
    Serial.println(message);
    argoir.setHeatMode(message.toInt());
    argoir.send();

  }  
  printACStatus();
  sendACStatus();
}

//print Status to Serial
void printACStatus(){
  Serial.println("New AC Device Status");
  Serial.print("AC Power: ");
  Serial.println(argoir.getPower());
  Serial.print("Set Temp: ");
  Serial.println(argoir.getTemp());
  Serial.print("AC Mode: ");
  Serial.println(argoir.getMode());
  Serial.print("Cooling Mode: ");
  Serial.println(argoir.getCoolMode());
  Serial.print("Heating Mode: ");
  Serial.println(argoir.getHeatMode());
  Serial.print("AC Fan: ");
  Serial.println(argoir.getFan());

  Serial.println("Sensor Values");
  Serial.print("Temperature: ");
  Serial.println(bme280_temperature);
  Serial.print("Humidity: ");
  Serial.println(bme280_humidity);
  Serial.print("Pressure: ");
  Serial.println(bme280_pressure);
}

//send Status via MQTT
void sendACStatus(){
   
   char outTopic_status[50];
   char msg[50];

   //power
   strcpy(outTopic_status,outTopic);
   dtostrf(argoir.getPower(),1,0,msg);
   strcat(outTopic_status,"power");
   client.publish(outTopic_status, msg);
   Serial.print("Sending MQTT AC Status: ");
   Serial.print(outTopic_status);
   Serial.print(" - ");
   Serial.println(msg);

  //acmode
    strcpy(outTopic_status,outTopic);
   dtostrf(argoir.getMode(),1,0,msg);
   strcat(outTopic_status,"ac_mode");
   client.publish(outTopic_status, msg);
   
  //settemp
   strcpy(outTopic_status,outTopic);
   dtostrf(argoir.getTemp(),1,0,msg);
   strcat(outTopic_status,"tset");
   client.publish(outTopic_status, msg);
   
  //Cool Mode
   strcpy(outTopic_status,outTopic);
   dtostrf(argoir.getCoolMode(),1,0,msg);
   strcat(outTopic_status,"coolmode");
   client.publish(outTopic_status, msg);
  
  //heat mode
   strcpy(outTopic_status,outTopic);
   dtostrf(argoir.getHeatMode(),1,0,msg);
   strcat(outTopic_status,"heatmode");
   client.publish(outTopic_status, msg);

  //fan
   strcpy(outTopic_status,outTopic);
   dtostrf(argoir.getFan(),1,0,msg);
   strcat(outTopic_status,"fan");
   client.publish(outTopic_status, msg);
   
  //roomtemp from BME280
    strcpy(outTopic_status,outTopic);
   dtostrf(bme280_temperature,2,2,msg); 
   strcat(outTopic_status,"troom");
   client.publish(outTopic_status, msg);

  //BME280 Humidity
   strcpy(outTopic_status,outTopic);
   dtostrf(bme280_humidity,2,2,msg); 
   strcat(outTopic_status,"humidity");
   client.publish(outTopic_status, msg);

   //BME280 Pressure
   strcpy(outTopic_status,outTopic);
   dtostrf(bme280_pressure,2,2,msg); 
   strcat(outTopic_status,"pressure");
   client.publish(outTopic_status, msg);

    //IP Address
   strcpy(outTopic_status,outTopic);
   strcat(outTopic_status,"ip_address");
   WiFi.localIP().toString().toCharArray(msg,50);
   client.publish(outTopic_status,msg ); 

}

void reconnect() {
  // Loop until we're reconnected
  
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(mqtt_id)) {
      Serial.println("connected");
      
      client.publish(outTopic, "ac_argo booted");
      
      //send current Status via MQTT to world
      sendACStatus();
      // ... and resubscribe
      client.subscribe(inTopic);

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");      
      delay(5000);
    }
  }
}

void update_sensors() {
  bme280_temperature=bme280.readTemperature()-bme280_temp_offset; //C
  bme280_pressure=bme280.readPressure() / 100.0F; //in hPA
  bme280_humidity=bme280.readHumidity(); //%
  bme280_height=bme280.readAltitude(SEALEVELPRESSURE_HPA); //m

  //update in argoir. but do not send
  argoir.setRoomTemp(int(bme280_temperature));
}

void setup() {
  // Status message will be sent to the PC at 115200 baud
  Serial.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY);
  
  argoir.begin();

  //INIT TIMERS
  timer_update_state_count=millis();
  delay_send=false;

  //INIT BME280
  //SDA, SCL
  Wire.begin(BME_SDA, BME_SCL);
  bool status;
  status = bme280.begin();
  if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring!");
      while (1);
  }
  update_sensors(); 

  //WIFI and MQTT
  setup_wifi();                   // Connect to wifi 
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  //print current Status to Serial 
  printACStatus();  
}


void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  update_sensors();


  //http Updater for OTA
  httpServer.handleClient(); 

  //send status update via MQTT every minute
  if(millis()-timer_update_state_count > timer_update_state) {
   //addLog_P(LOG_LEVEL_INFO, PSTR("Serial Timer triggerd."));
   timer_update_state_count=millis();
   sendACStatus();
   printACStatus();
  }
  if((millis()-timer_delay_send_count > timer_delay_send) && delay_send) {
    Serial.println("Delay send triggerd");
   delay_send=false;
   argoir.send();
   sendACStatus();
   
  }
}
