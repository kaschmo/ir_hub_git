/*
 * Combined IR Sender and Receiver
 * Also PubSubClient for MQTT relay
 * BME280 for P,H,T
 */

#include "wifi_credentials.h"
#include <IRrecv.h>
#include <IRutils.h>
#include <IRsend.h>
#include <ir_Argo.h> 
//required for MQTT
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_BME280.h>


// An IR detector/demodulator is connected to GPIO pin 14(D5 on a NodeMCU
uint16_t RECV_PIN = 14;
IRArgoESP argoir(D1);
IRrecv irrecv(RECV_PIN);
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

//BME280
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme280; // I2C
float bme280_temperature, bme280_pressure, bme280_humidity, bme280_height;


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
   
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      
      client.publish(outTopic, "AC Hub booted");
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
  bme280_temperature=bme280.readTemperature(); //C
  bme280_pressure=bme280.readPressure() / 100.0F; //in hPA
  bme280_humidity=bme280.readHumidity(); //%
  bme280_height=bme280.readAltitude(SEALEVELPRESSURE_HPA); //m

  //update in argoir. but do not send
  argoir.setRoomTemp(int(bme280_temperature));
}

void setup() {
  // Status message will be sent to the PC at 115200 baud
  Serial.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY);
  irrecv.enableIRIn();  // Start the receiver
  argoir.begin();

  //INIT TIMERS
  timer_update_state_count=millis();
  delay_send=false;

  //INIT BME280
  //SDA, SCL
  Wire.begin(D3, D4);
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

// Dump out the decode_results structure.
void dumpRaw(decode_results *results) {
  // Print Raw data
  Serial.print("Timing[");
  Serial.print(results->rawlen - 1, DEC);
  Serial.println("]: ");

  for (uint16_t i = 1;  i < results->rawlen;  i++) {
    if (i % 100 == 0)
      yield();  // Preemptive yield every 100th entry to feed the WDT.
    uint32_t x = results->rawbuf[i] * USECPERTICK;
    if (!(i & 1)) {  // even
      Serial.print("-");
      if (x < 1000) Serial.print(" ");
      if (x < 100) Serial.print(" ");
      Serial.print(x, DEC);
    } else {  // odd
      Serial.print("     ");
      Serial.print("+");
      if (x < 1000) Serial.print(" ");
      if (x < 100) Serial.print(" ");
      Serial.print(x, DEC);
      if (i < results->rawlen - 1)
        Serial.print(", ");  // ',' not needed for last one
    }
    if (!(i % 8)) Serial.println("");
  }
  Serial.println("");  // Newline
}

// Dump out the decode_results structure.
//
void dumpCode(decode_results *results) {
  // Start declaration
  Serial.print("uint16_t  ");              // variable type
  Serial.print("rawData[");                // array name
  Serial.print(results->rawlen - 1, DEC);  // array size
  Serial.print("] = {");                   // Start declaration

  // Dump data
  for (uint16_t i = 1; i < results->rawlen; i++) {
    Serial.print(results->rawbuf[i] * USECPERTICK, DEC);
    if (i < results->rawlen - 1)
      Serial.print(",");  // ',' not needed on last one
    if (!(i & 1)) Serial.print(" ");
  }

  // End declaration
  Serial.print("};");  //

  // Comment
  
  Serial.print(" ");
  serialPrintUint64(results->value, 16);

  // Newline
  Serial.println("");

  // Now dump "known" codes
  if (results->decode_type != UNKNOWN) {
    // Some protocols have an address &/or command.
    // NOTE: It will ignore the atypical case when a message has been decoded
    // but the address & the command are both 0.
    if (results->address > 0 || results->command > 0) {
      Serial.print("uint32_t  address = 0x");
      Serial.print(results->address, HEX);
      Serial.println(";");
      Serial.print("uint32_t  command = 0x");
      Serial.print(results->command, HEX);
      Serial.println(";");
    }

    // All protocols have data
    Serial.print("uint64_t  data = 0x");
    serialPrintUint64(results->value, 16);
    Serial.println(";");
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  // Check if the IR code has been received.
  if (irrecv.decode(&results, &save)) {
    Serial.println("Receiving IR data");
    //dumpRaw(&results);            // Output the results in RAW format
    dumpCode(&results);           // Output the results as source code
    Serial.println("");           // Blank line between entries
  }

  update_sensors();


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
