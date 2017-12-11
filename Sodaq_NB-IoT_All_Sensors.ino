
/***************************************************************************
	This is a library for the HTS221 Humidity Temperature Sensor
		Originally written by speirano for SmartEverything
		Adjusted by Gregory Knauff of SODAQ for the NB-IoT shield
	Adjusted by Jan van Loenen to work on Sodaq Explorer and Arduino Leonardo

	Standard I2C-address is 0x5F.

***************************************************************************/

/*****
 * ATT Settings
 *
 * create a new asset as Number
 *
 * device decoding:
 
{
    "sense": 
[
  {
        "asset": "my_temperature",
        "value": {
        "byte": 0,
        "bytelength": 2,
        "type": "integer",
        "calculation": "val / 100"
      }
   },
   {
        "asset": "my_humidity",
        "value": 
        {
          "byte": 2,
          "bytelength": 2,
          "type": "integer",
          "calculation": "val / 100"
        }
   },
   {
        "asset": "my_pressure",
        "value": 
        {
            "byte": 4,
            "bytelength": 2,
            "type": "integer"
        }
    },
    {
        "asset": "my_gps",
        "value": 
        {
          "latitude": 
          {
            "byte": 6,
            "bytelength": 4,
            "type": "integer",
            "calculation": "val / 100000"
          },
          "longitude": 
          {
            "byte": 10,
            "bytelength": 4,
            "type": "integer",
            "calculation": "val / 100000"
          }
        }
    },
    {
       "asset": "bit_status",
       "value": 
       {
         "byte": 14,
         "bytelength": 1,
         "type": "integer"
       }
    }
 ]
 }
*/

#include <Arduino.h>
#include <Wire.h>
#include <Sodaq_nbIOT.h>
 // #include <SoftwareSerial.h> // Uno
#include "Sodaq_HTS221.h"
#include "Sodaq_LPS22HB.h"
#include "Sodaq_UBlox_GPS.h"

#if defined(ARDUINO_AVR_LEONARDO)
#define DEBUG_STREAM Serial 
#define MODEM_STREAM Serial1

#elif defined(ARDUINO_AVR_UNO)
SoftwareSerial softSerial(10, 11); // RX, TX
// You can connect an uartsbee or other board (e.g. 2nd Uno) to connect the softserial.
#define DEBUG_STREAM softSerial 
#define MODEM_STREAM Serial

#elif defined(ARDUINO_SODAQ_EXPLORER)
#define DEBUG_STREAM SerialUSB
#define MODEM_STREAM Serial

#elif defined(ARDUINO_SAM_ZERO)
#define DEBUG_STREAM SerialUSB
#define MODEM_STREAM Serial1

#else
#error "Please select a Sodaq ExpLoRer, Arduino Leonardo or add your board."
#endif

#define L1_1_PIN 2
#define L2_1_PIN 3
#define L3_1_PIN 8
#define L1_2_PIN 9
#define L2_2_PIN 12
#define L3_2_PIN A0

Sodaq_nbIOT nbiot;
Sodaq_LPS22HB lps22hb;

uint32_t lat = 0;
uint32_t lon = 0;
uint16_t loopcount = 0;
uint16_t maxLoopcount = 12 *300*2; // 1 uur 
//uint16_t maxLoopcount = 120*2; // 1 min (60 x 500ms)
//uint16_t maxLoopcount = 20*2; // 10 sec
int delayMS = 500;

boolean L1_1_Hold = false;
boolean L2_1_Hold = false;
boolean L3_1_Hold = false;
boolean L1_2_Hold = false;
boolean L2_2_Hold = false;
boolean L3_2_Hold = false;

int byteCount = 15;
byte message[15];
byte  byte_state = 0;
byte  byte_state_tmp = 0;
boolean gps_fix = false;

void setup();
bool connectToNetwork();
void initHumidityTemperature();
void initPressureSensor();
void initGPS();
void initPINS();
void handleINPUTS(int cursor);
void loop();
void do_flash_led(int pin);

void initPINS(){
  pinMode(L1_1_PIN, INPUT_PULLUP);
  pinMode(L2_1_PIN, INPUT_PULLUP);
  pinMode(L3_1_PIN, INPUT_PULLUP);
  pinMode(L1_2_PIN, INPUT_PULLUP);
  pinMode(L2_2_PIN, INPUT_PULLUP);
  pinMode(L3_2_PIN, INPUT_PULLUP);
}

void setup()
{
	pinMode(13, OUTPUT);
	digitalWrite(13, LOW);

	DEBUG_STREAM.begin(9600);
	MODEM_STREAM.begin(nbiot.getDefaultBaudrate());

	while ((!DEBUG_STREAM) && (millis() < 10000)) {
		// Wait for serial monitor for 10 seconds
	}

	DEBUG_STREAM.println("\r\nSODAQ All Things Talk Arduino Example\r\n");

	nbiot.init(MODEM_STREAM, 7);
	nbiot.setDiag(DEBUG_STREAM);

	delay(2000);

	while (!connectToNetwork());

	initHumidityTemperature();
	initPressureSensor();
	initGPS();
  initPINS();
  readSensors();
  
	digitalWrite(13, LOW);
}

bool connectToNetwork() {
	if (nbiot.connect("oceanconnect.t-mobile.nl", "172.16.14.22", "20416")) {
		DEBUG_STREAM.println("Connected succesfully!");
		return true;
	}
	else {
		DEBUG_STREAM.println("Failed to connect!");
		delay(2000);
		return false;
	}
}

void initHumidityTemperature() {
	if (hts221.begin() == false)
	{
		DEBUG_STREAM.println("Error while retrieving WHO_AM_I byte...");
		while (1);
	}
}

void initPressureSensor() {
	lps22hb.begin(0x5D);	// 

	if (lps22hb.whoAmI() == false)
	{
		DEBUG_STREAM.println("Error while retrieving WHO_AM_I byte...");
	}
}

void initGPS() {
	sodaq_gps.init(6);
	// sodaq_gps.setDiag(DEBUG_STREAM);
}


void readSensors(){
        // Create the message
      
      uint16_t cursor = 0;
      int16_t temperature;
      int16_t humidity;
      int16_t pressure;

    
      
      temperature = hts221.readTemperature() * 100;
      DEBUG_STREAM.println("Temperature x100 : " + (String)temperature);
      message[cursor++] = temperature >> 8;
      message[cursor++] = temperature;
    
      delay(100);
    
      humidity = hts221.readHumidity() * 100;
      DEBUG_STREAM.println("Humidity x100 : " + (String)humidity);
      message[cursor++] = humidity >> 8;
      message[cursor++] = humidity;
    
      delay(100);
    
      pressure = lps22hb.readPressure();
      DEBUG_STREAM.println("Pressure:" + (String)pressure);
      message[cursor++] = pressure >> 8;
      message[cursor++] = pressure;
    
      uint32_t start = millis();
      uint32_t timeout = 1UL * 10 * 1000; // 10 sec timeout
      
      /*DEBUG_STREAM.println(String("waiting for fix ..., timeout=") + timeout + String("ms"));
      if (sodaq_gps.scan(true, timeout)) {
    
        lat = sodaq_gps.getLat() * 100000;
        lon = sodaq_gps.getLon() * 100000;
        gps_fix = true;
        DEBUG_STREAM.println("GPS FIX true");
      }
      else {
      */
      DEBUG_STREAM.println("No Fix");
        lat = 52.39259 * 100000;
        lon = 4.66719 * 100000;
        DEBUG_STREAM.print("lat = ");
        DEBUG_STREAM.println(lat);
        DEBUG_STREAM.print("lon = ");
        DEBUG_STREAM.println(lon);
        gps_fix = false;
        DEBUG_STREAM.println("GPS FIX false");
      /*
      }
      */
      message[cursor++] = lat >> 24;
      message[cursor++] = lat >> 16;
      message[cursor++] = lat >> 8;
      message[cursor++] = lat;
    
    
      message[cursor++] = lon >> 24;
      message[cursor++] = lon >> 16;
      message[cursor++] = lon >> 8;
      message[cursor++] = lon;
}

void printMessage(){
    // Print the message we want to send
    // DEBUG_STREAM.println(message);
    for (int i = 0; i < byteCount; i++) {
    if (message[i] < 0x10) {
      DEBUG_STREAM.print("0");
    }
    DEBUG_STREAM.print(message[i], HEX);
      if(i < (byteCount-1)){
      DEBUG_STREAM.print(":");
      }
    }
    DEBUG_STREAM.println();
}



void loop()
{
  
   if( loopcount == maxLoopcount) {   

      readSensors();
    	do_flash_led(13);
      DEBUG_STREAM.print("The Sensors    ");
      printMessage();
      
      if (nbiot.sendMessage(message, byteCount)){
         DEBUG_STREAM.println("Sensor Message is send.....");
       } else {
         DEBUG_STREAM.println("Sensor Message is not send.....");
         while (!connectToNetwork());
         DEBUG_STREAM.println("Sensor and reconnect.....");
         nbiot.sendMessage(message, byteCount);
         DEBUG_STREAM.println("Sensor And Message send again.....");
       }
      
      loopcount = 0;
   }

   loopcount++;
   handleINPUTS(byteCount-1);

    
    /*
    DEBUG_STREAM.print("byte_state = ");
    DEBUG_STREAM.println(byte_state);

    DEBUG_STREAM.print("byte_state_tmp = ");
    DEBUG_STREAM.println(byte_state_tmp);
    */
    // Send only message when status changed....
     if ( byte_state_tmp != byte_state){
       readSensors();
       DEBUG_STREAM.print("The state change    ");
       printMessage();
       //DEBUG_STREAM.println("sendMessage.....");
       do_flash_led(13);
       if (nbiot.sendMessage(message, byteCount)){
         DEBUG_STREAM.println("Message is send.....");
       } else {
         DEBUG_STREAM.println("Message is not send.....");
         while (!connectToNetwork());
         DEBUG_STREAM.println("and reconnect.....");
         nbiot.sendMessage(message, byteCount);
         DEBUG_STREAM.println("And Message send again.....");
       }
       byte_state_tmp = byte_state;
     }
   
     delay(delayMS);
   
}

void do_flash_led(int pin)
{
	for (size_t i = 0; i < 2; ++i) {
		delay(100);
		digitalWrite(pin, LOW);
		delay(100);
		digitalWrite(pin, HIGH);
		delay(100);
		digitalWrite(pin, LOW);
	}
}

void handleINPUTS(int cursor){

  if ( !digitalRead( L1_1_PIN )){   L1_1_Hold = true; } else {L1_1_Hold = false; }
  if ( !digitalRead( L2_1_PIN )){   L2_1_Hold = true; } else {L2_1_Hold = false; }
  if ( !digitalRead( L3_1_PIN )){   L3_1_Hold = true; } else {L3_1_Hold = false; }
  if ( !digitalRead( L1_2_PIN )){   L1_2_Hold = true; } else {L1_2_Hold = false; }
  if ( !digitalRead( L2_2_PIN )){   L2_2_Hold = true; } else {L2_2_Hold = false; }
  if ( !digitalRead( L3_2_PIN )){   L3_2_Hold = true; } else {L3_2_Hold = false; }
  
  bitWrite(message[cursor],0,L1_1_Hold);
  bitWrite(message[cursor],1,L2_1_Hold);
  bitWrite(message[cursor],2,L3_1_Hold);
  bitWrite(message[cursor],3,L1_2_Hold);
  bitWrite(message[cursor],4,L2_2_Hold);
  bitWrite(message[cursor],5,L3_2_Hold);
  
  //bitWrite(message[cursor],7,gps_fix);
  byte_state = message[cursor];
  
}

