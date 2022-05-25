/*
ACTIVE LAYER DATALOGGER
//Old version: wakes up with Arduino clock, not with DS3231 alarm.

Version: 24/05/2022
Power consumption: Sleeping 0.005 mah and saving data 120 mah (during 500 miliseconds)
A 3000 mah 18650 battery should last more than a year saving data at 30 minutes interval (indeed in a wondeful world should last almost 3 years!!)
http://oregonembedded.com/batterycalc.htm

HARDWARE:
a-Arduino Pro Mini 3.3v
b-Micro sd card and reader
c-DS3231 Clock
d-Printed board circuit Yanacona v1.4
e-NPN transistor (2n222a)
f-PNP transistor (2n3906)
g-Three 100 ohms resistors (one for green Led)
h-One 4700 k resistor
i-Green led 
j-Capacitor 330 uf
k-3/4" Tube
m- DS18B20 sensors

Data is saved in CSV format without headers (logger run outs of SRAM with more than 20 string columns)

-Column 1: UnixTime
-Columns 2-X: DS18B20 data
-Column X1: Temp DS3231 Clock
-Column X2: Battery Voltage

#The log interval is set by default to 30 minutes.  It can be changed from settings.txt file and restarting the logger (no need to upload de code again!)
#The loggers wakes up with the Arduino internal clock, so the time shifts some seconds in each record.
#Datetime is saved in unix format to avoid time zone issues

#Hardware power saving strategies

-Remove led resistance and voltage regulator of Arduino Pro Mini 3.3v
-Remove led resistance and RP1 resistance package of DS3231 Clock. 

-Green blinks first time if all is ready to save data and blinks when data is recorded

*/

//LIBRARIES
#include <JeeLib.h> //Low power consumption
#include <OneWire.h> //Temperature
#include <DallasTemperature.h> //Temperature
#include <Wire.h> //Clock
#include <SPI.h> //SD card
#include "SdFat.h"
#include <stdint.h>
#include "EEPROM.h"    
#include "RTClib.h"

//SERIAL

//#define ECHO_TO_SERIAL //Uncomment for testing and comment for field deployment

//THERMOMETER
//Pin 
// Data wire is plugged into pin 3 on the Arduino
#define ONE_WIRE_BUS 4

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

////////////////////////////////////////////////////////////////////////
//LIST OF SENSORS. THIS IS THE ONLY REQUIRED USER INPUT///

DeviceAddress listSensors[]= {
  {0X28, 0X60, 0X55, 0X8E, 0X10, 0X20, 0X6, 0X0},
{0X28, 0X28, 0XE8, 0X24, 0X11, 0X20, 0X6, 0X6E},
{0X28, 0XD8, 0XD5, 0XB2, 0X11, 0X20, 0X6, 0X9C},
{0X28, 0X78, 0X98, 0X26, 0X11, 0X20, 0X6, 0X2A},
{0X28, 0X94, 0X4F, 0X98, 0X10, 0X20, 0X6, 0X5E},
{0X28, 0X54, 0XEC, 0X8E, 0X10, 0X20, 0X6, 0X8D},
{0X28, 0XD4, 0XC0, 0X97, 0X10, 0X20, 0X6, 0X15},
{0X28, 0XB4, 0X8E, 0XAA, 0X11, 0X20, 0X6, 0XE3},
{0X28, 0X74, 0XDC, 0XA6, 0X10, 0X20, 0X6, 0XD3},
{0X28, 0X74, 0XA9, 0X1B, 0X11, 0X20, 0X6, 0XCB},
{0X28, 0XCC, 0X9D, 0X76, 0X11, 0X20, 0X6, 0XFB},
{0X28, 0X5C, 0XEB, 0X1A, 0X11, 0X20, 0X6, 0X39},
{0X28, 0X12, 0XDA, 0X2F, 0X11, 0X20, 0X6, 0XDF},
{0X28, 0XD2, 0X52, 0X30, 0X11, 0X20, 0X6, 0X17},
{0X28, 0X32, 0X67, 0XAA, 0X11, 0X20, 0X6, 0X92},
{0X28, 0X72, 0X18, 0X1C, 0X11, 0X20, 0X6, 0X7D},
{0X28, 0XA, 0XDF, 0X5E, 0X10, 0X20, 0X6, 0X7B},
{0X28, 0X8A, 0XEE, 0X7B, 0X10, 0X20, 0X6, 0X29},
{0X28, 0X4A, 0X10, 0XB, 0X11, 0X20, 0X6, 0XB1},
{0X28, 0X86, 0X3D, 0X94, 0X10, 0X20, 0X6, 0X97},
{0X28, 0X66, 0X95, 0X8A, 0X11, 0X20, 0X6, 0X35},
{0X28, 0XE6, 0X3F, 0X91, 0X10, 0X20, 0X6, 0X56},
{0X28, 0X36, 0X90, 0X12, 0X11, 0X20, 0X6, 0X34},
{0X28, 0XE, 0XCA, 0X5E, 0X10, 0X20, 0X6, 0X9},
{0X28, 0X8E, 0X7F, 0X94, 0X10, 0X20, 0X6, 0X5C},
{0X28, 0X9E, 0XE1, 0XAF, 0X11, 0X20, 0X6, 0X65},
{0X28, 0X9, 0X24, 0X97, 0X10, 0X20, 0X6, 0X5E},
{0X28, 0X9, 0X79, 0X7, 0X11, 0X20, 0X6, 0X6D},
{0X28, 0X85, 0X9F, 0X7D, 0X10, 0X20, 0X6, 0XA4},
{0X28, 0XE5, 0XE, 0X9, 0X11, 0X20, 0X6, 0X10},
{0X28, 0X15, 0XA3, 0X29, 0X11, 0X20, 0X6, 0XCD},
{0X28, 0X8D, 0X94, 0X16, 0X11, 0X20, 0X6, 0XDA},
{0X28, 0X8D, 0X2C, 0X35, 0X11, 0X20, 0X6, 0X53},
{0X28, 0XCD, 0XC6, 0X32, 0X11, 0X20, 0X6, 0XC7},
{0X28, 0X1D, 0XE4, 0X89, 0X10, 0X20, 0X6, 0XD6},
{0X28, 0X1D, 0X7E, 0X19, 0X11, 0X20, 0X6, 0X96},
{0X28, 0X13, 0X58, 0X15, 0X11, 0X20, 0X6, 0XF3},
{0X28, 0X33, 0XD6, 0X60, 0X10, 0X20, 0X6, 0XAE},
{0X28, 0X33, 0X95, 0X5E, 0X10, 0X20, 0X6, 0X54},
{0X28, 0XB3, 0X43, 0XA, 0X11, 0X20, 0X6, 0X1},
{0X28, 0X73, 0X7A, 0X7E, 0X10, 0X20, 0X6, 0XE4},
{0X28, 0X4B, 0X8D, 0X2, 0X11, 0X20, 0X6, 0X4E},
{0X28, 0XBB, 0XD2, 0X14, 0X11, 0X20, 0X6, 0XF7},
{0X28, 0XBB, 0X87, 0X15, 0X11, 0X20, 0X6, 0X3F},
{0X28, 0XDF, 0XE8, 0X11, 0X11, 0X20, 0X6, 0X50},
{0X28, 0XDF, 0X6E, 0X94, 0X10, 0X20, 0X6, 0XF4},
{0X28, 0X3F, 0XF9, 0XA2, 0X10, 0X20, 0X6, 0X7A},
{0X28, 0X7F, 0X31, 0X7F, 0X10, 0X20, 0X6, 0X8F},
{0X28, 0X7F, 0X7D, 0X16, 0X11, 0X20, 0X6, 0XEF},
{0X28, 0XFF, 0X9, 0XA3, 0X11, 0X20, 0X6, 0X67}
 }; 
//////////////////////////////////////////////////////////////////////////////      

int  numberSensors; 

//CSV FILE NAME STORED IN EEPROM

char loggerID[30];  

//SETTING LOGGING INTERVAL AND VARIABLES TO ADJUST TIME

long LogInterval;     

//POWER

const int power_GRD_NPN = 3; //NPN-from right to left (looking to flat face of transistor 2n2222a) GRD sensor, resistance 100 ohms to pin 3, GRD battery
const int power_VCC_PNP = 2; //PNP from right to left (looking to flat face of transistor 2n3906)VCC battery,resistance 100 ohms to pin 2, VCC sensor

//BATTERY
int battVolts;
float averageVolt;
float minBattery = 3.3; //Treshold of minimum battery to stop recording data
const float batteryOffset = 0.00; //Offset between multimeter and arduino measured battery

//CLOCK (DS3231)
// SDA A4 , SCL A5
RTC_DS3231 rtc;

//SD CARD
//MOSI = 11; MISO=12;CLK=13;CS=10

SdFat SD;
const int CS = 10;
const int MOSIPin = 11;
const int MISOPin = 12;
const int SCKPin = 13;
File mySensorData;

//GREEN LED SD OK

int ledPin = A1;

//LOW POWER
ISR(WDT_vect) {
  Sleepy::watchdogEvent();  // Setup the watchdog
}
void setup() {

Serial.begin(9600); 

//Start i2c communication
Wire.begin();

//LED
  pinMode(ledPin, OUTPUT);

//LOW POWER
pinMode(power_GRD_NPN, OUTPUT); //Controlan los transistores
pinMode(power_VCC_PNP, OUTPUT);

digitalWrite(power_GRD_NPN, HIGH); //NPN transistor are on when HIGH
digitalWrite(power_VCC_PNP, LOW); //PNP transistor are on when LOW

//SD CARD
pinMode(CS, OUTPUT);
pinMode(MOSIPin, OUTPUT);
pinMode(MISOPin, INPUT);
pinMode(SCKPin, OUTPUT);

//Begin communication with ds18b20 sensors
sensors.begin();
numberSensors= sizeof(listSensors) / sizeof(listSensors[0]);

Initialize_SDcard();
GetSettings();
 
}

void loop()
{

  //POWER TRANSISTORS TO CHECK BATTERY

  digitalWrite(power_GRD_NPN, HIGH); //NPN transistor are on when HIGH
  digitalWrite(power_VCC_PNP, LOW); //PNP transistor are on when LOW
  delay(100); //Give time for current to flow properly


  //CHECKING BATTERY. WE CALCULATE THE AVERAGE OF TWENTY VOLTAGE MEASUREMENTS!!

  float sum = 0;

  for (int i = 0; i <= 20; i++)

  {
    
    battVolts = getBandgap();
    sum = sum + battVolts;
  }

   averageVolt = ((sum/20)/100)+batteryOffset;

  //IF BATTERY IS OK

  
  if (averageVolt >= minBattery)//If battery voltage is higher than predefined voltage (3.3)

  {
    
    //LOG DATA

    logData();

  }

  //If battery is low print LOW BATTERY and sleep

  else

  {

 #ifdef ECHO_TO_SERIAL
 Serial.print(F(" Low_battery: "));
 Serial.print(averageVolt);
 Serial.println(F(" volts"));
 Serial.print(F("Going to sleep to protect battery and sd card!!"));
 Serial.flush();
 #endif
 
 //TURN OFF TRANSISTORS (turn of current)
    
 digitalWrite(power_GRD_NPN, LOW); //NPN transistors are off when LOW
 digitalWrite(power_VCC_PNP, HIGH); //PNP transistors are off when HIGH
 Sleepy::powerDown();
 
  }

}
///////////////////FUNCTIONS////////////////////////////////

//FUNCTION TO CHECK BATTERY VOLTAGE

int getBandgap(void)
{
  const long InternalReferenceVoltage = 1050L;  // Adust this value to your specific internal BG voltage x1000
  // REFS1 REFS0          --> 0 1, AVcc internal ref.
  // MUX3 MUX2 MUX1 MUX0  --> 1110 1.1V (VBG)
  ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0);
  // Start a conversion
  ADCSRA |= _BV( ADSC );
  // Wait for it to complete
  while ( ( (ADCSRA & (1 << ADSC)) != 0 ) );
  // Scale the value
  int results = (((InternalReferenceVoltage * 1024L) / ADC) + 5L) / 10L;
  return results;
}

//FUNCTION TO SAVE DATA IN MICRO SD CARD AND PRINT DATA IN SERIAL

void logData()
{

  digitalWrite(power_GRD_NPN, HIGH); //NPN transistor are on when HIGH
  digitalWrite(power_VCC_PNP, LOW); //PNP transistor are on when LOW
  delay(50); //Give time for current to flow properly

   //THERMOMETER
  sensors.begin();
  sensors.requestTemperatures();

  //Clock
  rtc.begin();
  DateTime now = rtc.now(); //Set the clock first running the RTC 3231 example. Then run this code
  
  //SD CARD
 
  SD.begin(CS);
  delay(50);
  mySensorData = SD.open(loggerID, O_CREAT | O_APPEND | O_WRITE);
  while (mySensorData)
  {

    //Unix time to avoid time zone issues
    
    mySensorData.print(now.unixtime());
    mySensorData.print(",");

    //Saving ds18b20 data
    
    for (uint8_t i=0;i<numberSensors;i++)
    {
      
      mySensorData.print(sensors.getTempC(listSensors[i])); 
      mySensorData.print(","); 
      
    } 
  
    //Clock temperature
    mySensorData.print(rtc.getTemperature());
    mySensorData.print(",");
     
     //Battery Voltage
    mySensorData.println(averageVolt);
    mySensorData.close();
  
     #ifdef ECHO_TO_SERIAL

    //PRINT DATA IN SERIAL

    Serial.println(F("Current date, time and data"));

    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(',');   
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.print(",");

    //Printing ds18b20 data

    for (uint8_t i=0;i<numberSensors;i++)
    {

      Serial.print(sensors.getTempC(listSensors[i])); 
      Serial.print(","); 
      
    } 

    //Temp Clock
    Serial.print(rtc.getTemperature());
    Serial.print(F(","));

    //Battery Voltage
    Serial.println(averageVolt);
    Serial.flush();

    #endif

    //PULL DOWN SPI PINS TO SAVE POWER

    digitalWrite(SCKPin, LOW);
    digitalWrite(CS, LOW);
    digitalWrite(MOSIPin, LOW);
    digitalWrite(MISOPin, LOW);
    SPI.end();
    Wire.end();

    //GIVE TIME TO SAVE DATA
    delay(250); 

  //TURN OFF TRANSISTORS (turn of current)
    
    digitalWrite(power_GRD_NPN, LOW); //NPN transistors are off when LOW
    digitalWrite(power_VCC_PNP, HIGH); //PNP transistors are off when HIGH

    //SETTING TIMING TO LOG DATA
    for (byte i = 0; i < LogInterval; ++i)
    {
   // Change number after < to indicate the interval in minutes

      Sleepy::loseSomeTime(60000);// Change the number in parenthesis to 60000 to measure minutes
   
  }
    
  }
}

//FUNCTION TO CHECK SD CARD AND WRITE COLUMN HEADERS

void Initialize_SDcard()
{

 //TURN ON TRANSISTORS

  digitalWrite(power_GRD_NPN, HIGH); 
  digitalWrite(power_VCC_PNP, LOW); 
  delay(100); //Give time for current to flow properly

//CHECKING BATTERY VOLTAGE

  float sum = 0;

  for (int i = 0; i <= 20; i++)

  {
    
    battVolts = getBandgap();
    sum = sum + battVolts;
  }

   averageVolt = ((sum/20)/100)+batteryOffset;

   //CHECK IF CARD IS PRESENT AND CAN BE INITIALIZED
    
  if (!SD.begin(CS)) 
  {
    #ifdef ECHO_TO_SERIAL
    Serial.println(F("Card failed!!"));
    #endif
  }

  else if (averageVolt<minBattery)
  
  {

    #ifdef ECHO_TO_SERIAL
    Serial.println(F("Low Battery!!= "));
    Serial.print(averageVolt);
    Serial.print(F("v"));
    #endif
    
    }
  
  else 
  
  {

     #ifdef ECHO_TO_SERIAL
    Serial.println(F("Sd Card Ready and Battery Ok!!"));
     #endif
     
    blinkyOkBatterySD();
    writeSettings();
    LoggerId();

/*

  //ADDING HEADERS TO CSV FILE. UNCOMMENT ONLY IF THERE ARE LESS THAN 20 COLUMNS WITH SHORT NAMES

     File dataFile = SD.open(loggerID, FILE_WRITE);
      
  // if file is available, write headers
  
     if (dataFile) {
    dataFile.println("UnixTime");
    dataFile.close();
    }

*/
     ////PULL DOWN SPI PINS TO SAVE POWER
     
    digitalWrite(SCKPin, LOW);
    digitalWrite(CS, LOW);
    digitalWrite(MOSIPin, LOW);
    digitalWrite(MISOPin, LOW);
    SPI.end();
  
  }

}

//BLINK LED IF CARD IS READY AND BATTERY OK

void blinkyOkBatterySD()
 {
for (int i = 0; i < 2; i++)
  {
    analogWrite(ledPin, 255);
    delay(500);
    analogWrite(ledPin, 0);
    delay(50);
  }
}

//OBTAINING DATA FROM SETTINGS FILE  settings.txt 

void GetSettings() 
   {
      
       // open settings file
       SdFile SettingsFile ("settings.txt", O_READ);
           
       for (byte i=0; i<=2; i++)  {  
              
         char line[30];
         SettingsFile.fgets(line,30);// read one line
         char* p_pos = strchr(line, '=');// find the '=' position
         
         if(i==0)  LogInterval = atoi(p_pos+1);
      
       }
          
           SettingsFile.close();    
           delay(50);    
    }

// CREATING SETTINGS FILE

 void writeSettings()

 {
  
  File settingsCSV;
  settingsCSV = SD.open("settings.txt");

  if (settingsCSV) 
  
  {

     #ifdef ECHO_TO_SERIAL
     Serial.println(F("Setting file already exists"));
     #endif

 settingsCSV.close();

  } 

else {

  settingsCSV = SD.open("settings.txt", O_CREAT | O_APPEND | O_WRITE);
  
  // Write log interval

  settingsCSV.println("Log interval(minutes)= 30");
  settingsCSV.close();
   
  }
  } 


//READING LOGGER ID STORED IN EEPROM

 void LoggerId()

 {

  int startingAddress = 30;
  for(int n = 0; n < 30; n++)
  
  {
     loggerID[n] = EEPROM.read(n+startingAddress);     
  }

  } 
