
/*
ACTIVE LAYER DATALOGGER
//New version: Logger wakes up with DS3231 alarm. It saves data every 5 seconds to test logger

Version: 22/07/2021
Power consumption: Sleeping 0.018 mah and saving data 60 mah (during 500 miliseconds)
A 3000 mah 18650 battery should last more than a year saving data at 30 minutes interval (in a wonderful world should last 3 years indeed!!)
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
m- 10 DS18B20 sensors

Data is saved in CSV format without headers (logger run outs of SRAM with more than 20 string columns)

-Column 1: UnixTime
-Columns 2-X: DS18B20 data
-Column X1: Temp DS3231 Clock
-Column X2: Battery Voltage

#The log interval is set by default to 5 seconds.  It can be changed from settings.txt file and restarting the logger (no need to upload de code again!)
#First data is saved the nearest hour. Example: If logger is turned on at 14:20:0 the first record will occurr at 15:0:0
#Datetime is saved in unix format to avoid time zone issues

#Hardware power saving strategies

-Remove led resistance and voltage regulator of Arduino Pro Mini 3.3v
-Remove led resistance and RP1 resistance package of DS3231 Clock. 

-Green blinks first time if all is ready to save data and blinks when data is recorded

*/

//LIBRARIES

#include <OneWire.h> //Temperature
#include <DallasTemperature.h> //Temperature
#include <Wire.h> //Clock
#include <SPI.h> //SD card
#include "SdFat.h"
#include <stdint.h>
#include "EEPROM.h"
#include <DS3232RTC.h>       
#include <LowPower.h>
#include "RTClib.h"
#include "PinChangeInterrupt.h"


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

 {0x28, 0xB4, 0x05, 0x95, 0xF0, 0x01, 0x3C, 0x23},
{0x28, 0x81, 0xB7, 0x95, 0xF0, 0x01, 0x3C, 0xE9},
{0x28, 0xFF, 0xA6, 0x95, 0xF0, 0x01, 0x3C, 0xD3},
{0x28, 0x84, 0x46, 0x95, 0xF0, 0x01, 0x3C, 0x69},
{0x28, 0xF7, 0x7D, 0x95, 0xF0, 0x01, 0x3C, 0x5C},
{0x28, 0xDF, 0xE0, 0x95, 0xF0, 0x01, 0x3C, 0x10},
{0x28, 0x7A, 0x3E, 0x95, 0xF0, 0x01, 0x3C, 0x5B},
{0x28, 0x5D, 0x41, 0x95, 0xF0, 0x01, 0x3C, 0x6A},
{0x28, 0x1C, 0xB6, 0x95, 0xF0, 0x01, 0x3C, 0xDF},
{0x28, 0x9B, 0x2F, 0x95, 0xF0, 0x01, 0x3C, 0xF4},
{0x28, 0x67, 0xE3, 0x95, 0xF0, 0x01, 0x3C, 0xF8},
{0x28, 0xDD, 0x5E, 0x95, 0xF0, 0x01, 0x3C, 0x93},
{0x28, 0xE0, 0x5A, 0x95, 0xF0, 0x01, 0x3C, 0x2B},
{0x28, 0x93, 0xA2, 0x95, 0xF0, 0x01, 0x3C, 0x72},
{0x28, 0xAA, 0x94, 0x95, 0xF0, 0x01, 0x3C, 0x11},
{0x28, 0xDE, 0xA4, 0x95, 0xF0, 0x01, 0x3C, 0xD1},
{0x28, 0x6D, 0x8E, 0x95, 0xF0, 0x01, 0x3C, 0xCA},
{0x28, 0x85, 0xD0, 0x95, 0xF0, 0x01, 0x3C, 0x75},
{0x28, 0x68, 0xE2, 0x95, 0xF0, 0x01, 0x3C, 0x11},
{0x28, 0x17, 0x9C, 0x95, 0xF0, 0x01, 0x3C, 0x62},
{0x28, 0x13, 0x58, 0x95, 0xF0, 0x01, 0x3C, 0x83},
{0x28, 0x18, 0xD6, 0x95, 0xF0, 0x01, 0x3C, 0x12},
{0x28, 0x15, 0x72, 0x95, 0xF0, 0x01, 0x3C, 0x74},
{0x28, 0xB2, 0xD8, 0x95, 0xF0, 0x01, 0x3C, 0x23},
{0x28, 0x79, 0x5D, 0x95, 0xF0, 0x01, 0x3C, 0x5D},
{0x28, 0xD0, 0x26, 0x95, 0xF0, 0x01, 0x3C, 0x8A},
{0x28, 0x64, 0xB3, 0x95, 0xF0, 0x01, 0x3C, 0x34},
{0x28, 0x6A, 0x6E, 0x95, 0xF0, 0x01, 0x3C, 0x95},
{0x28, 0x99, 0x47, 0x95, 0xF0, 0x01, 0x3C, 0xB5},
{0x28, 0xF9, 0x1E, 0x95, 0xF0, 0x01, 0x3C, 0x10},
{0x28, 0x0F, 0xEC, 0x95, 0xF0, 0x01, 0x3C, 0xF5},
{0x28, 0xDA, 0x9C, 0x95, 0xF0, 0x01, 0x3C, 0xB7},
{0x28, 0x77, 0xCE, 0x95, 0xF0, 0x01, 0x3C, 0xB7},
{0x28, 0xE1, 0x3C, 0x95, 0xF0, 0x01, 0x3C, 0x91},
{0x28, 0x79, 0xF3, 0x95, 0xF0, 0x01, 0x3C, 0xCC},
{0x28, 0x66, 0x63, 0x95, 0xF0, 0x01, 0x3C, 0x04},
{0x28, 0x60, 0x66, 0x95, 0xF0, 0x01, 0x3C, 0x64},
{0x28, 0x24, 0x8D, 0x95, 0xF0, 0x01, 0x3C, 0x67},
{0x28, 0x67, 0x21, 0x95, 0xF0, 0x01, 0x3C, 0x59},
{0x28, 0x43, 0xE1, 0x95, 0xF0, 0x01, 0x3C, 0x11},
{0x28, 0xFB, 0x02, 0x95, 0xF0, 0x01, 0x3C, 0x23},
{0x28, 0xCB, 0xF9, 0x95, 0xF0, 0x01, 0x3C, 0x18},

          
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
const byte rtcAlarmPin = 5; // External interrupt on pin 5

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

void setup() {

Serial.begin(9600); 

//Slowing down speed to use long cable
Wire.begin();
TWBR = 158;  
TWSR |= bit (TWPS0);

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

sensors.begin();

//Alarm

pinMode(rtcAlarmPin, INPUT_PULLUP); // Set interrupt pin
// initialize the alarms to known values, clear the alarm flags, clear the alarm interrupt flags
RTC.setAlarm(ALM1_MATCH_DATE, 5, 0, 0, 0);
RTC.setAlarm(ALM2_MATCH_DATE, 5, 0, 0, 0);
RTC.alarm(ALARM_1);
RTC.alarm(ALARM_2);
RTC.alarmInterrupt(ALARM_1, false);
RTC.alarmInterrupt(ALARM_2, false);
RTC.squareWave(SQWAVE_NONE);

// set the alarm
DateTime now = rtc.now();

RTC.setAlarm(ALM1_MATCH_MINUTES , 5, 0, 0, 0); //Logger will wake up next hour at 0 minutes and 0 seconds

// clear the alarm flag
RTC.alarm(ALARM_1);
RTC.alarmInterrupt(ALARM_1, true); // Enable alarm 1 interrupt A1IE

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
    
    //LOG DATA, SET ALARM AND GO TO SLEEP

    logData();
    sleepNow(); 
    setRTCagain();
  }

  //If battery is low print LOW BATTERY and sleep

  else

  {

 #ifdef ECHO_TO_SERIAL
 Serial.print(F(" Low_battery: "));
 Serial.print(averageVolt);
 Serial.println(F(" volts"));
 Serial.print(F("Going to sleep to protect battery and sd card!!"));
 #endif
 
 //TURN OFF TRANSISTORS (turn of current)
    
 digitalWrite(power_GRD_NPN, LOW); //NPN transistors are off when LOW
 digitalWrite(power_VCC_PNP, HIGH); //PNP transistors are off when HIGH

 LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
 
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
  mySensorData = SD.open(loggerID,  O_CREAT | O_APPEND | O_WRITE);
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

    //GIVE TIME TO SAVE DATA
    delay(250); 

    //TURN OFF TRANSISTORS

    digitalWrite(power_GRD_NPN, LOW); 
    digitalWrite(power_VCC_PNP, HIGH); 
    
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

  settingsCSV.println("Log interval(seconds)= 5");
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


 //INTERRUPT SERVICE ROUTINE
 
void wake ()
{
  detachPinChangeInterrupt (21); // Disable interrupts on pin 5 (see https://github.com/NicoHood/PinChangeInterrupt)
}


//SLEEP LOGGER

void sleepNow ()
{
  noInterrupts (); // Disable interrupts before entering sleep
  attachPinChangeInterrupt (21, wake, FALLING);  // Wake on falling edge of D3
  interrupts (); // Enable interrupts to ensure next instruction is executed

  //TURNING OFF TRANSISTORS
  
  digitalWrite(power_GRD_NPN, LOW); 
  digitalWrite(power_VCC_PNP, HIGH); 

  //GOING TO SLEEP
  
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
}

//SET ALARM AFTER WAKE UP

void setRTCagain() {
  
  #ifdef ECHO_TO_SERIAL
  Serial.println(F("Awake..."));
  #endif
 
  if ( RTC.alarm(ALARM_1) )
  {
    
    // set the alarm
    DateTime now = rtc.now();    
    DateTime future (now + TimeSpan(0, 0, 0, LogInterval*2)); //Se multiplica por dos, si no, queda el interavlo en la mitad
    
    #ifdef ECHO_TO_SERIAL
    Serial.println(F("Alarm is set to: "));
    #endif
    
    printDateTime(future);
    RTC.setAlarm(ALM1_MATCH_DATE, future.second(), future.minute(), future.hour(), future.day());
    
    //PULL DOWN SCL AND SDA RESISTORS TO SAVE POWER
    
    Wire.end();
  
  }
}

void printDateTime(DateTime t)
{
   #ifdef ECHO_TO_SERIAL
  Serial.print((t.day() < 10) ? "0" : ""); Serial.print(t.day(), DEC); Serial.print('/');
  Serial.print((t.month() < 10) ? "0" : ""); Serial.print(t.month(), DEC); Serial.print('/');
  Serial.print(t.year(), DEC); Serial.print(' ');
  Serial.print((t.hour() < 10) ? "0" : ""); Serial.print(t.hour(), DEC); Serial.print(':');
  Serial.print((t.minute() < 10) ? "0" : ""); Serial.print(t.minute(), DEC); Serial.print(':');
  Serial.print((t.second() < 10) ? "0" : ""); Serial.println(t.second(), DEC);
  Serial.flush();
   #endif
}
