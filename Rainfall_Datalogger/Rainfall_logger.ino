/*
RAINFALL DATALOGGER

Version: 24/05/2022
Power consumption: Sleeping 21.28 microamperes and saving data 105 mah (during 200 miliseconds)
A 3000 mah 18650 battery should last more than a year in a very rainy environment like Valdivia!!
http://oregonembedded.com/batterycalc.htm

HARDWARE:
a-Arduino Pro Mini 3.3v
b-Micro sd card and reader
c-DS3231 Clock
d-Printed board circuit Yanacona v1.3 (Blue one)
e-NPN transistor (2n222a)
f-PNP transistor (2n3906)
g-Three 100 ohms resistors (one for green Led)
i-Green led 
j-Capacitor 330 uf
m- Misol tipping bucket
n-Resistencia 10k

Data is saved in CSV format without headers (logger run outs of SRAM with more than 20 string columns)

-Column 1: Date
-Column 2: Time
-Columns 3: Rainfall
-Column 4: Temp DS3231 Clock
-Column 5: Battery Voltage

#Every time the bucket is emptied the logger saves data. 
The logger Yanacona 1.3 was adapated. The tipping bucket power line goes with a female cable to Arduino programming vcc. 
The ground goes to a common ground and the data goes to three pin borne soldered on pins 6,7 and 8 (we use number 6).

#Hardware power saving strategies

-Remove led resistance and voltage regulator of Arduino Pro Mini 3.3v
-Remove led resistance and RP1 resistance package of DS3231 Clock. 

-Green blinks first time if all is ready to save data and blinks when data is recorded

*/

//LIBRARIES

#include <Wire.h> //Clock
#include <SPI.h> //SD card
#include "SdFat.h"
#include <stdint.h>
#include "EEPROM.h"
#include <DS3232RTC.h>       
#include <LowPower.h>
#include "RTClib.h"
#include "PinChangeInterrupt.h"
#include <avr/sleep.h>
#include <avr/power.h>


//SERIAL

//define ECHO_TO_SERIAL //Uncomment for testing and comment for field deployment

//RAINFALL

const byte wakePin = 6; //Button to perform interrupt

//CSV FILE NAME STORED IN EEPROM

char loggerID[30];  

//SETTING LOGGING INTERVAL AND VARIABLES TO ADJUST TIME

long LogInterval;   
int Year;
int Month;
int Day;
int Hour;
int Minute;
int Second;     

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

void setup() {

Serial.begin(9600); 

//LED
 pinMode(ledPin, OUTPUT);

//LOW POWER
pinMode(power_GRD_NPN, OUTPUT); //Controlan los transistores
pinMode(power_VCC_PNP, OUTPUT);

//SD CARD
pinMode(CS, OUTPUT);
pinMode(MOSIPin, OUTPUT);
pinMode(MISOPin, INPUT);
pinMode(SCKPin, OUTPUT);

//BUCKET
pinMode(wakePin, INPUT_PULLUP);   

Initialize_SDcard();
GetSettings();

}

void loop()

{

 //Awake with bucket, log data and sleep
 
  logData();

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

  sleepNow();

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
    
  digitalWrite(power_GRD_NPN, HIGH); //NPN transistor are on when HIGH
  digitalWrite(power_VCC_PNP, LOW); //PNP transistor are on when LOW
  delay(50);

 
  //Clock
  rtc.begin();
  DateTime now = rtc.now(); //Set the clock first running the RTC 3231 example. Then run this code
  Serial.print(now.day()+Day, DEC);
  //SD CARD
 
  SD.begin(CS);
  delay(50);
  mySensorData = SD.open(loggerID, FILE_WRITE);
  while (mySensorData)
  {

    //Date and time can be adjusted from settings.txt file located in the sd card
    
    mySensorData.print(now.year()+Year, DEC);
    mySensorData.print('/');
    mySensorData.print(now.month()+Month, DEC);
    mySensorData.print('/');
    mySensorData.print(now.day()+Day, DEC);
    mySensorData.print(',');
    mySensorData.print(now.hour()+Hour, DEC);
    mySensorData.print(':');
    mySensorData.print(now.minute()+Minute, DEC);
    mySensorData.print(':');
    mySensorData.print(now.second()+Second, DEC);
    mySensorData.print(",");
    
    //Saving rain
    mySensorData.print(0.3537); //https://www.robotics.org.za/WH-SP-RG
    mySensorData.print(",");
        
    //Clock temperature
    mySensorData.print(rtc.getTemperature());
    mySensorData.print(",");
     
     //Battery Voltage
    mySensorData.println(averageVolt);
  
    mySensorData.close();

     #ifdef ECHO_TO_SERIAL

  
    Serial.print(now.hour()+Hour, DEC);
    Serial.print(':');
    Serial.println(now.minute()+Minute, DEC);
  
    #endif

    //PULL DOWN SPI PINS TO SAVE POWER

    digitalWrite(SCKPin, LOW);
    digitalWrite(CS, LOW);
    digitalWrite(MOSIPin, LOW);
    digitalWrite(MISOPin, LOW);
    SPI.end();
    
   //BUCKET
   
   digitalWrite(wakePin, LOW);  //
   digitalWrite(ledPin, LOW);//
    
   Wire.end(); //No cambia nada
    
    //GO TO SLEEP

//TURN OFF TRANSISTORS (turn off current)
    
 digitalWrite(power_GRD_NPN, LOW); //NPN transistors are off when LOW
 digitalWrite(power_VCC_PNP, HIGH); //PNP transistors are off when HIGH
   
 //Give time to save data
 delay(200);

  }


  }

  //If battery is low print LOW BATTERY and do nothing

  else

  {

 #ifdef ECHO_TO_SERIAL
 Serial.print(F(" Low_battery: "));
 Serial.print(averageVolt);
 Serial.println(F(" volts"));
 Serial.print(F("Going to sleep to protect battery and sd card!!"));
 #endif
 
 //TURN OFF TRANSISTORS (turn off current)
    
 digitalWrite(power_GRD_NPN, LOW); //NPN transistors are off when LOW
 digitalWrite(power_VCC_PNP, HIGH); //PNP transistors are off when HIGH

 Wire.end();

  }

}

//FUNCTION TO CHECK SD CARD AND WRITE COLUMN HEADERS

void Initialize_SDcard()
{

 //TURN ON TRANSISTORS

  digitalWrite(power_GRD_NPN, HIGH); 
  digitalWrite(power_VCC_PNP, LOW); 
  delay(250); //Give time for current to flow properly

//CHECKING BATTERY VOLTAGE

  float sum = 0;

  for (int i = 0; i <= 20; i++)

  {
    
    battVolts = getBandgap();
    sum = sum + battVolts;
  }

   averageVolt = ((sum/20)/100)+batteryOffset;

   Serial.print(averageVolt);
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
    
     //TURNING OFF TRANSISTORS
  
  digitalWrite(power_GRD_NPN, LOW); 
  digitalWrite(power_VCC_PNP, HIGH); 

   //GOING TO SLEEP
  
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
   
    }
  
  else 
  
  {

     #ifdef ECHO_TO_SERIAL
    Serial.println(F("Sd Card Ready and Battery Ok!!"));
     #endif
     
    blinkyOkBatterySD();
    writeSettings();
    LoggerId();

  //ADDING HEADERS TO CSV FILE. UNCOMMENT ONLY IF THERE ARE LESS THAN 20 COLUMNS WITH SHORT NAMES


     File dataFile = SD.open(loggerID, FILE_WRITE);
      
  // if file is available, write headers
  
     if (dataFile) {
    dataFile.println("Date,Time,Rain,TempBoxClock,voltage");
    dataFile.close();
    }

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
           
       for (byte i=0; i<=6; i++)  {  
              
         char line[30];
         SettingsFile.fgets(line,30);// read one line
         char* p_pos = strchr(line, '=');// find the '=' position
         
         if(i==0)  LogInterval = atoi(p_pos+1);
         if(i==1)  Year = atoi(p_pos+1);
         if(i==2)  Month = atoi(p_pos+1);
         if(i==3)  Day = atoi(p_pos+1);
         if(i==4)  Hour = atoi(p_pos+1);
         if(i==5)  Minute = atoi(p_pos+1);
         if(i==6)  Second = atoi(p_pos+1); 
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

  settingsCSV = SD.open("settings.txt", FILE_WRITE);
  
  // Write log interval

  settingsCSV.println("Interval in minutes= 30");
  settingsCSV.println("Years= 0");
  settingsCSV.println("Months= 0");
  settingsCSV.println("Days= 0");
  settingsCSV.println("Hours= 0");
  settingsCSV.println("Minutes=0");
  settingsCSV.println("Seconds= 0");

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

void sleepNow() {  

    ADCSRA = 0;
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here  
         
    sleep_enable();          // enables the sleep bit in the mcucr register  
    attachPinChangeInterrupt(22,wakeUpNow, CHANGE); // use interrupt 0 (pin 2) and run function  
    sleep_mode();            // here the device is actually put to sleep!!  
    // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP  
    sleep_disable();         // first thing after waking from sleep: disable sleep...  
    detachPinChangeInterrupt(22);      // disables interrupt 0 on pin 2 so the wakeUpNow code will not be executed during normal running time.  
}  


void wakeUpNow() {  
  // execute code here after wake-up before returning to the loop() function  
  // timers and code using timers (serial.print and more...) will not work here.  
  // we don't really need to execute any special functions here, since we  
  // just want the thing to wake up  
}  
