//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//SCRIPT TO CHECK SD AND CLOCK CONECTIONS THROUGH TRANSISTORS CONECTED IN PIN 3 AND 2 AND CS TO PIN 4
//Run this sketch to check if DS3231 RTC and sd card reader are working and to set RTC date and time
//14/06/2021
//Pablo Iribarren
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//LIBRARIES
#include <JeeLib.h> //Low power consumption
#include "RTClib.h"
#include <Wire.h> //Clock
#include <SPI.h> //SD card
#include "SdFat.h"

//BATTERY//

int battVolts;
float averageVolt;

//POWER

const int power_GRD_NPN= 3; //NPN-from right to left (looking to flat face of transistor 2n2222a) GRD sensor, resistance 100 ohms to pin 3, GRD battery
const int power_VCC_PNP = 2; //PNP from right to left (looking to flat face of transistor 2n3906)VCC battery,resistance 100 ohms to pin 2, VCC sensor 

//CLOCK (DS3231)
//Pins
// SDA A4 , SCL A5
RTC_DS3231 rtc;

//SD CARD
//Pins
//MOSI = 11; MISO=12;CLK=13;CS=10
SdFat SD;
const int CS = 10;
const int MOSIPin = 11;
const int MISOPin = 12;
const int SCKPin = 13;
File mySensorData;


//LOW POWER
ISR(WDT_vect) { Sleepy::watchdogEvent(); } // Setup the watchdog

void setup() {
Serial.begin(9600); 

//LOW POWER
  pinMode(power_GRD_NPN, OUTPUT); //Transistors
  pinMode(power_VCC_PNP, OUTPUT);
  
//SD CARD
  pinMode(CS, OUTPUT);
  pinMode(MOSIPin, OUTPUT);
  pinMode(MISOPin, INPUT);
  pinMode(SCKPin, OUTPUT);

  //POWER TRANSISTORS
  digitalWrite(power_GRD_NPN, HIGH); //NPN transistor are on when HIGH
  digitalWrite(power_VCC_PNP, LOW); //PNP transistor are on when LOW
  delay(100); //Give time for current to flow properly

  //SET TIME
  rtc.begin();
  
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  }
 

  //SD CARD
  pinMode(CS, OUTPUT);

  //TURN OFF TRANSISTORS

  digitalWrite(power_GRD_NPN, LOW); //NPN transistors are off when LOW
  digitalWrite(power_VCC_PNP, HIGH); //PNP transistors are off when HIGH


}

void loop() {

//POWER TRANSISTORS
digitalWrite(power_GRD_NPN, HIGH); //NPN transistor are on when HIGH
digitalWrite(power_VCC_PNP,LOW); //PNP transistor are on when LOW
delay(100); //Give time for current to flow properly

//Log data

logData();

}

//FUNCTION TO SAVE DATA IN MICRO SD CARD AND PRINT DATA IN SERIAL

void logData()

{

  digitalWrite(power_GRD_NPN, HIGH); //NPN transistor are on when HIGH
  digitalWrite(power_VCC_PNP, LOW); //PNP transistor are on when LOW
  delay(100);

  //Clock
  DateTime now = rtc.now(); 
  
  //PRINT DATA
 
 
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
    Serial.println("RTC works!!");
    Serial.println("-------Waiting for SD card reader--------!!!");
   
    delay(1000);

  //SD CARD
  SD.begin(CS);
  mySensorData = SD.open("Testing_Logger.csv", FILE_WRITE);
  while (mySensorData)
  {

       //SAVE DATA

    mySensorData.print(now.year(), DEC);
    mySensorData.print('/');
    mySensorData.print(now.month(), DEC);
    mySensorData.print('/');
    mySensorData.print(now.day(), DEC);
    mySensorData.print(',');
    mySensorData.print(now.hour(), DEC);
    mySensorData.print(':');
    mySensorData.print(now.minute(), DEC);
    mySensorData.print(':');
    mySensorData.print(now.second(), DEC);
    mySensorData.println(",");
    mySensorData.close();

   digitalWrite(MISOPin, LOW);
   digitalWrite(SCKPin, LOW);
   digitalWrite(CS,LOW);
   digitalWrite(MOSIPin, LOW);
   SPI.end();

   //Checking battery

  float sum = 0;

  for (int i = 0; i <= 20; i++)

  {
    
    battVolts = getBandgap();
    sum = sum + battVolts;
  }

   averageVolt = ((sum/20)/100);
   averageVolt = ((sum/20)/100);
   
    //TURN OFF TRANSISTORS (turn of current)

    digitalWrite(power_GRD_NPN, LOW); //NPN transistors are off when LOW
    digitalWrite(power_VCC_PNP, HIGH); //PNP transistors are off when HIGH
 
    //SETTING TIMING TO LOG DATA
    for (byte i = 0; i < 5; ++i)// Change number after < to indicate the interval in minutes
      Sleepy::loseSomeTime(1000);// Change the number in parenthesis to 60000 to measure minutes
 
  Serial.println("---------DATA SAVED----------!!!!");
  
  Serial.println("---------BATTERY VOLTAGE----------!!!!");

  Serial.println(averageVolt);

  Serial.println("---------CHECKING CLOCK----------!!!!");
   
  }

}

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
