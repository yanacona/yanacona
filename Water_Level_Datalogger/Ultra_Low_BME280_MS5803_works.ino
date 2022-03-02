/*
 TEMPERATURE AND PRESSURE DATALOGGER 
  Version2: 30/04/2021
  Power consumption: Stanby 0.10 mah, measuring 47 mah (con tarjeta de 128MB)
  
  HARDWARE:
 a-Arduino Pro Mini
 c-BME280 sensor
 d-Micro sd card reader
 e-DS3231 Clock
 f-Printed board circuit Yanacona v.1
 g-NPN transistor (2n222a)
 e-PNP transistor (2n3906)
 f-Three 100 ohms resistors (one for green Led)
 g-One 4700 k resistor
 h-Green led 
 I-Condensador 330 uf
 J-MS5803 sensor
 K-Cable 20 awg de 4 pines Cordón Svt 4x 0.75mm (4x 20awg)
 M-IP 65 box 100*100*55
 
 Data is saved in CSV format as following:
 
 -Column 1: Date
 -Column 2: Time
 -Column 3: Air temperature BME
 -Column 4: Air pressure BME
 -Column 5: Air humidity BME
 -Column 6: Water temperature MS5803
 -Column 7: Water pressure MS5803
 -Column 8: water_level(cm)
 -Column 9: Box temperature DS3231
 -Column 10: Voltage

 #The log interval in seconds is read from first line of txt file named settings.txt located in sd card

 LogInterval= 10

#Hardware power saving strategies

-Remove LED and voltage regulator of Arduino Pro Mini
-Remove Led and resistance of DS3231 Clock

-Green blinks first time if all is ready to save data 

/BMP280 sensor is modified to address x77 (cut trace and solder pad)https://forum.arduino.cc/index.php?topic=404946.0

*/

//LIBRARIES
#include <JeeLib.h> //Low power consumption
#include <OneWire.h> //Temperature
#include <DallasTemperature.h> //Temperature
#include "RTClib.h"
#include <Wire.h> //Clock
#include <SPI.h> //SD card
#include "SdFat.h"
#include <stdint.h>
#include "SparkFunBME280.h"
#include <SparkFun_MS5803_I2C.h>


//BME 280 PRESSURE SENSOR (air)
BME280 seBME280; //Sensor that goes to the water

float BME280_temp;
float BME280_pressure;

//MS5803 PRESSURE SENSOR (water)

MS5803 sensor(ADDRESS_HIGH);

float temp_MS5803;
double pressure_MS5803;

float offset= 0; //Offset between BME280 and MS5803 sensor

//CSV FILE NAME
String FileName = "Y0013.csv";

//PARAMETERS

unsigned long LogInterval;   

//POWER

const int power_GRD_NPN = 3; //NPN-from right to left (looking to flat face of transistor 2n2222a) GRD sensor, resistance 100 ohms to pin 3, GRD battery
const int power_VCC_PNP = 2; //PNP from right to left (looking to flat face of transistor 2n3906)VCC battery,resistance 100 ohms to pin 2, VCC sensor

//BATTERY
int battVolts;
float averageVolt;
float minBattery = 3.3; //Treshold of minimum battery to stop recording data
const float batteryOffset = 0; //Offset between multimeter and arduino measured battery

//CLOCK (DS3231)
// SDA A4 , SCL A5
RTC_DS3231 rtc;

//SD CARD
//MOSI = 11; MISO=12;CLK=13;CS=5 (only cs can be changed)

SdFat SD;
const int CS = 10;
const int MOSIPin = 11;
const int MISOPin = 12;
const int SCKPin = 13;
File mySensorData;

//GEEN LED SD OK

int ledPin = A1;

//LOW POWER
ISR(WDT_vect) {
  Sleepy::watchdogEvent();  // Setup the watchdog
}

void setup() {
  Serial.begin(9600); // Starts the serial communication

//Slowing down speed to use long cable
  Wire.begin();
  TWBR = 158;  
  TWSR |= bit (TWPS0);

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

   //BMP280

  //***Set up sensor 'A'******************************//
  //commInterface can be I2C_MODE
  seBME280.settings.commInterface = I2C_MODE;
  seBME280.settings.I2CAddress = 0x77;
  seBME280.settings.runMode = 3; //  3, Normal mode
  seBME280.settings.tStandby = 0; //  0, 0.5ms
  seBME280.settings.filter = 0; //  0, filter off
  seBME280.settings.tempOverSample = 1;
  seBME280.settings.pressOverSample = 1;
  seBME280.settings.humidOverSample = 1;

  //***Initialize BMP sensor**************************//

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

  
  if (averageVolt >= minBattery)//If battery is higher than predefined voltage (3.6)

  {
    //TURN OFF TRANSISTORS

    digitalWrite(power_GRD_NPN, LOW); //NPN transistors are off when LOW
    digitalWrite(power_VCC_PNP, HIGH); //PNP transistors are off when HIGH

    //LOG DATA AND PRINT SERIAL

    logData();

  }

  //If battery is low print LOW BATTERY and do nothing

  else

  {

 Serial.print(" Low_battery: ");
 Serial.print(averageVolt);
 Serial.println(" volts");
 Serial.print("Going to sleep to protect battery!!");
 
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
  delay(50);

  //Clock
  rtc.begin();
  DateTime now = rtc.now(); //Set the clock first running the RTC 3231 example. Then run this code

  //SD CARD
 
  SD.begin(CS);
  mySensorData = SD.open(FileName, FILE_WRITE);
  while (mySensorData)
  {

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
    mySensorData.print(",");

    //Iniciando sensor BME280

    (seBME280.begin(), HEX);
    //Sin este delay nose guarda el dato correcto en la tarjeta SD aunque muestra el dato correcto en Serial
    delay(10);

   //Air temperature
    mySensorData.print(seBME280.readTempC(), 2);
    mySensorData.print(", ");

    //Air pressure
    mySensorData.print((seBME280.readFloatPressure() / 100));
    mySensorData.print(", ");

    
    //Humidity
    mySensorData.print(seBME280.readFloatHumidity());
    mySensorData.print(", ");
 
      //Retrieve calibration constants for conversion math.

    sensor.reset();
    sensor.begin();

    temp_MS5803 = sensor.getTemperature(CELSIUS, ADC_512);
    pressure_MS5803 = sensor.getPressure(ADC_4096);

    //Water temperature
    mySensorData.print(temp_MS5803);
    mySensorData.print(", ");

    //Water pressure
      mySensorData.print(pressure_MS5803);
      mySensorData.print(", ");

    //Water level
    
     mySensorData.print(((sensor.getPressure(ADC_4096)-(seBME280.readFloatPressure() / 100)) + offset)*1.0197);
     mySensorData.print(", ");

    //Clock temperature
    mySensorData.print(rtc.getTemperature());
    mySensorData.print(",");

    //Battery Voltage
    mySensorData.println(averageVolt);

    //PRINT DATA IN SERIAL

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

    //Air temperature
    Serial.print(seBME280.readTempC(), 2);
    Serial.print(", ");

    //Air pressure
    Serial.print((seBME280.readFloatPressure() / 100));
    Serial.print(", ");

    //Humidity
    Serial.print(seBME280.readFloatHumidity());
    Serial.print(", ");

        //Water temperature
    Serial.print(temp_MS5803);
    Serial.print(", ");

    //Water pressure
     Serial.print(pressure_MS5803);
     Serial.print(", ");

    //Water level
     
     Serial.print(((sensor.getPressure(ADC_4096)-(seBME280.readFloatPressure() / 100)) + offset)*1.0197);
     Serial.print(", ");

    //Temp Clock
    Serial.print(rtc.getTemperature());
    Serial.print(",");

    //Battery Voltage
    Serial.println(averageVolt);

    mySensorData.close();

    //SD CARD

    digitalWrite(SCKPin, LOW);
    digitalWrite(CS, LOW);
    digitalWrite(MOSIPin, LOW);
    digitalWrite(MISOPin, LOW);
    SPI.end();
    

    // Pull down SCL resistor
    digitalWrite(A5, LOW);

        //GIVE TIME TO SAVE DATA
    delay(1000); 

    //TURN OFF TRANSISTORS (turn of current)
    
    digitalWrite(power_GRD_NPN, LOW); //NPN transistors are off when LOW
    digitalWrite(power_VCC_PNP, HIGH); //PNP transistors are off when HIGH

    //SETTING TIMING TO LOG DATA
    for (byte i = 0; i < 10; ++i)
    {
   // Change number after < to indicate the interval in minutes

      Sleepy::loseSomeTime(60000);// Change the number in parenthesis to 60000 to measure minutes
    }
  }
}

//FUNCTION TO CHECK SD CARD AND WRITE COLUMN HEADERS

void Initialize_SDcard()
{

  digitalWrite(power_GRD_NPN, HIGH); //NPN transistor are on when HIGH
  digitalWrite(power_VCC_PNP, LOW); //PNP transistor are on when LOW
  delay(100); //Give time for current to flow properly

//Checking battery

  float sum = 0;

  for (int i = 0; i <= 20; i++)

  {
    
    battVolts = getBandgap();
    sum = sum + battVolts;
  }

   averageVolt = ((sum/20)/100)+batteryOffset;

    // see if the card is present and can be initialized:
    
  if (!SD.begin(CS)) 
  {
    Serial.println("Card failed!!");
  }

  else if (averageVolt<minBattery)
  
  {
    
    Serial.println("Low Battery!!= ");
    Serial.print(averageVolt);
    Serial.print("v");
    
    }
  
  else 
  
  {

    Serial.println("Sd Card Ready and Battery Ok!!");
    blinkyOkBatterySD();
    writeSettings();

      File dataFile = SD.open(FileName, FILE_WRITE);
  // if the file is available, write headers
     if (dataFile) {
    dataFile.println("Date,Time,BME_Temp,BME_pressure,BME_Humidity,MS5803_temp,MS5803_Pressure,water_level(cm),TempClock,voltage");
    dataFile.close();
    
     //SD CARD
    digitalWrite(SCKPin, LOW);
    digitalWrite(CS, LOW);
    digitalWrite(MOSIPin, LOW);
    digitalWrite(MISOPin, LOW);
    SPI.end();
  }
  
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

//Set log interval using settings.txt file

void GetSettings() {
      
           // open settings file
           SdFile SettingsFile ("settings.txt", O_READ);
               
           for (byte i=0; i<=5; i++)  {        
                   char line[24];
                   SettingsFile.fgets(line, 24);     // read one line
                   char* p_pos = strchr(line, '=');  // find the '=' position
                   if(i==0)  LogInterval = atoi(p_pos+1);
           }

           Serial.print(LogInterval);
           SettingsFile.close();
                 
    }


 void writeSettings()

 {

  File settingsCSV;

  settingsCSV = SD.open("settings.txt");

  if (settingsCSV) 
  
  {

 Serial.print("Setting file already exists");
 settingsCSV.close();

  } 

else {

  settingsCSV = SD.open("settings.txt", FILE_WRITE);
  // Write log interval
  
  settingsCSV.println("logInterval= 10");
  settingsCSV.println("//Setting log time interval in seconds//");
  settingsCSV.close();
  }
  } 


  
