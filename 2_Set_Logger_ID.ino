//Sketch to set Data Logger ID. 
//ID is saved in Arduino's Promini EEPROM memory 
//so this code si run just once

#include "EEPROM.h"

////////////////////////////////////////
char loggerId[] = "Y0023.csv";
/////////////////////////////////////////


int startingAddress = 30;
void setup()
{
  Serial.begin(9600);
  
  for(int n = 0; n < sizeof(loggerId); n++) // automatically adjust for number of digits
  {
     EEPROM.write(n + startingAddress, loggerId[n]);  
    
  }
  
  char readNumber[30];  // make sure there is enough room for all digits
  
  for(int n = 0; n < 30; n++)
  {
     readNumber[n] = EEPROM.read(n+startingAddress);     
  }

  Serial.println(readNumber);  
}

void loop()
{
  
  
}
