#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "Alarms.h"
#include "I2C_ServoDriver_Arduino.h"
#include <Wire.h>
//#include "Serial.h"



void receiveEvent(int bytes) {
  
  blinkLED(2,120,4); //global_conf.currentSet+1);
  Serial.print("EVENT CALLED\r\n");
  //SerialEnd(0);
  //SerialEnd(0);
  
  while (1 < Wire.available()) { // loop through all but the last
    char c = Wire.read(); // receive byte as a character
    
  }
  
  int x = Wire.read();    // receive byte as an integer

}


void configureI2CSlave()
{
    //SerialWrite(1,0x64);
    blinkLED(2,120,4); //global_conf.currentSet+1);
    //SerialWrite (0,'C');
    //SerialEnd(0);
    Serial.print("configureI2CSlave\r\n");

    // Start the I2C Bus as Slave on address 9
    Wire.begin(9); 
    // Attach a function to trigger when something is received.
    Wire.onReceive(receiveEvent);

}


