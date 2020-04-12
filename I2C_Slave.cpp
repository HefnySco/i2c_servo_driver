#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "Alarms.h"
#include "I2C_ServoDriver_Arduino.h"
#include <Wire.h>
//#include "Serial.h"
#define CH1 
static uint16_t Servo_Buffer[10] = {3000,3000,3000,3000,3000,3000,3000,3000};

void receiveEvent(int bytes) {
  
  blinkLED(2,120,4); //global_conf.currentSet+1);
  Serial.print("EVENT CALLED ");
  Serial.print(bytes, DEC);
  Serial.print(" bytes \r\n");
  char c;
  int cmd = Wire.read();
  int param1, param2;
  int i = 0;

  switch (cmd)
  {
    case PCA9685_RA_ALL_LED_ON_L:
        Serial.print("PCA9685_RA_ALL_LED_ON_L\r\n");
        while (Wire.available() && (i<10))
        {
          Servo_Buffer[i] =  0xFF & Wire.read();
          ++i;
        }
      break;
    case PCA9685_RA_MODE1:
        Serial.print("PCA9685_RA_MODE1\r\n");
        Wire.read(); 
        //PCA9685_ALL_LED_OFF_H_SHUT
      break;
    case PCA9685_RA_ALL_LED_OFF_H:
          Serial.print("PCA9685_RA_ALL_LED_OFF_H\r\n");
          // Restart the device to apply new settings and enable auto-incremented write PCA9685_MODE1_RESTART_BIT | PCA9685_MODE1_AI_BIT
          Wire.read(); 
      break;
    case PCA9685_RA_PRE_SCALE:
          Serial.print("PCA9685_RA_PRE_SCALE\r\n");
          Wire.read(); 
      break;
    case PCA9685_RA_LED0_ON_L:
        Serial.print("PCA9685_RA_LED0_ON_L\r\n");
        while (Wire.available() && (i<10))
        {
          param1 = Wire.read();
          param2 = Wire.read();
          Serial.print(param1,HEX);
          Serial.print("\r\n");
          Serial.print(param2,HEX);
          Serial.print("\r\n");
          Servo_Buffer[i] =  param1 +  param2 * 0xFF;
          ++i;
        }
      break;
    default:
      Serial.print("UNKNOWN\r\n");
      Serial.print(cmd,HEX);
      Serial.print(" cmd \r\n");
      while (Wire.available()>0) {
        Serial.print(Wire.read(),HEX);
        Serial.print("\r\n");
      }
      break;
  }
  // SHOULD NOT BE AVAILE
  while (Wire.available()>0) {
        Serial.print("EXTRA \r\n");
        Serial.print(Wire.read(),HEX);
        Serial.print("\r\n");
      }

  Serial.print("Servo_Buffer:\r\n");
  for (i=0;i<8;++i)
  {
    Serial.print(Servo_Buffer[i]);
    Serial.print("\r\n");
  }
  
      rcCommand[ROLL] = Servo_Buffer[0];
      rcCommand[PITCH] = Servo_Buffer[1];
      rcCommand[THROTTLE] = Servo_Buffer[2];
      rcCommand[YAW] = Servo_Buffer[3]; 
      rcCommand[AUX1] = Servo_Buffer[4]; 
      rcCommand[AUX2] = Servo_Buffer[5]; 
      rcCommand[AUX3] = Servo_Buffer[6]; 
      rcCommand[AUX4] = Servo_Buffer[7];
}


void configureI2CSlave()
{
    //SerialWrite(1,0x64);
    blinkLED(2,120,4); //global_conf.currentSet+1);
    //SerialWrite (0,'C');
    //SerialEnd(0);
    Serial.print("configureI2CSlave initialized at ");

    // Start the I2C Bus as Slave on address 9
    Wire.begin(PCA9685_PRIMARY_ADDRESS); 

    Serial.print(PCA9685_PRIMARY_ADDRESS, HEX);
    Serial.print(" \r\n");
    // Attach a function to trigger when something is received.
    Wire.onReceive(receiveEvent);

}


