#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "Alarms.h"
#include "I2C_ServoDriver_Arduino.h"
#include "WireMod.h"
#include "i2c.h"

volatile static uint8_t atomic_update =0;

#define REQUEST_ID 0xFFFF
static int i = 0;

static void requestEvent (void)
{
  Serial.print("requestEvent CALLED \r\n");
}

static void receiveEvent(int bytes) {
  
  // Serial.print("EVENT CALLED ");
  //Serial.print(bytes, DEC);
  //Serial.print(" bytes \r\n");
  char c;
  
  int cmd = Wire.readFast(); //i2c_read(1); //
  int param1, param2;
  
  i2c_slave_received = 1;
  if (atomic_update ==1) return ;
  atomic_update = 1;

  switch (cmd)
  {
    case REQUEST_ID:
      // asking for ID
      Serial.print("ID\n");

    break;
    
    case MW_I2C_RA_ALL_OUTPUT_SAME:
        Serial.print("MW_I2C_RA_ALL_OUTPUT_SAME\r\n");
        param1 = Wire.readFast();
        param2 = Wire.readFast();
        i=MW_I2C_CH0;
        while (i<SERVO_CHANNELS_MAX)
        {
          Servo_Buffer[i] =  param1 +  param2 * 0xFF;
          ++i;
        }
    break;
    
    case MW_I2C_CH0:
        i=MW_I2C_CH0;
        while (Wire.available() && (i<SERVO_CHANNELS_MAX))
        {
          param1 = Wire.readFast();
          param2 = Wire.readFast();
          // Serial.print(param1,HEX);
          // Serial.print("\r\n");
          // Serial.print(param2,HEX);
          // Serial.print("\r\n");
          Servo_Buffer[i] =  param1 +  param2 * 0xFF;
          ++i;
        }
    break;

    case MW_I2C_CH1:
        i=MW_I2C_CH1;
        while (Wire.available() && (i<SERVO_CHANNELS_MAX))
        {
          param1 = Wire.readFast();
          param2 = Wire.readFast();
          // Serial.print(param1,HEX);
          // Serial.print("\r\n");
          // Serial.print(param2,HEX);
          // Serial.print("\r\n");
          Servo_Buffer[i] =  param1 +  param2 * 0xFF;
          ++i;
        }
    break;

    case MW_I2C_CH2:
        i=MW_I2C_CH2;
        while (Wire.available() && (i<SERVO_CHANNELS_MAX))
        {
          param1 = Wire.readFast();
          param2 = Wire.readFast();
          // Serial.print(param1,HEX);
          // Serial.print("\r\n");
          // Serial.print(param2,HEX);
          // Serial.print("\r\n");
          Servo_Buffer[i] =  param1 +  param2 * 0xFF;
          ++i;
        }
    break;

    case MW_I2C_CH3:
        i=MW_I2C_CH3;
        while (Wire.available() && (i<SERVO_CHANNELS_MAX))
        {
          param1 = Wire.readFast();
          param2 = Wire.readFast();
          // Serial.print(param1,HEX);
          // Serial.print("\r\n");
          // Serial.print(param2,HEX);
          // Serial.print("\r\n");
          Servo_Buffer[i] =  param1 +  param2 * 0xFF;
          ++i;
        }
    break;

    case MW_I2C_CH4:
        i=MW_I2C_CH4;
        while (Wire.available() && (i<SERVO_CHANNELS_MAX))
        {
          param1 = Wire.readFast();
          param2 = Wire.readFast();
          // Serial.print(param1,HEX);
          // Serial.print("\r\n");
          // Serial.print(param2,HEX);
          // Serial.print("\r\n");
          Servo_Buffer[i] =  param1 +  param2 * 0xFF;
          ++i;
        }
    break;

    case MW_I2C_CH5:
        i=MW_I2C_CH5;
        while (Wire.available() && (i<SERVO_CHANNELS_MAX))
        {
          param1 = Wire.readFast();
          param2 = Wire.readFast();
          // Serial.print(param1,HEX);
          // Serial.print("\r\n");
          // Serial.print(param2,HEX);
          // Serial.print("\r\n");
          Servo_Buffer[i] =  param1 +  param2 * 0xFF;
          ++i;
        }
    break;

    case MW_I2C_CH6:
        i=MW_I2C_CH6;
        while (Wire.available() && (i<SERVO_CHANNELS_MAX))
        {
          param1 = Wire.readFast();
          param2 = Wire.readFast();
          // Serial.print(param1,HEX);
          // Serial.print("\r\n");
          // Serial.print(param2,HEX);
          // Serial.print("\r\n");
          Servo_Buffer[i] =  param1 +  param2 * 0xFF;
          ++i;
        }
    break;

    case MW_I2C_CH7:
        i=MW_I2C_CH7;
        while (Wire.available() && (i<SERVO_CHANNELS_MAX))
        {
          param1 = Wire.readFast();
          param2 = Wire.readFast();
          // Serial.print(param1,HEX);
          // Serial.print("\r\n");
          // Serial.print(param2,HEX);
          // Serial.print("\r\n");
          Servo_Buffer[i] =  param1 +  param2 * 0xFF;
          ++i;
        }
    break;

    case MW_I2C_CH8:
        i=MW_I2C_CH8;
        while (Wire.available() && (i<SERVO_CHANNELS_MAX))
        {
          param1 = Wire.readFast();
          param2 = Wire.readFast();
          // Serial.print(param1,HEX);
          // Serial.print("\r\n");
          // Serial.print(param2,HEX);
          // Serial.print("\r\n");
          Servo_Buffer[i] =  param1 +  param2 * 0xFF;
          ++i;
        }
    break;

    case MW_I2C_CH9:
        i=MW_I2C_CH9;
        while (Wire.available() && (i<SERVO_CHANNELS_MAX))
        {
          param1 = Wire.readFast();
          param2 = Wire.readFast();
          // Serial.print(param1,HEX);
          // Serial.print("\r\n");
          // Serial.print(param2,HEX);
          // Serial.print("\r\n");
          Servo_Buffer[i] =  param1 +  param2 * 0xFF;
          ++i;
        }
    break;

    case MW_I2C_CH10:
        i=MW_I2C_CH10;
        while (Wire.available() && (i<SERVO_CHANNELS_MAX))
        {
          param1 = Wire.readFast();
          param2 = Wire.readFast();
          // Serial.print(param1,HEX);
          // Serial.print("\r\n");
          // Serial.print(param2,HEX);
          // Serial.print("\r\n");
          Servo_Buffer[i] =  param1 +  param2 * 0xFF;
          ++i;
        }
    break;

    case MW_I2C_CH11:
        i=MW_I2C_CH11;
        while (Wire.available() && (i<SERVO_CHANNELS_MAX))
        {
          param1 = Wire.readFast();
          param2 = Wire.readFast();
          // Serial.print(param1,HEX);
          // Serial.print("\r\n");
          // Serial.print(param2,HEX);
          // Serial.print("\r\n");
          Servo_Buffer[i] =  param1 +  param2 * 0xFF;
          ++i;
        }
    break;

    case MW_I2C_CH12:
        i=MW_I2C_CH12;
        while (Wire.available() && (i<SERVO_CHANNELS_MAX))
        {
          param1 = Wire.readFast();
          param2 = Wire.readFast();
          // Serial.print(param1,HEX);
          // Serial.print("\r\n");
          // Serial.print(param2,HEX);
          // Serial.print("\r\n");
          Servo_Buffer[i] =  param1 +  param2 * 0xFF;
          ++i;
        }
    break;

    case MW_I2C_CH13:
        i=MW_I2C_CH13;
        while (Wire.available() && (i<SERVO_CHANNELS_MAX))
        {
          param1 = Wire.readFast();
          param2 = Wire.readFast();
          // Serial.print(param1,HEX);
          // Serial.print("\r\n");
          // Serial.print(param2,HEX);
          // Serial.print("\r\n");
          Servo_Buffer[i] =  param1 +  param2 * 0xFF;
          ++i;
        }
    break;

    case MW_I2C_CH14:
        i=MW_I2C_CH14;
        while (Wire.available() && (i<SERVO_CHANNELS_MAX))
        {
          param1 = Wire.readFast();
          param2 = Wire.readFast();
          // Serial.print(param1,HEX);
          // Serial.print("\r\n");
          // Serial.print(param2,HEX);
          // Serial.print("\r\n");
          Servo_Buffer[i] =  param1 +  param2 * 0xFF;
          ++i;
        }
    break;

    case MW_I2C_CH15:
        i=MW_I2C_CH15;
        while (Wire.available() && (i<SERVO_CHANNELS_MAX))
        {
          param1 = Wire.readFast();
          param2 = Wire.readFast();
          // Serial.print(param1,HEX);
          // Serial.print("\r\n");
          // Serial.print(param2,HEX);
          // Serial.print("\r\n");
          Servo_Buffer[i] =  param1 +  param2 * 0xFF;
          ++i;
        }
    break;

    case MW_I2C_CH16:
        i=MW_I2C_CH16;
        while (Wire.available() && (i<SERVO_CHANNELS_MAX))
        {
          param1 = Wire.readFast();
          param2 = Wire.readFast();
          // Serial.print(param1,HEX);
          // Serial.print("\r\n");
          // Serial.print(param2,HEX);
          // Serial.print("\r\n");
          Servo_Buffer[i] =  param1 +  param2 * 0xFF;
          ++i;
        }
    break;

    case MW_I2C_CH17:
        i=MW_I2C_CH17;
        while (Wire.available() && (i<SERVO_CHANNELS_MAX))
        {
          param1 = Wire.readFast();
          param2 = Wire.readFast();
          // Serial.print(param1,HEX);
          // Serial.print("\r\n");
          // Serial.print(param2,HEX);
          // Serial.print("\r\n");
          Servo_Buffer[i] =  param1 +  param2 * 0xFF;
          ++i;
        }
    break;

    default:
      Serial.print("UNKNOWN\r\n");
      Serial.print(cmd,HEX);
      Serial.print(" cmd \r\n");
      while (bytes>1) {
        Serial.print(Wire.read(),HEX);
        Serial.print("\r\n");
        --bytes;
      }
      break;
  }
  // SHOULD NOT BE AVAILE
  while (Wire.available()>0) {
        Serial.print("EXTRA \r\n");
        Serial.print(Wire.readFast(),HEX);
        Serial.print("\r\n");
      }

  //Serial.print("Servo_Buffer:\r\n");
  // for (i=0;i<8;++i)
  // {
  //   Serial.print(Servo_Buffer[i]);
  //   Serial.print("  ");
  // }
  //Serial.print("\r\n");
      
  atomic_update =0;
}

void zeroI2C ()
{
  for (int i=0;i<8;++i)
  {
    if (atomic_update ==1) return;
    Servo_Buffer[i] = 0;
  }
}

void configureI2CSlave()
{
    Serial.print("configureI2CSlave initialized at ");

    // Start the I2C Bus as Slave on address 9
    Wire.setClock(400000L) ;
    Wire.begin(MW_I2C__PRIMARY_ADDRESS); 
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);

}


