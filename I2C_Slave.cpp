#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "Alarms.h"
#include "I2C_ServoDriver_Arduino.h"
#include "WireMod.h"
#include "i2c.h"
//#include "Serial.h"
#define CH1 
volatile uint8_t atomic_update =0;

#define REQUEST_ID 0xFFFF

static void requestEvent (void)
{
  Serial.print("requestEvent CALLED \r\n");
}
static int i = 0;
static void receiveEvent(int bytes) {
  
  // Serial.print("EVENT CALLED ");
  Serial.print(bytes, DEC);
  Serial.print(" bytes \r\n");
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
    case MW_I2C__RA_ALL_LED_ON_L:
        Serial.print("MW_I2C__RA_ALL_LED_ON_L\r\n");
        while ((bytes>1) && (i<18))
        {
          Servo_Buffer[i] =  0xFF & Wire.read();
          ++i;
          --bytes;
        }
      break;
    // case MW_I2C__RA_MODE1:
    //     Serial.print("MW_I2C__RA_MODE1\r\n");
    //     Wire.read(); 
    //     //MW_I2C__ALL_LED_OFF_H_SHUT
    //   break;
    case MW_I2C__CH0:
        i=MW_I2C__CH0;
        while (Wire.available() && (i<18))
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

    case MW_I2C__CH1:
        i=MW_I2C__CH1;
        while (Wire.available() && (i<18))
        {
          param1 = Wire.read();
          param2 = Wire.read();
          // Serial.print(param1,HEX);
          // Serial.print("\r\n");
          // Serial.print(param2,HEX);
          // Serial.print("\r\n");
          Servo_Buffer[i] =  param1 +  param2 * 0xFF;
          ++i;
        }
    break;

    case MW_I2C__CH2:
        i=MW_I2C__CH2;
        while (Wire.available() && (i<18))
        {
          param1 = Wire.read();
          param2 = Wire.read();
          // Serial.print(param1,HEX);
          // Serial.print("\r\n");
          // Serial.print(param2,HEX);
          // Serial.print("\r\n");
          Servo_Buffer[i] =  param1 +  param2 * 0xFF;
          ++i;
        }
    break;

    case MW_I2C__CH3:
        i=MW_I2C__CH3;
        while (Wire.available() && (i<18))
        {
          param1 = Wire.read();
          param2 = Wire.read();
          // Serial.print(param1,HEX);
          // Serial.print("\r\n");
          // Serial.print(param2,HEX);
          // Serial.print("\r\n");
          Servo_Buffer[i] =  param1 +  param2 * 0xFF;
          ++i;
        }
    break;

    case MW_I2C__CH4:
        i=MW_I2C__CH4;
        while (Wire.available() && (i<18))
        {
          param1 = Wire.read();
          param2 = Wire.read();
          // Serial.print(param1,HEX);
          // Serial.print("\r\n");
          // Serial.print(param2,HEX);
          // Serial.print("\r\n");
          Servo_Buffer[i] =  param1 +  param2 * 0xFF;
          ++i;
        }
    break;

    case MW_I2C__CH5:
        i=MW_I2C__CH5;
        while (Wire.available() && (i<18))
        {
          param1 = Wire.read();
          param2 = Wire.read();
          // Serial.print(param1,HEX);
          // Serial.print("\r\n");
          // Serial.print(param2,HEX);
          // Serial.print("\r\n");
          Servo_Buffer[i] =  param1 +  param2 * 0xFF;
          ++i;
        }
    break;

    case MW_I2C__CH6:
        i=MW_I2C__CH6;
        while (Wire.available() && (i<18))
        {
          param1 = Wire.read();
          param2 = Wire.read();
          // Serial.print(param1,HEX);
          // Serial.print("\r\n");
          // Serial.print(param2,HEX);
          // Serial.print("\r\n");
          Servo_Buffer[i] =  param1 +  param2 * 0xFF;
          ++i;
        }
    break;

    case MW_I2C__CH7:
        i=MW_I2C__CH7;
        while (Wire.available() && (i<18))
        {
          param1 = Wire.read();
          param2 = Wire.read();
          // Serial.print(param1,HEX);
          // Serial.print("\r\n");
          // Serial.print(param2,HEX);
          // Serial.print("\r\n");
          Servo_Buffer[i] =  param1 +  param2 * 0xFF;
          ++i;
        }
    break;

    case MW_I2C__CH8:
        i=MW_I2C__CH8;
        while (Wire.available() && (i<18))
        {
          param1 = Wire.read();
          param2 = Wire.read();
          // Serial.print(param1,HEX);
          // Serial.print("\r\n");
          // Serial.print(param2,HEX);
          // Serial.print("\r\n");
          Servo_Buffer[i] =  param1 +  param2 * 0xFF;
          ++i;
        }
    break;

    case MW_I2C__CH9:
        i=MW_I2C__CH9;
        while (Wire.available() && (i<18))
        {
          param1 = Wire.read();
          param2 = Wire.read();
          // Serial.print(param1,HEX);
          // Serial.print("\r\n");
          // Serial.print(param2,HEX);
          // Serial.print("\r\n");
          Servo_Buffer[i] =  param1 +  param2 * 0xFF;
          ++i;
        }
    break;

    case MW_I2C__CH10:
        i=MW_I2C__CH10;
        while (Wire.available() && (i<18))
        {
          param1 = Wire.read();
          param2 = Wire.read();
          // Serial.print(param1,HEX);
          // Serial.print("\r\n");
          // Serial.print(param2,HEX);
          // Serial.print("\r\n");
          Servo_Buffer[i] =  param1 +  param2 * 0xFF;
          ++i;
        }
    break;

    case MW_I2C__CH11:
        i=MW_I2C__CH11;
        while (Wire.available() && (i<18))
        {
          param1 = Wire.read();
          param2 = Wire.read();
          // Serial.print(param1,HEX);
          // Serial.print("\r\n");
          // Serial.print(param2,HEX);
          // Serial.print("\r\n");
          Servo_Buffer[i] =  param1 +  param2 * 0xFF;
          ++i;
        }
    break;

    case MW_I2C__CH12:
        i=MW_I2C__CH12;
        while (Wire.available() && (i<18))
        {
          param1 = Wire.read();
          param2 = Wire.read();
          // Serial.print(param1,HEX);
          // Serial.print("\r\n");
          // Serial.print(param2,HEX);
          // Serial.print("\r\n");
          Servo_Buffer[i] =  param1 +  param2 * 0xFF;
          ++i;
        }
    break;

    case MW_I2C__CH13:
        i=MW_I2C__CH13;
        while (Wire.available() && (i<18))
        {
          param1 = Wire.read();
          param2 = Wire.read();
          // Serial.print(param1,HEX);
          // Serial.print("\r\n");
          // Serial.print(param2,HEX);
          // Serial.print("\r\n");
          Servo_Buffer[i] =  param1 +  param2 * 0xFF;
          ++i;
        }
    break;

    case MW_I2C__CH14:
        i=MW_I2C__CH14;
        while (Wire.available() && (i<18))
        {
          param1 = Wire.read();
          param2 = Wire.read();
          // Serial.print(param1,HEX);
          // Serial.print("\r\n");
          // Serial.print(param2,HEX);
          // Serial.print("\r\n");
          Servo_Buffer[i] =  param1 +  param2 * 0xFF;
          ++i;
        }
    break;

    case MW_I2C__CH15:
        i=MW_I2C__CH15;
        while (Wire.available() && (i<18))
        {
          param1 = Wire.read();
          param2 = Wire.read();
          // Serial.print(param1,HEX);
          // Serial.print("\r\n");
          // Serial.print(param2,HEX);
          // Serial.print("\r\n");
          Servo_Buffer[i] =  param1 +  param2 * 0xFF;
          ++i;
        }
    break;

    case MW_I2C__CH16:
        i=MW_I2C__CH16;
        while (Wire.available() && (i<18))
        {
          param1 = Wire.read();
          param2 = Wire.read();
          // Serial.print(param1,HEX);
          // Serial.print("\r\n");
          // Serial.print(param2,HEX);
          // Serial.print("\r\n");
          Servo_Buffer[i] =  param1 +  param2 * 0xFF;
          ++i;
        }
    break;

    case MW_I2C__CH17:
        i=MW_I2C__CH17;
        while (Wire.available() && (i<18))
        {
          param1 = Wire.read();
          param2 = Wire.read();
          // Serial.print(param1,HEX);
          // Serial.print("\r\n");
          // Serial.print(param2,HEX);
          // Serial.print("\r\n");
          Servo_Buffer[i] =  param1 +  param2 * 0xFF;
          ++i;
        }
    break;

    case MW_I2C__RA_LED0_ON_L:
        //Serial.print("MW_I2C__RA_LED0_ON_L\r\n");
        while (Wire.available() && (i<18))
        {
          param1 = Wire.read();
          param2 = Wire.read();
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
        Serial.print(Wire.read(),HEX);
        Serial.print("\r\n");
      }

  Serial.print("Servo_Buffer:\r\n");
  for (i=0;i<8;++i)
  {
    Serial.print(Servo_Buffer[i]);
    Serial.print("  ");
  }
  //Serial.print("\r\n");
      // Servo_Buffer[ROLL] = Servo_Buffer[0];
      // Servo_Buffer[PITCH] = Servo_Buffer[1];
      // Servo_Buffer[THROTTLE] = Servo_Buffer[2];
      // Servo_Buffer[YAW] = Servo_Buffer[3]; 
      // Servo_Buffer[AUX1] = Servo_Buffer[4]; 
      // Servo_Buffer[AUX2] = Servo_Buffer[5]; 
      // Servo_Buffer[AUX3] = Servo_Buffer[6]; 
      // Servo_Buffer[AUX4] = Servo_Buffer[7];
  atomic_update =0;
}

void zeroI2C ()
{
  if (atomic_update ==1) return;
  atomic_update = 1;

  for (int i=0;i<8;++i)
  {
    Servo_Buffer[i] = 0;
  }

  Servo_Buffer[ROLL]     = Servo_Buffer[0];
  Servo_Buffer[PITCH]    = Servo_Buffer[1];
  Servo_Buffer[THROTTLE] = Servo_Buffer[2];
  Servo_Buffer[YAW]      = Servo_Buffer[3]; 
  Servo_Buffer[AUX1]     = Servo_Buffer[4]; 
  Servo_Buffer[AUX2]     = Servo_Buffer[5]; 
  Servo_Buffer[AUX3]     = Servo_Buffer[6]; 
  Servo_Buffer[AUX4]     = Servo_Buffer[7];

  atomic_update = 0;
}
void configureI2CSlave()
{
    //SerialWrite(1,0x64);
    blinkLED(2,120,4); //global_conf.currentSet+1);
    //SerialWrite (0,'C');
    //SerialEnd(0);
    Serial.print("configureI2CSlave initialized at ");

    // Start the I2C Bus as Slave on address 9
    Wire.setClock(400000L) ;
    Wire.begin(MW_I2C__PRIMARY_ADDRESS); 
    //Serial.print(MW_I2C__PRIMARY_ADDRESS, HEX);
    //Serial.print(" \r\n");
    // Attach a function to trigger when something is received.
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);

}


