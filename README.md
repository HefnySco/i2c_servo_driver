## I2C Servo Driver using Arduino

### Purpose
The purpose of this project is to make a simple semi-alternative of **[MW_I2C_ 16-Channel Servo Driver](https://learn.adafruit.com/16-channel-pwm-servo-driver?view=all "MW_I2C_ 16-Channel Servo Driver") **using Arduino boards.

The project make use of the old and excellent code of iconic **[Multiwii](https://github.com/multiwii/multiwii-firmware "Multiwii")**. As Multiwii has very optimized and flexible code for controlling motor and servos.


cmd = **MW_I2C_RA_ALL_OUTPUT_SAME**   0xFA
sets all channel to same value.
e.g. {0xFA, 0x00, 0x00}

**Channels:**
MW_I2C_CH0		0
MW_I2C_CH1		0x1
MW_I2C_CH2		0x2
MW_I2C_CH3		0x3
MW_I2C_CH4		0x4
MW_I2C_CH5		0x5
MW_I2C_CH6		0x6
MW_I2C_CH7		0x7
MW_I2C_CH8		0x8
MW_I2C_CH9		0x9
MW_I2C_CH10		0xa
MW_I2C_CH12		0xb
MW_I2C_CH13		0xc
MW_I2C_CH14		0xe
MW_I2C_CH15		0xf
MW_I2C_CH16		0x10
MW_I2C_CH17		0x11

you can send data to a given channel or channels after it.
e.g.
{**MW_I2C_CH0**, 0x4C, 0x04, 0x08, 0x07}
write data to channel 0 & 1

{**MW_I2C_CH3**, 0x4C, 0x04, 0x08, 0x07,  0xDC, 0x05}
write data to channel 3 & 4 & 5


### Output Signals:
output signals are defined using multiwii types in config.h. I added **MW_I2C_ROVER**.
Also these types are mainly used to assign motors and servos, but NO processing or mixing of any type ias applied.


People are welcome to add and tune functions.



**STATUS:** still in developig and need lots of code cleansing.


It is worth to note here that it is simple to make this code uses **[Multiwii protocol](https://github.com/multiwii/multiwii-firmware/blob/upstream_shared/Protocol.cpp "Multiwii protocol")** that is already available to send serial commands instead of using I2C.
