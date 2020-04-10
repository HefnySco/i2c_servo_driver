### I2C Servo Driver using Arduino

The purpose of this project is to make a simple semi-alternative of **[PCA9685 16-Channel Servo Driver](https://learn.adafruit.com/16-channel-pwm-servo-driver?view=all "PCA9685 16-Channel Servo Driver") **using Arduino boards.

The project make use of the old and excellent code of iconic **[Multiwii](https://github.com/multiwii/multiwii-firmware "Multiwii")**. as Multiwii has very optimized and flexible code for controlling motor and servos.

This code emulates I2C communication with  **[PCA9685 16-Channel Servo Driver](https://learn.adafruit.com/16-channel-pwm-servo-driver?view=all "PCA9685 16-Channel Servo Driver") ** however it is no intended to make all functions.

People are welcome to add and tune functions.

It is worth to note here that it is very simple to use **[Multiwii protocol](https://github.com/multiwii/multiwii-firmware/blob/upstream_shared/Protocol.cpp "Multiwii protocol")** that is already available to send serial commands instead of using I2C.

**STATUS:** still in developig and need lots of code cleansing.