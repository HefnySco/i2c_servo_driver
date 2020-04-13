#ifndef I2C_H_
#define I2C_H_

void i2c_init(void);
void i2c_rep_start(uint8_t address);
void i2c_stop(void);
uint8_t i2c_read(uint8_t ack) ;
uint8_t i2c_readAck();
uint8_t i2c_readNak(void);
void waitTransmissionI2C();
void i2c_read_reg_to_buf(uint8_t add, uint8_t reg, uint8_t *buf, uint8_t size);
void swap_endianness(void *buf, size_t size);
void i2c_getSixRawADC(uint8_t add, uint8_t reg);
void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val);
uint8_t i2c_readReg(uint8_t add, uint8_t reg);


#endif /* I2C_H_ */