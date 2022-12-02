#ifndef VCNL4040_H_
#define VCNL4040_H_

#define VCNL4040_I2C_ADDR		0x60
#define VCNL4040_H_THRESH_REG	0x01
#define VCNL4040_H_THRESH_H		0x75 //30000
#define VCNL4040_H_THRESH_L		0x30 //0x7530
#define VCNL4040_L_THRESH_REG	0x02
#define VCNL4040_L_THRESH_H		0x00 //63
#define VCNL4040_L_THRESH_L		0x3f
#define VCNL4040_ALS_REG		0x09 //als data
#define VCNL4040_ALS_CONF_REG	0x00
#define VCNL4040_ALS_CONF_SET_L	0b00000010 //7:6 80ms, 5:4 res, 3:2 int persist(1), 1 int on, 0 als on
#define VCNL4040_INT_FLAG 		0x0B
#define VCNL4040_ALS_CONF_SET_H 0b00000000 //7:0 all reserved

typedef struct bufMsg{
	int regAddr; 
	int LByte;
	int HByte;
} bufMsg;

typedef struct calVals{
	uint16_t ambH;
	uint16_t ambL;
} calVals;

bool vcnlWrite(const struct device* dev, uint8_t* i2c_buffer, bufMsg* msg);
bool vcnlRead(const struct device* dev, uint8_t* i2c_buffer, uint8_t reg);
bool sensorSetup(const struct device* dev, uint8_t* i2c_buffer);
void calibrate(const struct device* dev, uint8_t* buf, calVals* calibVals);
void pollALS(const struct device* dev, uint8_t* i2c_buffer);


#endif 