#include <zephyr/sys/printk.h>
#include <zephyr/drivers/i2c.h>
#include "vcnl4040.h"

extern bool buttonFlag;
extern bool INTflag;

bool vcnlWrite(const struct device* dev, uint8_t* i2c_buffer, bufMsg* msg)
{
	i2c_buffer[0] = msg->regAddr;
	i2c_buffer[1] = msg->LByte;
	i2c_buffer[2] = msg->HByte;
	int error = i2c_write(dev, i2c_buffer, 3, VCNL4040_I2C_ADDR);
	if(error < 0)
	{
		printk("error with write %d, to address: %X!\n", error, msg->regAddr);
		return false;
	}
	return true;
}

bool vcnlRead(const struct device* dev, uint8_t* i2c_buffer, uint8_t reg)
{
	int error; 
	//set read to als register
	i2c_buffer[0] = reg;
	error = i2c_write_read(dev, VCNL4040_I2C_ADDR, i2c_buffer, 1, i2c_buffer, 2);
	if(error < 0)
	{
		printk("error with read %d!\n", error);
		return false;
	}

	printk("Reading from channel %X: %u\n", reg, (i2c_buffer[1]<<8) + (i2c_buffer[0]));
	return true;
}


bool sensorSetup(const struct device* dev, uint8_t* i2c_buffer) 
{
	if (!device_is_ready(dev)) 
	{
		// No such node, or the node does not have status "okay". 
		printk("\nError: no device found.\n");
		return false;
	} 
	printk("setting up sensor\n");
	bufMsg msg;
	msg.regAddr = VCNL4040_ALS_CONF_REG; //send to als config command
	msg.LByte = VCNL4040_ALS_CONF_SET_L; //low byte
	msg.HByte = VCNL4040_ALS_CONF_SET_H; //high byte unneccessary
	//config thru i2c, set initial conifg for als to be on and such
	if(!vcnlWrite(dev, i2c_buffer, &msg))
	{return false;}

	msg.regAddr = VCNL4040_H_THRESH_REG;
	msg.LByte = VCNL4040_H_THRESH_L;
	msg.HByte = VCNL4040_H_THRESH_H;
	if(!vcnlWrite(dev, i2c_buffer, &msg))
	{return false;}

	msg.regAddr = VCNL4040_L_THRESH_REG;
	msg.LByte = VCNL4040_L_THRESH_L;
	msg.HByte = VCNL4040_L_THRESH_H;
	if(!vcnlWrite(dev, i2c_buffer, &msg))
	{return false;}

	return true;
}

void calibrate(const struct device* dev, uint8_t* buf, calVals* calibVals)
{
	static bool freshCalib = true;
	static uint16_t aggre;
	static int reads;
	static int avg;

	uint16_t reading = ((uint16_t)(buf[1])<<8) + (uint16_t)(buf[0]);
	bufMsg msg;
	if(freshCalib == true)
	{
		aggre = reading;
		reads = 1; 
		calibVals->ambH = reading+1;
		calibVals->ambL = reading-1;
		freshCalib = false;
		printk("updated ambH to %u\n", calibVals->ambH);
		printk("updated ambL to %u\n", calibVals->ambL);
		return;
	} else {
		aggre += reading;
		reads += 1;
		if (reads > 20)
		{
			avg = aggre/reads;
			aggre = avg;
			reads = 1;
		}
		printk("aggre is %d. avg is %d. \n", aggre, avg);
	}
	if(reading > calibVals->ambH)
	{
		calibVals->ambH = reading;
		printk("updated ambH to %u\n", calibVals->ambH);
		msg.regAddr = VCNL4040_H_THRESH_REG;
		msg.LByte = buf[0];
		msg.HByte = buf[1];
		if(!vcnlWrite(dev, buf, &msg))
		{return;}
	}
	if(reading < calibVals->ambL)
	{
		calibVals->ambL = reading;
		printk("updated ambL to %u\n", calibVals->ambL);
		msg.regAddr = VCNL4040_L_THRESH_REG;
		msg.LByte = buf[0];
		msg.HByte = buf[1];
		if(!vcnlWrite(dev, buf, &msg))
		{return;}
	}
}

void pollALS(const struct device* dev, uint8_t* i2c_buffer)
{
	//poll ALS
	static calVals calibValues; //store highest and lowest ambients
	vcnlRead(dev, i2c_buffer, VCNL4040_ALS_REG);
	calibrate(dev, i2c_buffer, &calibValues);
	if(INTflag == true)
	{
		printk("interrupt was triggered\n");
		vcnlRead(dev, i2c_buffer, VCNL4040_INT_FLAG);
		INTflag = false;
		printk("looking for bluetooth...\n");
		k_msleep(1000);
	}
	printk("bluetooth %s\n", buttonFlag ? "on!" : "off.");
}