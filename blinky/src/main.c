/*
 * Copyright (c) 2022 Michael Mahan
 *
 * oh that was fun
 */
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/i2c.h>

#define SLEEPTIME				2000
#define VCNL4040_I2C_ADDR		0x60
// #define VCNL4040_ID_REG			0x0C
#define VCNL4040_ALS_REG		0x09 //data
#define VCNL4040_ALS_CONF_REG	0x00
#define VCNL4040_ALS_CONF_SET_L	0b1100000
#define VCNL4040_ALS_CONF_SET_H 0b0000000
#define I2C_NODE				DT_NODELABEL(vcnl40401)

void main(void)
{
	const struct device *dev = device_get_binding("I2C_1");
	uint8_t i2c_buffer[3];
	// uint8_t i2c_read_buffer[2];
	int error; 
	
	if (dev == NULL) 
	{
		// No such node, or the node does not have status "okay". 
		printk("\nError: no device found.\n");
		return;
	}

	// read id as test
	// i2c_buffer[0] = VCNL4040_ID_REG; //set to id register
	// error = i2c_write_read(dev, VCNL4040_I2C_ADDR, i2c_buffer, 1, i2c_buffer, 2);
	// if(error < 0)
	// {
	// 	printk("error with reading id %d!\n", error);
	// 	return;
	// } else { printk("dev id: %d\n", i2c_buffer[0] + i2c_buffer[1]); }

	while(true)
	{
		i2c_buffer[0] = VCNL4040_ALS_CONF_REG; //send to als config command
		i2c_buffer[1] = VCNL4040_ALS_CONF_SET_L; //high byte
		i2c_buffer[2] = VCNL4040_ALS_CONF_SET_H; //low byte
		
		//config thru i2c, set initial conifg for als to be on and such
		error = i2c_write(dev, i2c_buffer, 3, VCNL4040_I2C_ADDR);
		if(error < 0)
		{
			printk("error with conf %d!\n", error);
		}

		//set read to als register
		i2c_buffer[0]= VCNL4040_ALS_REG;
		error = i2c_write_read(dev, VCNL4040_I2C_ADDR, i2c_buffer, 1, i2c_buffer, 2);
		if(error < 0)
		{
			printk("error with read %d!\n", error);
		}

		printk("Reading from sensor: %d\n", (i2c_buffer[1] & 0x0F) + (i2c_buffer[0]));
		k_msleep(SLEEPTIME);
	}
}
