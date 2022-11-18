/*
 * Copyright (c) 2022 Michael Mahan
 *
 */
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/zephyr.h>
#include <zephyr/device.h>
// #include <nrfx_gpiote.h>

#define SMARTERROR printk(("This is line %d of file %s (function %s)\n",\
                      __LINE__, __FILE__, __func__))

#define SLEEPTIME				2000
#define VCNL4040_I2C_ADDR		0x60
#define VCNL4040_H_THRESH_REG	0x01
#define VCNL4040_H_THRESH_H		0x75 //30000
#define VCNL4040_H_THRESH_L		0x30
#define VCNL4040_L_THRESH_REG	0x02
#define VCNL4040_L_THRESH_H		0x00 //15
#define VCNL4040_L_THRESH_L		0xff
#define VCNL4040_ALS_REG		0x09 //data
#define VCNL4040_ALS_CONF_REG	0x00
#define VCNL4040_ALS_CONF_SET_L	0b0000010 //7:6 80ms, 5:4 res, 3:2 int persist, 1 int on, 0 als on
#define VCNL4040_INT_FLAG 		0x0B
// #define VCNL4040_ALS_CONF_SET_H 0b0000000 //7:0 all reserved
#define INT_PIN					10
// #define I2C_NODE				DT_NODELABEL(vcnl40401)
// #define GPIO_NODE				DT_NODELABEL(gpio1)

// struct intFlags {
// 	0b00000000 Reserved,
// 	0b1000000 PS_SPFLAG, //PS entering protection mode
// 	0b100000 ALS_IF_L, //ALS crossing low THD INT trigger event
// 	0b10000 ALS_IF_H, //ALS crossing high THD INT trigger event
// 	0b0000 Reserved,
// 	0b000 Reserved,
// 	0b01 PS_IF_CLOSE, PS rises above PS_THDH INT trigger event
// 	0b1 PS_IF_AWAY //PS drops below PS_THDL INT trigger event
// };

void smartError(const char* msg){
	printk("%s:%d ERROR: %s", __FILE__, __LINE__, msg);
}

// bool vcnlWrite(const struct device* dev, int reg, uint8_t* buf[3]){
// 	buf[0] = reg;
// 	int error = i2c_write(dev, buf, 3, VCNL4040_I2C_ADDR);
// 	if(error < 0)
// 	{
// 		printk("error with conf %d!\n", error);
// 		return false;
// 	} else { return true; }
// }

// bool vcnlRead(const struct device* dev, int reg, uint8_t* i2c_buffer)
// {
// 	int error; 
// 	//set read to als register
// 	i2c_buffer[0]= VCNL4040_ALS_REG;
// 	error = i2c_write_read(dev, VCNL4040_I2C_ADDR, i2c_buffer, 1, i2c_buffer, 2);

// 	if(error < 0)
// 	{
// 		printk("error with read %d!\n", error);
// 		return false;
// 	}

// 	printk("Reading from ALS sensor: %u\n", (i2c_buffer[1]<<8) + (i2c_buffer[0]));
// 	k_msleep(SLEEPTIME);
// 	return true;
// }


void int_triggered(const struct device* dev, struct gpio_callback* callb, uint32_t pins)
{
	smartError("Interrupt triggered\n");
}

bool setup(const struct device* dev, uint8_t* i2c_buffer) 
{
	int error;
	i2c_buffer[0] = VCNL4040_ALS_CONF_REG; //send to als config command
	i2c_buffer[1] = VCNL4040_ALS_CONF_SET_L; //high byte
	// i2c_buffer[2] = VCNL4040_ALS_CONF_SET_H; //low byte unneccessary
	
	//config thru i2c, set initial conifg for als to be on and such
	error = i2c_write(dev, i2c_buffer, 2, VCNL4040_I2C_ADDR);
	if (error < 0)
	{
		printk("error with conf %d!\n", error);
		return false;
	}

	i2c_buffer[0] = VCNL4040_H_THRESH_REG;
	i2c_buffer[1] = VCNL4040_H_THRESH_H;
	i2c_buffer[2] = VCNL4040_H_THRESH_L;
	error = i2c_write(dev, i2c_buffer, 3, VCNL4040_I2C_ADDR);
	if (error < 0)
	{
		printk("error with conf %d!\n", error);
		return false;
	}

	i2c_buffer[0] = VCNL4040_L_THRESH_REG;
	i2c_buffer[1] = VCNL4040_L_THRESH_H;
	i2c_buffer[2] = VCNL4040_L_THRESH_L;
	error = i2c_write(dev, i2c_buffer, 3, VCNL4040_I2C_ADDR);
	if (error < 0)
	{
		printk("error with conf %d!\n", error);
		return false;
	}

	
	
	return true;
}

bool readALS(const struct device* dev, uint8_t* i2c_buffer)
{
	int error; 
	//set read to als register
	i2c_buffer[0]= VCNL4040_ALS_REG;
	error = i2c_write_read(dev, VCNL4040_I2C_ADDR, i2c_buffer, 1, i2c_buffer, 2);

	if(error < 0)
	{
		printk("error with read %d!\n", error);
		return false;
	}

	printk("Reading from ALS sensor: %u\n", (i2c_buffer[1]<<8) + (i2c_buffer[0]));
	k_msleep(SLEEPTIME);
	return true;
}

bool readINT(const struct device* dev, uint8_t* i2c_buffer)
{
	int error; 
	//set read to als register
	i2c_buffer[0]= VCNL4040_INT_FLAG;
	error = i2c_write_read(dev, VCNL4040_I2C_ADDR, i2c_buffer, 1, i2c_buffer, 2);

	if(error < 0)
	{
		printk("error with read %d!\n", error);
		return false;
	}

	printk("Reading from INT FLAG: %d\n", (i2c_buffer[1]<<8) + (i2c_buffer[0]));
	k_msleep(SLEEPTIME);
	return true;
}

// bool setupINT(const struct device* dev, uint8_t* i2c_buffer, struct gpio_callback *cb, uint32_t pins)
// {
    
// 	printk("Interrupt triggered.\n");
// 	return true;
// }

void main(void)
{
	// const struct device *vcnl = device_get_binding("I2C_1");
	const struct device *dev= DEVICE_DT_GET(DT_NODELABEL(i2c1));
	const struct device *gp_cont = DEVICE_DT_GET(DT_NODELABEL(gpio0));
	// const struct device *interrupt_pin = DT_CHILD(gp_cont, DT_NODELABEL());
	
	uint8_t i2c_buffer[3];
	int ret;
	if (!device_is_ready(gp_cont))
	{
		printk("gpio not ready :(");
		return;
	} else { 
		ret = gpio_pin_configure(gp_cont, INT_PIN, GPIO_INPUT); 
	}
	if(ret != 0) 
	{
		printk("Error %d: failed to configure pin", ret);
		return;
	}

	if (dev== NULL) 
	{
		// No such node, or the node does not have status "okay". 
		printk("\nError: no device found.\n");
		return;
	} else if(!setup(dev, i2c_buffer)) 
	{
		return;
	}

	
	ret = gpio_pin_interrupt_configure(gp_cont, INT_PIN, GPIO_ACTIVE_HIGH);
	if(ret != 0) 
	{
		printk("Error %d: failed to configure interrupt for pin", ret);
		return;
	}
	static struct gpio_callback interrupt_callback_data; 
	gpio_init_callback(&interrupt_callback_data, int_triggered, BIT(INT_PIN));
	gpio_add_callback(gp_cont, &interrupt_callback_data);

	while(readALS(dev, i2c_buffer)&&readINT(dev, i2c_buffer))
	{
		// int pinout = gpio_pin_get_raw(gp_cont, INT_PIN);
		// printk("Interrupt is set to %d\n", pinout);
		// readINT(pin13, i2c_buffer);
		// continue;
	}
}
