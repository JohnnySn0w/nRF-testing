/*
 * Copyright (c) 2022 Michael Mahan
 *
 */
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
// #include <nrfx_gpiote.h>


#define SLEEPTIME				2000
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
#define INT_PIN					4
#define INT_PIN_CONFIG			(GPIO_INPUT|GPIO_ACTIVE_HIGH|GPIO_INT_ENABLE|GPIO_PULL_UP)
// #define I2C_NODE				DT_NODELABEL(vcnl40401)
// #define GPIO_NODE				DT_NODELABEL(gpio1)

bool readINT(const struct device* dev, uint8_t* i2c_buffer);

// struct intFlags {
// 	0b10000000: Reserved,
// 	0b1000000: PS_SPFLAG, //PS entering protection mode
// 	0b100000: ALS_IF_L, //ALS crossing low THD INT trigger event 8192
// 	0b10000: ALS_IF_H, //ALS crossing high THD INT trigger event 4096
// 	0b1000: Reserved,
// 	0b100: Reserved,
// 	0b10: PS_IF_CLOSE, PS rises above PS_THDH INT trigger event
// 	0b1: PS_IF_AWAY //PS drops below PS_THDL INT trigger event
// };

typedef struct bufMsg{
	int regAddr; 
	int LByte;
	int HByte;
} bufMsg;

int INTflag = 0;

bool vcnlWrite(const struct device* dev, uint8_t* buf[3], bufMsg* msg){
	buf[0] = msg->regAddr;
	buf[1] = msg->LByte;
	buf[2] = msg->HByte;
	int error = i2c_write(dev, buf, 3, VCNL4040_I2C_ADDR);
	if(error < 0)
	{
		printk("error with write %d, to address: %u!\n", error, msg->regAddr);
		return false;
	} else { return true; }
}

bool vcnlRead(const struct device* dev, uint8_t* i2c_buffer, int* reg)
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

	printk("Reading from %X channel: %u\n", reg, (i2c_buffer[1]<<8) + (i2c_buffer[0]));
	k_msleep(SLEEPTIME);
	return true;
}

void int_triggered(const struct device* dev, struct gpio_callback* callb, uint32_t pin)
{
	INTflag=1;
}

bool setup(const struct device* dev, uint8_t* i2c_buffer) 
{
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
	return true;
}

bool readINT(const struct device* dev, uint8_t* i2c_buffer)
{
	int error; 
	//set read to als register
	i2c_buffer[0]= VCNL4040_INT_FLAG;
	// i2c_buffer[1]=VCNL4040_I2C_ADDR;
	error = i2c_write_read(dev, VCNL4040_I2C_ADDR, i2c_buffer, 1, i2c_buffer, 2);

	if(error < 0)
	{
		printk("error with read %d!\n", error);
		return false;
	}

	printk("Reading from INT FLAG: %u\n", (i2c_buffer[1]<<8) + (i2c_buffer[0]));
	return true;
}

void main(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
	const struct device *gp_cont = DEVICE_DT_GET(DT_NODELABEL(gpio0));
	// const struct device *interrupt_pin = DT_CHILD(gp_cont, DT_NODELABEL());
	uint8_t i2c_buffer[3];
	printk("setting up");
	if (!device_is_ready(dev)) 
	{
		// No such node, or the node does not have status "okay". 
		printk("\nError: no device found.\n");
		return;
	} else if(!setup(dev, i2c_buffer)) 
	{
		printk("setup failed");
		return;
	}
	int ret;
	if (!device_is_ready(gp_cont))
	{
		printk("gpio not ready :(");
		return;
	} 
	else
	{
		ret = gpio_pin_configure(gp_cont, INT_PIN, GPIO_INPUT|GPIO_ACTIVE_LOW|GPIO_INT_ENABLE|GPIO_PULL_UP);
		if(ret != 0){ 
			printk("Error %d: failed to configure pin", ret);
			return;
		}
	}
	
	ret = gpio_pin_interrupt_configure(gp_cont, INT_PIN, GPIO_INT_LOW_0);
	if(ret != 0) 
	{
		printk("Error %d: failed to configure interrupt for pin", ret);
		return;
	}

	//setup callback
	static struct gpio_callback interrupt_callback_data; 
	gpio_init_callback(&interrupt_callback_data, int_triggered, BIT(INT_PIN));
	gpio_add_callback(gp_cont, &interrupt_callback_data);

	// struct k_timer timer;
	//poll
	while(readALS(dev, i2c_buffer))
	{
		printk("Another %d\n", INTflag);
		int pinout = gpio_pin_get_raw(gp_cont, INT_PIN);
		printk("input is %d\n", (pinout));
		if(INTflag ==1||pinout ==0)
		{
			readINT(dev, i2c_buffer);
			INTflag = 0;
		}
		k_msleep(SLEEPTIME);
	}
}
