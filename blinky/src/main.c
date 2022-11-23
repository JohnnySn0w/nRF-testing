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

typedef struct bufMsg{
	int regAddr; 
	int LByte;
	int HByte;
} bufMsg;

typedef struct calVals{
	uint16_t ambH;
	uint16_t ambL;
} calVals;

bool vcnlWrite(const struct device* dev, uint8_t* buf, bufMsg* msg);
bool vcnlRead(const struct device* dev, uint8_t* i2c_buffer, uint8_t reg);
void int_triggered(const struct device* dev, struct gpio_callback* callb, uint32_t pin);
bool sensorSetup(const struct device* dev, uint8_t* i2c_buffer);
bool setupGPIO(const struct device* gp_cont);
void calibrate(uint8_t* buf, calVals* calibVals);
void pollALS(const struct device* dev, uint8_t* i2c_buffer);

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

//global flag
int INTflag = 0;
int freshCalib = 1;

bool vcnlWrite(const struct device* dev, uint8_t* i2c_buffer, bufMsg* msg){
	i2c_buffer[0] = msg->regAddr;
	i2c_buffer[1] = msg->LByte;
	i2c_buffer[2] = msg->HByte;
	int error = i2c_write(dev, i2c_buffer, 3, VCNL4040_I2C_ADDR);
	if(error < 0)
	{
		printk("error with write %d, to address: %X!\n", error, msg->regAddr);
		return false;
	} else { return true; }
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

void int_triggered(const struct device* dev, struct gpio_callback* callb, uint32_t pin)
{
	INTflag=1;
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

bool setupGPIO(const struct device* gp_cont){
	int ret; 
	printk("setting up gpio\n");
	if (!device_is_ready(gp_cont))
	{
		printk("GPIO not ready.\n");
		return false;
	} 
	else
	{
		ret = gpio_pin_configure(gp_cont, INT_PIN, INT_PIN_CONFIG);
		if(ret != 0){ 
			printk("Error %d: failed to configure pin", ret);
			return false;
		}
	}
	ret = gpio_pin_interrupt_configure(gp_cont, INT_PIN, GPIO_INT_LOW_0);
	if(ret != 0) 
	{
		printk("Error %d: failed to configure interrupt for pin", ret);
		return false;
	} 
	static struct gpio_callback interrupt_callback_data;
	gpio_init_callback(&interrupt_callback_data, int_triggered, BIT(INT_PIN));
	gpio_add_callback(gp_cont, &interrupt_callback_data);
	return true;
}

void calibrate(uint8_t* buf, calVals* calibVals)
{
	// uint16_t reading = ((uint16_t)(buf[1])<<8) + (uint16_t)(buf[0]);
	// if(freshCalib == 1)
	// {
	// 	calibVals->ambH = reading+1;
	// 	calibVals->ambL = reading-1;
	// 	freshCalib = 0;
	// 	return;
	// }
	// if(reading > calibVals->ambH)
	// {
	// 	calibVals->ambH = reading;
	// 	printk("updated ambH to %x\n", calibVals->ambH);
	// }
	// if(reading < calibVals->ambL && reading > 0)
	// {
	// 	calibVals->ambH = reading;
	// 	printk("updated ambL to %x\n", calibVals->ambL);
	// }
}

void pollALS(const struct device* dev, uint8_t* i2c_buffer)
{
	//poll ALS
	calVals calibValues; //store highest and lowest ambients
	// calibValues.ambH = 65535;
	// calibValues.ambL = 0; 
	while(vcnlRead(dev, i2c_buffer, VCNL4040_ALS_REG))
	{
		calibrate(i2c_buffer, &calibValues);
		if(INTflag == 1)
		{
			printk("interrupt was triggered\n");
			vcnlRead(dev, i2c_buffer, VCNL4040_INT_FLAG);
			INTflag = 0;
		}
		k_msleep(SLEEPTIME);
	}
}

void main(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
	const struct device *gp_cont = DEVICE_DT_GET(DT_NODELABEL(gpio0));
	// const struct device *interrupt_pin = DT_CHILD(gp_cont, DT_NODELABEL());
	uint8_t i2c_buffer[3];
	
	if(!sensorSetup(dev, i2c_buffer)) 
	{
		printk("Sensor setup failed!\n");
		return;
	}

	if(!setupGPIO(gp_cont))
	{
		printk("GPIO setup failed.");
		return;
	}

	pollALS(dev, i2c_buffer);
}
