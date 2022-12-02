#include <zephyr/drivers/gpio.h>
#include "gpioDev.h"
#include "callbacks.h"

bool setupGPIO(const struct device* gp_cont)
{
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
		if(ret != 0)
		{ 
			printk("Error %d: failed to configure INT pin\n", ret);
			return false;
		}
		ret = gpio_pin_configure_dt(&button3_spec, GPIO_INPUT);
		if(ret != 0)
		{ 
			printk("Error %d: failed to configure button pin\n", ret);
			return false;
		}
	}
	ret = gpio_pin_interrupt_configure(gp_cont, INT_PIN, GPIO_INT_LOW_0);
	if(ret != 0) 
	{
		printk("Error %d: failed to configure interrupt for pin %d\n", ret, INT_PIN);
		return false;
	} 
	ret = gpio_pin_interrupt_configure_dt(&button3_spec, GPIO_INT_EDGE_TO_ACTIVE);
	if(ret != 0) 
	{
		printk("Error %d: failed to configure interrupt for pin %d\n", ret, button3_spec.pin);
		return false;
	}
	//int pin setup
	static struct gpio_callback interrupt_callback_data;
	gpio_init_callback(&interrupt_callback_data, int_triggered, BIT(INT_PIN));
	gpio_add_callback(gp_cont, &interrupt_callback_data);
	//button setup
	static struct gpio_callback button3_pressed;
	gpio_init_callback(&button3_pressed, proxyBLE, BIT(button3_spec.pin));
	gpio_add_callback(button3_spec.port, &button3_pressed);
	return true;
}