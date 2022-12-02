#include "callbacks.h"

#define AMBIENT_COUNTDOWN		10
struct k_timer ambientTimer;
extern bool buttonFlag;
extern bool sleepFlag; 
extern bool INTflag;

void proxyBLE(const struct device *gpioButton, struct gpio_callback *callback, gpio_port_pins_t pins)
{
	buttonFlag = !buttonFlag;
}

void int_triggered(const struct device* dev, struct gpio_callback* callb, uint32_t pin)
{
	INTflag = true;
	sleepFlag = false;
	startTimer();
}

void sleepFuncCb(struct k_timer *dummy)
{
	sleepFlag = true; 
	//put device into sleep mode, waiting for interrupt
}

void createTimer()
{
    k_timer_init(&ambientTimer, sleepFuncCb, NULL);
}

void startTimer()
{
	k_timer_start(&ambientTimer, K_SECONDS(AMBIENT_COUNTDOWN),K_SECONDS(0));
}