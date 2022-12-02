#ifndef CB_H_
#define CB_H_

#include <zephyr/drivers/gpio.h>


void startTimer();

void proxyBLE(const struct device *gpioButton, struct gpio_callback *callback, gpio_port_pins_t pins);

void int_triggered(const struct device* dev, struct gpio_callback* callb, uint32_t pin);

void sleepFuncCb(struct k_timer *dummy);

void createTimer();


#endif