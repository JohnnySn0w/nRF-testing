#ifndef BLE_H_
#define BLE_H_

#include <zephyr/drivers/gpio.h>

void proxyBLE(const struct device *gpioButton, struct gpio_callback *callback, gpio_port_pins_t pins);

#endif