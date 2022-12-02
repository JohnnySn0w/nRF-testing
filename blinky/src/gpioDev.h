#ifndef GPIODEV_H_
#define GPIODEV_H_

#include <zephyr/drivers/gpio.h>

#define BUTTON3					DT_NODELABEL(button3)
#define INT_PIN					4
#define INT_PIN_CONFIG			(GPIO_INPUT|GPIO_ACTIVE_HIGH|GPIO_INT_ENABLE|GPIO_PULL_UP)

static const struct gpio_dt_spec button3_spec = GPIO_DT_SPEC_GET(BUTTON3, gpios);

bool setupGPIO(const struct device* gp_cont);

#endif 