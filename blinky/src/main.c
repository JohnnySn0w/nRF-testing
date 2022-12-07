/*
 * Copyright (c) 2022 Michael Mahan
 *
 */
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include "vcnl4040.h"
#include "gpioDev.h"
#include "callbacks.h"
#include "remoteService/remote.h"
// #include <nrfx_gpiote.h>


#define SLEEPTIME				2000
extern bool sleepFlag;
extern bool bleConnFlag;

struct bt_conn_cb bluetooth_callbacks = {
    .connected = on_connected,
    .disconnected = on_disconnected,
};

struct bt_account_service_cb account_service_callbacks = {
	.data_received = on_data_received,
};

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

void main(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
	const struct device *gp_cont = DEVICE_DT_GET(DT_NODELABEL(gpio0));
	uint8_t i2c_buffer[3];
	
	if(!sensorSetup(dev, i2c_buffer)) 
	{
		return;
	}

	if(!setupGPIO(gp_cont))
	{
		return;
	}
	// bluetooth_callbacks;
	if(!bluetoothInit(&bluetooth_callbacks, &account_service_callbacks))
	{
		return;
	}
	
	//create and start minute countdown
	createTimer();
	startTimer();
	vcnlRead(dev, i2c_buffer, VCNL4040_INT_FLAG);//primes int pin, otherwise locks in int state
	while(true)
	{
		if(!sleepFlag)
		{
			pollALS(dev, i2c_buffer);
			k_msleep(SLEEPTIME);
		}
		// } else {bluetoothDisable();}
		k_msleep(SLEEPTIME); //prevents loop from locking?
	}
}
