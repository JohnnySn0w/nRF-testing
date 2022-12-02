#include "remote.h"

// #define LOG_MODULE_NAME remote 
// LOG_MODULE_REGISTER(LOG_MODULE_NAME);

bool bluetoothInit(void)
{
    int err;
    printk("initializing bluetooth\n");

    err = 0;

    if (err != 0)
    {
        printk("Bluetooth init failed w/ error %d\n", err);
        return false;
    }
    return true;
}