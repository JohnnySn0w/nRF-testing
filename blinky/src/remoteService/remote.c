#include "remote.h"
// #include "../callbacks.h"

// #define LOG_MODULE_NAME remote 
// LOG_MODULE_REGISTER(LOG_MODULE_NAME);

//semaphore to make sure the bt_enable only calls one at a time.
static K_SEM_DEFINE(bt_init_ok, 1,1);

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME)-1)

//global
static uint8_t acc_value = 0;

//advertising data
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL|BT_LE_AD_NO_BREDR)), //support advert, block bt classic
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN)
};
//response data
static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_REMOTE_SERV_VAL),
};

//cbs
static ssize_t read_acc_characteristic_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &acc_value, sizeof(acc_value));
}

BT_GATT_SERVICE_DEFINE(
    remote_srv,
    BT_GATT_PRIMARY_SERVICE(BT_REMOTE_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_REMOTE_SET_ACC_CHRC,
    BT_GATT_CHRC_READ,
    BT_GATT_PERM_READ,
    read_acc_characteristic_cb,
    NULL,
    NULL),
);

void on_connected(struct bt_conn *conn, uint8_t err)
{
    if(err)
    {
        printk("connection error %d\n", err);
        return;
    }
    printk("bluetooth connected\n");
    current_conn = bt_conn_ref(conn);
}

void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("Disconnected reason %d\n", reason);
    if (current_conn)
    {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }    
}

void set_acc_value(uint8_t acc_value)
{
    printk("returning account status");
}

void bt_ready(int err)
{	
	if (err)
	{
		printk("bt return error: %d\n", err);
	}
	k_sem_give(&bt_init_ok);
}

bool bluetoothInit(struct bt_conn_cb *bt_cb)
{
    int err;
    printk("initializing bluetooth\n");

    if (bt_cb == NULL)
    {
        return -NRFX_ERROR_NULL;
    }

    bt_conn_cb_register(bt_cb);
     
    err = 0;

    err = bt_enable(bt_ready);
    if (err != 0)
    {
        printk("Bt_enable failed w/ error %d\n", err);
        return false;
    }

    k_sem_take(&bt_init_ok, K_FOREVER); //indef wait on semaphore 
    
    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if(err != 0)
    {
        printk("Couldn't start advertising, error: %d", err);
        return false;
    }
    
    return true;
}

bool bluetoothDisable()
{
    int err;
    printk("Turning off bluetooth\n");
    
    // if (bt_cb == NULL)
    // {
    //     return -NRFX_ERROR_NULL;
    // }
    // bt_cb.disconnect();
    err = 0;

    err = bt_disable();
    if (err != 0)
    {
        printk("Bt_disable failed w/ error %d\n", err);
        return false;
    }

    k_sem_give(&bt_init_ok);
    // err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    // if(err != 0)
    // {
    //     printk("Couldn't start advertising, error: %d", err);
    //     return false;
    // }
    
    return true;
}