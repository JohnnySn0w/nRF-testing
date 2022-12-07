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
extern bool buttonFlag;
extern bool bleConnFlag;
static struct bt_account_service_cb account_service_callbacks;

//advertising data
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL|BT_LE_AD_NO_BREDR)), //support advert, block bt classic
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN)
};
//response data
static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_ACC_SERV_VAL),
};

//cbs
static ssize_t read_acc_characteristic_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &acc_value, sizeof(acc_value));
}

static ssize_t write_acc_characteristic_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    printk("Received data, handle %d, conn %p", attr->handle, (void *)conn);

    if (account_service_callbacks.data_received)
    {
        account_service_callbacks.data_received(conn, buf, len);
    }
    return len;
}
void on_data_received(struct bt_conn *conn, const uint8_t *const data,uint16_t len)
{
    uint8_t temp_str[len+1];
    memcpy(temp_str, data, len);
    temp_str[len] = 0x00;
    printk("Received data on conn %p, len: %d\n", (void *)conn,len);
    printk("Data: %s\n",temp_str);
}    
BT_GATT_SERVICE_DEFINE(
    account_srv,
    BT_GATT_PRIMARY_SERVICE(BT_ACC_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_GET_ACC_CHRC,
    BT_GATT_CHRC_READ,
    BT_GATT_PERM_READ,
    read_acc_characteristic_cb,
    NULL,
    NULL),
    BT_GATT_CHARACTERISTIC(BT_SET_ACC_CHRC,
        BT_GATT_CHRC_WRITE_WITHOUT_RESP, //quicker than waiting for resp, tho we may want resp for validation later
        BT_GATT_PERM_WRITE,
        NULL,
        write_acc_characteristic_cb,
        NULL
    ),
);

void on_connected(struct bt_conn *conn, uint8_t err)
{
    if(err)
    {
        printk("connection error %d\n", err);
        return;
    }
    printk("bluetooth connected\n");
    bleConnFlag = true;
    current_conn = bt_conn_ref(conn);
    set_acc_value(acc_value+1);
}

void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("Disconnected reason %d\n", reason);
    if (current_conn)
    {
        bt_conn_unref(current_conn);
        current_conn = NULL;
        bleConnFlag = false;
    }    
}

void set_acc_value(uint8_t val)
{
    acc_value = val;
    printk("updated account status\n");
}

void bt_ready(int err)
{	
	if (err)
	{
		printk("bt return error: %d\n", err);
	}
	k_sem_give(&bt_init_ok);
}

bool bluetoothInit(struct bt_conn_cb *bt_cb, struct bt_account_service_cb *account_cb)
{
    int err;
    printk("initializing bluetooth\n");

    if (bt_cb == NULL)
    {
        return -NRFX_ERROR_NULL;
    }

    bt_conn_cb_register(bt_cb);
    account_service_callbacks.data_received = account_cb->data_received;
     
    err = 0;

    err = bt_enable(bt_ready);
    if (err != 0)
    {
        printk("Bt_enable failed w/ error %d\n", err);
        return false;
    }
    buttonFlag = true;
    k_sem_take(&bt_init_ok, K_FOREVER); //indef wait on semaphore 
    
    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if(err != 0)
    {
        printk("Couldn't start advertising, error: %d\n", err);
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