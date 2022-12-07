#include <zephyr/bluetooth/services/bas.h>
#include "remote.h"

//semaphore to make sure the bt_enable only calls one at a time.
static K_SEM_DEFINE(bt_init_ok, 1, 1);

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME)-1)

//global
static uint64_t acc_value;
extern bool buttonFlag;
extern bool bleConnFlag;
extern int operatingStatus;
static struct bt_conn *current_conn;
static struct bt_account_service_cb account_service_callbacks;
/* GENERIC CBs for read/write/notify */
// void acc_chrc_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);
static ssize_t read_acc_characteristic_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset);
static ssize_t write_acc_characteristic_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags);

//advertising data
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL|BT_LE_AD_NO_BREDR)), //support advert, block bt classic
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
    // BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)),
};
//response data
static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_ACC_SERV_VAL),
};

/* Callbacks */
// notifies config changed
// void acc_chrc_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
// {
//     bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);
//     printk("Notifications %s\n", notif_enabled ? "enabled\n" : "disabled\n");
// }

static ssize_t read_acc_characteristic_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &acc_value, sizeof(acc_value));
}

static ssize_t write_acc_characteristic_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{

    if (account_service_callbacks.data_received)
    {
        account_service_callbacks.data_received(conn, buf, len);
    }
    set_acc_value(buf);
    return len;
    // return bt_gatt_attr_write_func_t thing (conn, attr, buf, len, offset, &acc_value, sizeof(acc_value));
}

static ssize_t read_operating_characteristic_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &operatingStatus, sizeof(operatingStatus));
}

static ssize_t write_operating_reboot_characteristic_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{

    if (account_service_callbacks.data_received)
    {
        account_service_callbacks.data_received(conn, buf, len);
    }
    return len;
    // return bt_gatt_attr_write_func_t thing (conn, attr, buf, len, offset, &acc_value, sizeof(acc_value));
}


// static ssize_t battery_characteristic_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
// {
//     int bat_level = bt_bas_get_battery_level();
//     return bt_gatt_attr_read(conn, attr, buf, len, offset, &bat_level, sizeof(bat_level));
// }

void on_data_received(struct bt_conn *conn, const uint8_t *const data,uint16_t len)
{
    uint8_t temp_str[len+1];
    memcpy(temp_str, data, len);
    temp_str[len] = 0x00; //null char last 
    printk("Received data on conn %p, len: %d\n", (void *)conn,len);
    printk("Data: %s\n",temp_str);
}    
// static ssize_t write_acc_characteristic_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
// {
//     // return bt_gatt_attr_wr(conn, attr, buf, len, offset, &acc_value, sizeof(acc_value));
// }


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

void set_acc_value(const uint8_t *const val)
{
    //this currently does not flush the old id correctly
    acc_value = 0;
    memcpy(&acc_value, val, 64);
    // acc_value = val;
    printk("updated account value\n");
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
    err = 0;
    err = bt_le_adv_stop();
    if(err != 0)
    {
        printk("Couldn't stop advertising, error: %d\n", err);
        return false;
    }
    k_sem_give(&bt_init_ok);
    
    // sd_softdevice_disable();
    err = bt_disable();
    if (err != 0)
    {
        printk("Bt_disable failed w/ error %d\n", err);
        return false;
    }
    
    
    return true;
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
    )
    // BT_GATT_CCC(acc_chrc_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), //applies to the chrc above it
);

BT_GATT_SERVICE_DEFINE(
    operating_srv,
    BT_GATT_PRIMARY_SERVICE(BT_OPERATING_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_OPER_STATUS_CHRC,
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ,
        read_operating_characteristic_cb,
        NULL,
        NULL
    ),
    BT_GATT_CHARACTERISTIC(BT_OPER_STATUS_REBOOT_CHRC,
        BT_GATT_CHRC_WRITE,
        BT_GATT_PERM_WRITE,
        NULL,
        write_operating_reboot_characteristic_cb,
        NULL
    )
    // BT_GATT_CCC(acc_chrc_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

// BT_GATT_SERVICE_DEFINE(
//     battery_srv,
//     BT_GATT_PRIMARY_SERVICE(BT_BATTERY_SERVICE),
//     BT_GATT_CHARACTERISTIC(BT_GET_BATTERY_CHRC,
//         BT_GATT_CHRC_READ,
//         BT_GATT_PERM_READ,
//         read_acc_characteristic_cb,
//         NULL,
//         NULL)
// );