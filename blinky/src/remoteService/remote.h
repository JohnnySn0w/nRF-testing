#ifndef REMOTE_H_
#define REMOTE_H_

#include <kernel.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/hci.h>


// #include <logging/log.h>

/* ADVERTISING */
/* Services */
#define BT_UUID_ACC_SERV_VAL \
    BT_UUID_128_ENCODE(0x8bd5ce50, 0x82fa, 0x46d8, 0x927b, 0x3b35eb3b4d7a)

#define BT_UUID_OPERATING_SERV_VAL \
    BT_UUID_128_ENCODE(0xfd169afd, 0xcd31, 0x433f, 0x809b, 0xb662c185d699)

/* 
#define BT_UUID_BATTERY_SERV_VAL BT_UUID_16_ENCODE(0x180F) 
*/

/* Characteristics */
//account
#define BT_UUID_SET_ACC_CHRC_VAL \
    BT_UUID_128_ENCODE(0xbf1bd7cc, 0x1f3d,0x4e5a, 0xb2d1, 0x1352417835b2)
#define BT_UUID_GET_ACC_CHRC_VAL \
    BT_UUID_128_ENCODE(0x3c30ddcd, 0x4609, 0x4776, 0xb0bf, 0x763af721c683)
//operating

#define BT_UUID_OPER_STATUS_CHRC_VAL \
    BT_UUID_128_ENCODE(0x23aa4d85, 0x801c, 0x4a67, 0xa228, 0x2512031ce696)
#define BT_UUID_OPER_STATUS_REBOOT_CHRC_VAL \
    BT_UUID_128_ENCODE(0x58ccf955, 0xfe58, 0x4727, 0xbd33, 0x58f779a19690)

//battery
/*/
#define BT_UUID_GET_BATTERY_CHRC_VAL BT_UUID_16_ENCODE(0x2A19)
*/
/* UUID declarations*/
#define BT_ACC_SERVICE BT_UUID_DECLARE_128(BT_UUID_ACC_SERV_VAL)
#define BT_SET_ACC_CHRC BT_UUID_DECLARE_128(BT_UUID_SET_ACC_CHRC_VAL)
#define BT_GET_ACC_CHRC BT_UUID_DECLARE_128(BT_UUID_GET_ACC_CHRC_VAL)

#define BT_OPERATING_SERVICE BT_UUID_DECLARE_128(BT_UUID_OPERATING_SERV_VAL)
#define BT_OPER_STATUS_CHRC BT_UUID_DECLARE_128(BT_UUID_OPER_STATUS_CHRC_VAL)
#define BT_OPER_STATUS_REBOOT_CHRC BT_UUID_DECLARE_128(BT_UUID_OPER_STATUS_REBOOT_CHRC_VAL)

/*
#define BT_BATTERY_SERVICE  BT_UUID_DECLARE_16(BT_UUID_BATTERY_SERV_VAL)
#define BT_GET_BATTERY_CHRC BT_UUID_DECLARE_16(BT_UUID_GET_BATTERY_CHRC_VAL)
*/



struct bt_account_service_cb {
    void (*data_received)(struct bt_conn *conn, const uint8_t *const data, uint16_t len);
};

//struct funcs
void on_connected(struct bt_conn *conn, uint8_t err);
void on_disconnected(struct bt_conn *conn, uint8_t reason);
void on_data_received(struct bt_conn *conn, const uint8_t *const data, uint16_t len);

void set_acc_value(const uint8_t *const val);

//check if bt is ready to be used
void bt_ready(int err);

/* utility funcs */
bool bluetoothInit(struct bt_conn_cb *bt_cb, struct bt_account_service_cb *account_cb);
bool bluetoothDisable();

#endif