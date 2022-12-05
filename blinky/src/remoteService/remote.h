#ifndef REMOTE_H_
#define REMOTE_H_

#include <kernel.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/hci.h>


// #include <logging/log.h>

/* ADVERTISING */
//Services
#define BT_UUID_REMOTE_SERV_VAL \
    BT_UUID_128_ENCODE(0x8bd5ce50, 0x82fa, 0x46d8, 0x927b, 0x3b35eb3b4d7a)
//characteristics
#define BT_UUID_SET_ACC_CHRC_VAL \
    BT_UUID_128_ENCODE(0xbf1bd7cc, 0x1f3d, 0x4e5a, 0xb2d1, 0x1352417835b2)

#define BT_REMOTE_SERVICE BT_UUID_DECLARE_128(BT_UUID_REMOTE_SERV_VAL)
#define BT_REMOTE_SET_ACC_CHRC BT_UUID_DECLARE_128(BT_UUID_SET_ACC_CHRC_VAL)

static struct bt_conn *current_conn;

static ssize_t read_acc_characteristic_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset);

void on_connected(struct bt_conn *conn, uint8_t err);
void on_disconnected(struct bt_conn *conn, uint8_t reason);

void set_acc_value(uint8_t btn_value);

void bt_ready(int err);

bool bluetoothInit(struct bt_conn_cb *bt_cb);

bool bluetoothDisable();

#endif