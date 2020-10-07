/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include <bluetooth/conn.h>

#include <bluetooth/gatt.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt_dm.h>
#include <bluetooth/scan.h>

#include <dk_buttons_and_leds.h>
#include <sys/byteorder.h>

#include <net/nrf_cloud.h>
#include "aggregator.h"

#define BT_UUID_BEACON                                                            \
	BT_UUID_DECLARE_16(0xfeaa)

#define BEACON_UUID_START 4
#define BEACON_UUID_SIZE 16

extern void alarm(void);

/**@brief Function to print strings without null-termination
 */
static void data_print(u8_t *prefix, u8_t *data, size_t len)
{
	char buf[len + 1];

	memcpy(buf, data, len);
	buf[len] = 0;
	printk("%s%s\n", prefix, buf);
}

static void insert_data(const void *data, u8_t len)
{
        // data_print("DEVICE NAME: ", data, len);
        char data_arr[len];
        strcpy(data_arr, data);
        struct sensor_data in_data;

        in_data.type = BEACON_ID;
        in_data.length = len;
//        in_data.data[0] = ((u8_t *)data)[0];
        memcpy(in_data.data, data_arr, len);

        if (aggregator_put(in_data) != 0) {
                printk("Was not able to insert device name data into aggregator.\n");
        }
        printk("AGGREGATOR COUNT: %d\n",aggregator_element_count_get());
//          /* If the thingy is upside down, trigger an alarm. */
//          if (((u8_t *)data)[0] == 3) {
//                  alarm();
//          }
}

static bool adv_data_found(struct bt_data *data, void *user_data)
{
        int i;
        if (data->type == BT_DATA_UUID16_ALL) {
//              printk("Direct advertising received from %s\n", (char *)user_data);
//              printk("--------------\n");
              for (i=0; i < data->data_len; i++) {
//                  printk("%x", data->data[i]);
              }
//              printk("\n");
              insert_data(user_data, strlen((char *)user_data));
        }
        if (data->type == BT_DATA_SVC_DATA16) {
//              printk("--------------\n");
              for (i=0; i < data->data_len; i++) {
//                  printk("%x", data->data[i]);
              }
//              printk("\n");
        }
        if (data->type == BT_DATA_NAME_COMPLETE) {
//              printk("--------------\n");
              for (i=0; i < data->data_len; i++) {
//                  printk("%c", data->data[i]);
              }
//              printk("\n");
        }
//	if(data->type== BT_DATA_MANUFACTURER_DATA) {
//           printk("--------------\n");
//           printk("Manuf. data: ");
//	   for(i=0; i < data->data_len; i++){
//                printk("%x, ", data->data[i]);
//           }
//           printk("\n");
//           printk("Beacon uuid: ");
//           for(i=0; i < BEACON_UUID_SIZE; i++){
//                index = i + BEACON_UUID_START;
//                beacon_uuid[i] = (uint8_t)data->data[index];
//                printk("%x,", data->data[index]);
//           }
//           printk("\n");
//
//           printk("Beacon major: ");
//           for(i=0; i < BEACON_MAJOR_SIZE; i++){
//                index = i + BEACON_MAJOR_START;
//                beacon_major[i] = (uint8_t)data->data[index];
//                printk("%x,", data->data[index]);
//           }
//           printk("\n");
//            printk("Beacon minor: ");
//           for(i=0; i < BEACON_MINOR_SIZE; i++){
//                index = i + BEACON_MINOR_START;
//                beacon_minor[i] = (uint8_t)data->data[index];
//                printk("%x,", data->data[index]);
//           }
//           printk("\n\n");
//	   return false;
//	}


	return true;
}

static void scan_cb(const bt_addr_le_t *addr, s8_t rssi, u8_t adv_type,
		    struct net_buf_simple *ad)
{
//        printk("scan callback\n");
	char addr_str[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
//        printk("Direct advertising received from %s\n", addr_str);
        bt_data_parse(ad, adv_data_found, addr_str);
}

//BT_SCAN_CB_INIT(scan_cb, scan_filter_match, scan_filter_no_match, scan_connecting_error, NULL);

static void scan_start(void)
{
	int err;
//
	struct bt_le_scan_param scan_param = {
		.type = BT_HCI_LE_SCAN_PASSIVE,
		.filter_dup = BT_HCI_LE_SCAN_FILTER_DUP_DISABLE,
		.interval = 0x3E80, // SCAN EVERY ms
		.window = 0x0030, // FOR THIS MANY ms
	};
//
	struct bt_scan_init_param scan_init = {
//		.connect_if_match = 1,
		.scan_param = &scan_param
//		.conn_param = BT_LE_CONN_PARAM_DEFAULT,
	};
//
	bt_scan_init(&scan_init);
//	bt_scan_cb_register(&scan_cb);
//
////        struct bt_scan_short_name device_name;
////        const char *device_name = "Nordic Discovery Sample";
//
//	err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_BEACON);
//	if (err) {
//		printk("Scanning filters cannot be set\n");
//		return;
//	}
//
//	err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
//	if (err) {
//		printk("Filters cannot be turned on\n");
//	}

	err = bt_le_scan_start(&scan_param, scan_cb);
	if (err) {
		printk("Scanning failed to start, err %d\n", err);
	}

	printk("Scanning...\n");
}

static void ble_ready(int err)
{
	printk("Bluetooth ready\n");

	// bt_conn_cb_register(&conn_callbacks);
	scan_start();
}

void ble_init(void)
{
	int err;

	printk("Initializing Bluetooth..\n");
	err = bt_enable(ble_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}
}
