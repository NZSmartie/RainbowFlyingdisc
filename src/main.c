/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// Include generated headerfile for auto completion
#include <autoconf.h>

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <misc/printk.h>
#include <misc/byteorder.h>
#include <zephyr.h>

#include <settings/settings.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#include <board.h>
#include <device.h>
#include <led_strip.h>
#include <gpio.h>
#include <sensor.h>

#include "ble.h"
#include "events.h"
#include "pov.h"

K_MSGQ_DEFINE(events, sizeof(event_t), 10, 4);

static void bt_ready(int err)
{
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized\n");

    // TODO: Support customising the name per owner?
    // bt_set_name("Rainbow Flying Disc");

    // Initialise and register our Rainbow Flying Disc GATT service
    rainbow_flying_disc_init(&events);

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }
}

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    printk("Passkey for %s: %06u\n", addr, passkey);
}

static void auth_cancel(struct bt_conn *conn)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    printk("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb auth_cb_display = {
    .passkey_display = auth_passkey_display,
    .passkey_entry = NULL,
    .cancel = auth_cancel,
};

void print_mpu_data(struct device *mpu6050) {
    struct sensor_value accel_xyz[3], gyro_xyz[3];

    if(sensor_sample_fetch(mpu6050)) {
        return;
    }

    sensor_channel_get(mpu6050, SENSOR_CHAN_ACCEL_XYZ, accel_xyz);
    sensor_channel_get(mpu6050, SENSOR_CHAN_GYRO_XYZ, gyro_xyz);

    printk("%d.%d\t%d.%d\t%d.%d\t%d\t%d\t%d\n",
        accel_xyz[0].val1, accel_xyz[0].val2,
        accel_xyz[1].val1, accel_xyz[1].val2,
        accel_xyz[2].val1, accel_xyz[2].val2,
        gyro_xyz[0].val1,
        gyro_xyz[1].val1,
        gyro_xyz[2].val1);
}

void main(void)
{
    int err, count = 0;

    struct device *leds = device_get_binding(LED0_GPIO_PORT),
                  *sw0 = device_get_binding(SW0_GPIO_NAME),
                  *sw1 = device_get_binding(SW1_GPIO_NAME);
    // struct device *mpu6050 = device_get_binding(CONFIG_MPU6050_NAME);

    gpio_pin_configure(leds, LED0_GPIO_PIN, GPIO_DIR_OUT);
    gpio_pin_configure(sw0, SW0_GPIO_PIN, GPIO_DIR_IN | SW0_GPIO_PIN_PUD);
    gpio_pin_configure(sw1, SW1_GPIO_PIN, GPIO_DIR_IN | SW1_GPIO_PIN_PUD);


    event_t event_data = {0};

    pov_init();

    err = bt_enable(bt_ready);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    // if (mpu6050 == NULL) {
    //     printk("Failed to get IMU driver %s\n", CONFIG_MPU6050_NAME);
    // }

    // TODO: Support a whitelisted mode, only connecting to known devices
    // bt_conn_auth_cb_register(&auth_cb_display);

    while (1) {
        k_sleep(100);

        if (k_msgq_get(&events, &event_data, K_NO_WAIT) == 0)
        {
            switch(event_data.event_type) {
                case kEventMessage:
                    printk("new message event! read %s (%d)\n", event_data.message, event_data.size);
                    pov_set_message(event_data.message, event_data.size);
                    break;
                default:
                    printk("Unhandled event (%d)\n", event_data.event_type);
                    break;
            }
        }

        int sw0_state;
        if(gpio_pin_read(sw0, SW0_GPIO_PIN, &sw0_state) == 0 && sw0_state == 0)
            printk("SW0 is pressed\n");

        int sw1_state;
        if(gpio_pin_read(sw1, SW1_GPIO_PIN, &sw1_state) == 0 && sw1_state == 0)
        {
            printk("SW1 is pressed\n");
            rainbow_flying_disc_discover();
        }

        // if (mpu6050 != NULL)
        //     print_mpu_data(mpu6050); // Test the MPU driver

        gpio_pin_write(leds, LED0_GPIO_PIN, count++ & 0x01);
    }
}
