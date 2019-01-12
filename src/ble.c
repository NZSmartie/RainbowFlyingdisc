#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#include "ble.h"
#include "events.h"

#define SCAN_TIMEOUT K_SECONDS(2)
#define MAX_FOUND_DEVICES 10

static enum {
	BLE_DISCONNECTED,
	BLE_SCAN_START,
	BLE_SCAN,
	BLE_CONNECT_CREATE,
	BLE_CONNECT_CANCEL,
	BLE_CONNECTED,
} ble_state;

typedef struct found_device {
    s8_t rssi;
    bt_addr_le_t addr;
} found_device_t;

static struct bt_uuid_128 rfd_service_uuid = BT_UUID_INIT_128(UUID_RAINBOW_FLYING_DISC_SERVICE);
static struct bt_uuid_128 rfd_message_uuid = BT_UUID_INIT_128(UUID_RAINBOW_FLYING_DISC_CHARACTERISTIC_MESSAGE);

static struct bt_gatt_discover_params discov_param;
static struct bt_gatt_read_params read_params;

static u8_t rfd_message[] = { 'R', 'a', 'i', 'n', 'b', 'o', 'w', ' ', 'F', 'l', 'y', 'i', 'n', 'g', ' ', 'D', 'i', 's', 'c'};
static struct {
    found_device_t devices[MAX_FOUND_DEVICES];
    u8_t count;
} found_devices = {0};

static struct k_delayed_work ble_work;
static struct bt_conn *default_conn;
static bool connect_canceled = false;
static u16_t remote_handle;
static bool remote_ready;
static struct k_msgq *_events = NULL;

ssize_t rfd_read_message(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, u16_t len, u16_t offset);
ssize_t rfd_write_message(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, u16_t len, u16_t offset, u8_t flags);

static void connected(struct bt_conn *conn, u8_t err);
static void disconnected(struct bt_conn *conn, u8_t reason);

// Vendor Primary Service Declaration
static struct bt_gatt_attr vnd_attrs[] = {
    // Vendor Primary Service Declaration
    BT_GATT_PRIMARY_SERVICE(&rfd_service_uuid),
    BT_GATT_CHARACTERISTIC(&rfd_message_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
        BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT,
        rfd_read_message,
        rfd_write_message,
        rfd_message),
};

static struct bt_gatt_service vnd_svc = BT_GATT_SERVICE(vnd_attrs);

struct bt_le_scan_param scan_param = {
    .type       = BT_HCI_LE_SCAN_PASSIVE,
    .filter_dup = BT_HCI_LE_SCAN_FILTER_DUP_DISABLE,
    .interval   = 0x0010,
    .window     = 0x0010,
};

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

ssize_t rfd_read_message(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, u16_t len, u16_t offset)
{
    const char *value = attr->user_data;

    return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
                 strlen(value));
}

ssize_t rfd_write_message(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, u16_t len, u16_t offset, u8_t flags)
{
    u8_t *value = attr->user_data;

    if (offset + len > sizeof(rfd_message)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    memcpy(value + offset, buf, len);

    return len;
}

static bool uuid_match(const u8_t *data, u8_t len, const struct bt_uuid* uuid)
{
    // Loop through each 128-bit UUID and return true on the first match
	while (len >= 16) {
		if (!memcmp(data, BT_UUID_128(uuid)->val, 16)) {
			return true;
		}

		len -= 16;
		data += 16;
	}

	return false;
}

static void create_conn()
{
    if (default_conn || found_devices.count == 0) {
		return;
	}

    // Start with the first device found
    found_device_t *device = &found_devices.devices[0];

    // select the deivce with the highest rssi;
    for(int i = 1; i < found_devices.count; i++)
    {
        if (found_devices.devices[i].rssi > device->rssi)
            device = &found_devices.devices[i];
    }

    char src[BT_ADDR_LE_STR_LEN];
    bt_addr_to_str(&device->addr.a, src, sizeof(src));
    printk("Connecting to [%s]...\n", src);

    default_conn = bt_conn_create_le(&device->addr, BT_LE_CONN_PARAM_DEFAULT);
	if (!default_conn) {
		printk("Failed to initiate connection");
		return;
	}

	ble_state = BLE_CONNECT_CREATE;
	k_delayed_work_submit(&ble_work, SCAN_TIMEOUT);
}

static void connected(struct bt_conn *conn, u8_t err)
{
	struct bt_conn_info info;

	if (err) {
		printk("Connection failed (err %u)\n", err);
		return;
	}

	if (!default_conn) {
		default_conn = bt_conn_ref(conn);
	}

	bt_conn_get_info(conn, &info);
	remote_ready = false;
	remote_handle = 0;

	printk("Connected\n");
	ble_state = BLE_CONNECTED;

	k_delayed_work_submit(&ble_work, K_NO_WAIT);
}

static void disconnected(struct bt_conn *conn, u8_t reason)
{
	printk("Disconnected (reason %u)\n", reason);

	if (default_conn) {
		bt_conn_unref(default_conn);
		default_conn = NULL;
	}

	remote_handle = 0;

	if (ble_state == BLE_CONNECTED) {
		ble_state = BLE_DISCONNECTED;
	}
}

static void device_found(const bt_addr_le_t *addr, s8_t rssi, u8_t type,
			 struct net_buf_simple *ad)
{
	if (type != BT_LE_ADV_IND || found_devices.count >= MAX_FOUND_DEVICES) {
		return;
	}


	while (ad->len > 1) {
		u8_t len = net_buf_simple_pull_u8(ad);
		u8_t type;

		/* Check for early termination */
		if (len == 0) {
			return;
		}

		if (len > ad->len) {
			printk("AD malformed\n");
			return;
		}

		char addr_str[BT_ADDR_LE_STR_LEN];
		bt_addr_to_str(&addr->a, addr_str, sizeof(addr_str));

		type = net_buf_simple_pull_u8(ad);
		if (type == BT_DATA_UUID128_ALL && uuid_match(ad->data, len - 1, &rfd_service_uuid.uuid)) {
			printk("Found device [%s] with RSSI: %ddB\n", addr_str, rssi);

			found_devices.devices[found_devices.count].rssi = rssi;
			memcpy(&found_devices.devices[found_devices.count].addr, addr, sizeof(bt_addr_le_t));
			found_devices.count++;

			return;
		}

		net_buf_simple_pull(ad, len - 1);
	}
}

static u8_t read_message_cb(struct bt_conn *conn, u8_t err, struct bt_gatt_read_params *params, const void *data, u16_t length)
{
    if(data == NULL)
        return BT_GATT_ITER_STOP;

    if(err)
    {
        printk("read_message_cb() read error (%d)", err);
        return BT_GATT_ITER_STOP;
    }

    // queue the received message
    if (event_message(_events, (const char*)data, length, K_NO_WAIT) != 0)
        printk("Could not queue message");

    // Disconnect as we're done.
    // TODO: Where is the best place to disconnect after all read/write operations are done?
    bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);

    return BT_GATT_ITER_STOP;
}

static u8_t discover_func(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  struct bt_gatt_discover_params *param)
{
	int err;

	if (!attr) {
		printk("Discover complete\n");
		memset(&discov_param, 0, sizeof(discov_param));
		return BT_GATT_ITER_STOP;
	}

	printk("Attribute handle %u\n", attr->handle);

	if (param->uuid == &rfd_service_uuid.uuid) {
		printk("Rainbow Flying Disc service discovered\n");
		discov_param.uuid = &rfd_message_uuid.uuid;
		discov_param.start_handle = attr->handle + 1;
		discov_param.type = BT_GATT_DISCOVER_CHARACTERISTIC;

		err = bt_gatt_discover(conn, &discov_param);
		if (err) {
			printk("Char Discovery failed (err %d)\n", err);
		}
	} else if (param->uuid == &rfd_message_uuid.uuid) {
		printk("Message characteristic discovered\n");

		remote_handle = attr->handle + 1;

	} else {
		printk("Uhhh, unknown uuid in dovery callback %s", bt_uuid_str(param->uuid));
	}

	if (remote_handle) {
        // TODO: Read message here?

        read_params.func = read_message_cb;
        read_params.single.handle = remote_handle;
        read_params.single.offset = 0;
        read_params.handle_count = 1;
        bt_gatt_read(conn, &read_params);
	}

	return BT_GATT_ITER_STOP;
}

static void ble_timeout(struct k_work *work)
{
    int err;

    if (connect_canceled) {
		rainbow_flying_disc_cancel_connect();
		return;
	}

	switch (ble_state) {
	    case BLE_DISCONNECTED:
		    break;
	    case BLE_SCAN_START:
            // Reset found devices
            found_devices.count = 0;

            err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
            if (err) {
                printk("Scanning failed to start (err %d)\n", err);
            }

            printk("Started scanning for devices\n");
            ble_state = BLE_SCAN;
            k_delayed_work_submit(&ble_work, SCAN_TIMEOUT);
            break;
        case BLE_CONNECT_CREATE:
            printk("Connection attempt timed out\n");
            bt_conn_disconnect(default_conn,
                    BT_HCI_ERR_REMOTE_USER_TERM_CONN);
            ble_state = BLE_SCAN_START;
            k_delayed_work_submit(&ble_work, K_NO_WAIT);
            break;
        case BLE_SCAN:
            bt_le_scan_stop();

            if (found_devices.count == 0) {
                printk("No devices found during scan\n");

                ble_state = BLE_SCAN_START;
                k_delayed_work_submit(&ble_work, K_NO_WAIT);
            } else {
                // Try connecting to the device with the highest RSSI
                create_conn();
            }

            break;
        case BLE_CONNECTED:
            discov_param.uuid = &rfd_service_uuid.uuid;
            discov_param.func = discover_func;
            discov_param.start_handle = 0x0001;
            discov_param.end_handle = 0xffff;
            discov_param.type = BT_GATT_DISCOVER_PRIMARY;

            err = bt_gatt_discover(default_conn, &discov_param);
            if (err) {
                printk("Discover failed (err %d)\n", err);
                return;
            }
            break;
        case BLE_CONNECT_CANCEL:
            break;
	}
}

void rainbow_flying_disc_init(struct k_msgq *events)
{
    _events = events;
    k_delayed_work_init(&ble_work, ble_timeout);

    bt_conn_cb_register(&conn_callbacks);

	bt_gatt_service_register(&vnd_svc);
}

void rainbow_flying_disc_discover()
{
    if (ble_state != BLE_DISCONNECTED) {
		printk("Not ready to connect\n");
		return;
	}

	ble_state = BLE_SCAN_START;
	k_delayed_work_submit(&ble_work, K_NO_WAIT);
}

void rainbow_flying_disc_cancel_connect(void)
{
	printk("rainbow_flying_disc_cancel_connect()\n");

	k_delayed_work_cancel(&ble_work);

	switch (ble_state) {
	case BLE_DISCONNECTED:
		break;
	case BLE_SCAN_START:
		ble_state = BLE_DISCONNECTED;
		break;
	case BLE_SCAN:
		connect_canceled = true;
		k_delayed_work_submit(&ble_work, K_NO_WAIT);
		break;
	case BLE_CONNECT_CREATE:
		ble_state = BLE_CONNECT_CANCEL;
		/* Intentional fall-through */
	case BLE_CONNECTED:
		connect_canceled = true;
		k_delayed_work_submit(&ble_work, K_NO_WAIT);
		break;
	case BLE_CONNECT_CANCEL:
		break;
	}
}
