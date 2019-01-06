#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#include "gatt.h"

#define SCAN_TIMEOUT K_SECONDS(2)

static enum {
	BLE_DISCONNECTED,
	BLE_SCAN_START,
	BLE_SCAN,
	BLE_CONNECT_CREATE,
	BLE_CONNECT_CANCEL,
	BLE_ADV_START,
	BLE_ADVERTISING,
	BLE_CONNECTED,
} ble_state;

static struct bt_uuid_128 rfd_service_uuid = BT_UUID_INIT_128(UUID_RAINBOW_FLYING_DISC_SERVICE);
static struct bt_uuid_128 rfd_message_uuid = BT_UUID_INIT_128(UUID_RAINBOW_FLYING_DISC_CHARACTERISTIC_MESSAGE);

static struct bt_gatt_discover_params discov_param;
static struct bt_gatt_read_params read_params;

static u8_t rfd_message[] = { 'R', 'a', 'i', 'n', 'b', 'o', 'w', ' ', 'F', 'l', 'y', 'i', 'n', 'g', ' ', 'D', 'i', 's', 'c'};

static struct k_delayed_work ble_work;
static struct bt_conn *default_conn;
static bool connect_canceled = false;
static u16_t remote_handle;
static bool remote_ready;

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

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, UUID_RAINBOW_FLYING_DISC_SERVICE),
};

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

    printk("Message is now %s\n", rfd_message);

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

static void create_conn(const bt_addr_le_t *addr)
{
	if (default_conn) {
		return;
	}

    char src[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(addr, src, sizeof(src));
	printk("Found [%s], starting connection...\n", src);

	default_conn = bt_conn_create_le(addr, BT_LE_CONN_PARAM_DEFAULT);
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

	if (ble_state == BLE_ADVERTISING) {
		bt_le_adv_stop();
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
	if (type != BT_LE_ADV_IND) {
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

		type = net_buf_simple_pull_u8(ad);
		if (type == BT_DATA_UUID128_ALL &&
            uuid_match(ad->data, len - 1, &rfd_service_uuid.uuid)) {
			bt_le_scan_stop();
			create_conn(addr);
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

    char message[80];
    memset(message, 0, sizeof(message));
    memcpy(message, data, length);

    printk("read_message_cb() read %s (%d)\n", (const char*)message, length);
    for(int i = 0; i < length; i++)
    {
        printk("%02X", *((char*)data + i) & 0xFF);
    }
    printk("\n");

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
            ble_state = BLE_ADV_START;
            k_delayed_work_submit(&ble_work, K_NO_WAIT);
            break;
        case BLE_SCAN:
            printk("No devices found during scan\n");
            bt_le_scan_stop();
            ble_state = BLE_ADV_START;
            k_delayed_work_submit(&ble_work, K_NO_WAIT);
            break;
        case BLE_ADV_START:
            err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad),
                        NULL, 0);
            if (err) {
                printk("Advertising failed to start (err %d)\n", err);
                return;
            }

            printk("Advertising successfully started\n");
            ble_state = BLE_ADVERTISING;
            k_delayed_work_submit(&ble_work, K_SECONDS(10));
            break;
        case BLE_ADVERTISING:
            printk("Timed out advertising\n");
            bt_le_adv_stop();
            ble_state = BLE_SCAN_START;
            k_delayed_work_submit(&ble_work, K_NO_WAIT);
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

void rainbow_flying_disc_init()
{
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
	case BLE_ADV_START:
		ble_state = BLE_DISCONNECTED;
		break;
	case BLE_ADVERTISING:
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
