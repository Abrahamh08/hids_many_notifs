/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <soc.h>
#include <assert.h>

#include <zephyr/settings/settings.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/bluetooth/services/bas.h>
#include <bluetooth/services/hids.h>
#include <zephyr/bluetooth/services/dis.h>
#include <dk_buttons_and_leds.h>

#include <zephyr/usb/class/hid.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

#define DEVICE_NAME     CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define BASE_USB_HID_SPEC_VERSION   0x0101

/* Number of input reports in this application. */
#define INPUT_REPORT_COUNT          1
/* Length of Mouse Input Report containing button data. */
#define INPUT_REP_BUTTONS_LEN       20
/* Index of Mouse Input Report containing button data. */
#define INPUT_REP_BUTTONS_INDEX     0
/* Id of reference to Mouse Input Report containing button data. */
#define INPUT_REP_REF_BUTTONS_ID    1

/* Key used to accept or reject passkey value */
#define KEY_PAIRING_ACCEPT DK_BTN1_MSK
#define KEY_PAIRING_REJECT DK_BTN2_MSK
#define TOGGLE_START DK_BTN3_MSK

volatile bool start = false;

/* HIDS instance. */
BT_HIDS_DEF(hids_obj,
        INPUT_REP_BUTTONS_LEN);

#if CONFIG_BT_DIRECTED_ADVERTISING
/* Bonded address queue. */
K_MSGQ_DEFINE(bonds_queue,
          sizeof(bt_addr_le_t),
          CONFIG_BT_MAX_PAIRED,
          4);
#endif

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE,
              (CONFIG_BT_DEVICE_APPEARANCE >> 0) & 0xff,
              (CONFIG_BT_DEVICE_APPEARANCE >> 8) & 0xff),
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_HIDS_VAL),
                                      BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)),
};

static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static struct conn_mode {
    struct bt_conn *conn;
    bool in_boot_mode;
} conn_mode[CONFIG_BT_HIDS_MAX_CLIENT_COUNT];

static volatile bool is_adv_running;

static struct k_work adv_work;

static struct k_work pairing_work;
struct pairing_data_mitm {
    struct bt_conn *conn;
    unsigned int passkey;
};

K_MSGQ_DEFINE(mitm_queue,
          sizeof(struct pairing_data_mitm),
          CONFIG_BT_HIDS_MAX_CLIENT_COUNT,
          4);

#if CONFIG_BT_DIRECTED_ADVERTISING
static void bond_find(const struct bt_bond_info *info, void *user_data)
{
    int err;

    /* Filter already connected peers. */
    for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
        if (conn_mode[i].conn) {
            const bt_addr_le_t *dst =
                bt_conn_get_dst(conn_mode[i].conn);

            if (!bt_addr_le_cmp(&info->addr, dst)) {
                return;
            }
        }
    }

    err = k_msgq_put(&bonds_queue, (void *) &info->addr, K_NO_WAIT);
    if (err) {
        printk("No space in the queue for the bond.\n");
    }
}
#endif

static void advertising_continue(void)
{
    struct bt_le_adv_param adv_param;

#if CONFIG_BT_DIRECTED_ADVERTISING
    bt_addr_le_t addr;

    if (!k_msgq_get(&bonds_queue, &addr, K_NO_WAIT)) {
        char addr_buf[BT_ADDR_LE_STR_LEN];
        int err;

        if (is_adv_running) {
            err = bt_le_adv_stop();
            if (err) {
                printk("Advertising failed to stop (err %d)\n", err);
                return;
            }
            is_adv_running = false;
        }

        adv_param = *BT_LE_ADV_CONN_DIR(&addr);
        adv_param.options |= BT_LE_ADV_OPT_DIR_ADDR_RPA;

        err = bt_le_adv_start(&adv_param, NULL, 0, NULL, 0);

        if (err) {
            printk("Directed advertising failed to start (err %d)\n", err);
            return;
        }

        bt_addr_le_to_str(&addr, addr_buf, BT_ADDR_LE_STR_LEN);
        printk("Direct advertising to %s started\n", addr_buf);
    } else
#endif
    {
        int err;

        if (is_adv_running) {
            return;
        }

        adv_param = *BT_LE_ADV_CONN;
        adv_param.options |= BT_LE_ADV_OPT_ONE_TIME;
        err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad),
                  sd, ARRAY_SIZE(sd));
        if (err) {
            printk("Advertising failed to start (err %d)\n", err);
            return;
        }

        printk("Regular advertising started\n");
    }

    is_adv_running = true;
}

static void advertising_start(void)
{
#if CONFIG_BT_DIRECTED_ADVERTISING
    k_msgq_purge(&bonds_queue);
    bt_foreach_bond(BT_ID_DEFAULT, bond_find, NULL);
#endif

    k_work_submit(&adv_work);
}

static void advertising_process(struct k_work *work)
{
    advertising_continue();
}

static void pairing_process(struct k_work *work)
{
    int err;
    struct pairing_data_mitm pairing_data;

    char addr[BT_ADDR_LE_STR_LEN];

    err = k_msgq_peek(&mitm_queue, &pairing_data);
    if (err) {
        return;
    }

    bt_addr_le_to_str(bt_conn_get_dst(pairing_data.conn),
              addr, sizeof(addr));

    printk("Passkey for %s: %06u\n", addr, pairing_data.passkey);
    printk("Press Button 1 to confirm, Button 2 to reject.\n");
}

static void insert_conn_object(struct bt_conn *conn)
{
    for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
        if (!conn_mode[i].conn) {
            conn_mode[i].conn = conn;
            conn_mode[i].in_boot_mode = false;

            return;
        }
    }

    printk("Connection object could not be inserted %p\n", conn);
}

static bool is_conn_slot_free(void)
{
    for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
        if (!conn_mode[i].conn) {
            return true;
        }
    }

    return false;
}

static void connected(struct bt_conn *conn, uint8_t err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    is_adv_running = false;

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (err) {
        if (err == BT_HCI_ERR_ADV_TIMEOUT) {
            printk("Direct advertising to %s timed out\n", addr);
            k_work_submit(&adv_work);
        } else {
            printk("Failed to connect to %s (%u)\n", addr, err);
        }
        return;
    }

    printk("Connected %s\n", addr);

    err = bt_hids_connected(&hids_obj, conn);

    if (err) {
        printk("Failed to notify HID service about connection\n");
        return;
    }

    insert_conn_object(conn);

    if (is_conn_slot_free()) {
        advertising_start();
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    int err;
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    printk("Disconnected from %s (reason %u)\n", addr, reason);

    err = bt_hids_disconnected(&hids_obj, conn);

    if (err) {
        printk("Failed to notify HID service about disconnection\n");
    }

    for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
        if (conn_mode[i].conn == conn) {
            conn_mode[i].conn = NULL;
            break;
        }
    }

    advertising_start();
}


#ifdef CONFIG_BT_HIDS_SECURITY_ENABLED
static void security_changed(struct bt_conn *conn, bt_security_t level,
                 enum bt_security_err err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (!err) {
        printk("Security changed: %s level %u\n", addr, level);
    } else {
        printk("Security failed: %s level %u err %d\n", addr, level,
            err);
    }
}
#endif


BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
#ifdef CONFIG_BT_HIDS_SECURITY_ENABLED
    .security_changed = security_changed,
#endif
};


static void hids_pm_evt_handler(enum bt_hids_pm_evt evt,
                struct bt_conn *conn)
{
    char addr[BT_ADDR_LE_STR_LEN];
    size_t i;

    for (i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
        if (conn_mode[i].conn == conn) {
            break;
        }
    }

    if (i >= CONFIG_BT_HIDS_MAX_CLIENT_COUNT) {
        return;
    }

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    switch (evt) {
    case BT_HIDS_PM_EVT_BOOT_MODE_ENTERED:
        printk("Boot mode entered %s\n", addr);
        conn_mode[i].in_boot_mode = true;
        break;

    case BT_HIDS_PM_EVT_REPORT_MODE_ENTERED:
        printk("Report mode entered %s\n", addr);
        conn_mode[i].in_boot_mode = false;
        break;

    default:
        break;
    }
}

//*****************************************************************************
//
//! This is a macro to assist adding vendor-specific Usage entries in HID
//! report descriptors.
//!
//! \param value is the vendor-specific Usage value in the range 0xFF00 to
//! 0xFFFF.
//!
//! This macro takes a value and prepares it to be placed as a Usage entry into
//! a HID report structure.  These are defined by the USB HID specification.
//!
//! \return Not a function.
//
//*****************************************************************************
#define UsageVendor(value)   0x0A, ((value) & 0xFF), (((value) >> 8) & 0xFF)

static void hid_init(void)
{
    int err;
    struct bt_hids_init_param hids_init_param = { 0 };
    struct bt_hids_inp_rep *hids_inp_rep;

    static const uint8_t report_map[] = {
        HID_USAGE_PAGE(HID_USAGE_GEN_DESKTOP),

        HID_USAGE(HID_USAGE_GEN_DESKTOP_GAMEPAD),
        HID_COLLECTION(HID_COLLECTION_APPLICATION),
            HID_REPORT_ID(0x01),
            UsageVendor(0xFF01),
            HID_LOGICAL_MIN8(0x00),
            HID_LOGICAL_MAX8(0xff),
            HID_REPORT_SIZE(8),
            HID_REPORT_COUNT(20),
            HID_INPUT(0x02),
        HID_END_COLLECTION,
    };

    hids_init_param.rep_map.data = report_map;
    hids_init_param.rep_map.size = sizeof(report_map);

    hids_init_param.info.bcd_hid = BASE_USB_HID_SPEC_VERSION;
    hids_init_param.info.b_country_code = 0x00;
    hids_init_param.info.flags = (BT_HIDS_REMOTE_WAKE |
                      BT_HIDS_NORMALLY_CONNECTABLE);

    hids_inp_rep = &hids_init_param.inp_rep_group_init.reports[0];
    hids_inp_rep->size = INPUT_REP_BUTTONS_LEN;
    hids_inp_rep->id = INPUT_REP_REF_BUTTONS_ID;
    hids_init_param.inp_rep_group_init.cnt++;

    hids_init_param.is_mouse = true;
    hids_init_param.pm_evt_handler = hids_pm_evt_handler;

    err = bt_hids_init(&hids_obj, &hids_init_param);
    __ASSERT(err == 0, "HIDS initialization failed\n");
}

static void send_dummy_mouse_buttons_report(uint8_t count)
{
    for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
        if (!conn_mode[i].conn) {
            continue;
        }

        uint8_t buff[INPUT_REP_BUTTONS_LEN] = {0};
        memset(buff, count, sizeof(buff));

        LOG_INF("Sending dummy mouse buttons report");
        int err = bt_hids_inp_rep_send(&hids_obj, conn_mode[i].conn,
                        INPUT_REP_BUTTONS_INDEX,
                        buff, sizeof(buff), NULL);
        LOG_INF("Dummy mouse buttons report sent with result %d", err);
    }
}

#if defined(CONFIG_BT_HIDS_SECURITY_ENABLED)
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    printk("Passkey for %s: %06u\n", addr, passkey);
}


static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
    int err;

    struct pairing_data_mitm pairing_data;

    pairing_data.conn    = bt_conn_ref(conn);
    pairing_data.passkey = passkey;

    err = k_msgq_put(&mitm_queue, &pairing_data, K_NO_WAIT);
    if (err) {
        printk("Pairing queue is full. Purge previous data.\n");
    }

    /* In the case of multiple pairing requests, trigger
     * pairing confirmation which needed user interaction only
     * once to avoid display information about all devices at
     * the same time. Passkey confirmation for next devices will
     * be proccess from queue after handling the earlier ones.
     */
    if (k_msgq_num_used_get(&mitm_queue) == 1) {
        k_work_submit(&pairing_work);
    }
}


static void auth_cancel(struct bt_conn *conn)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    printk("Pairing cancelled: %s\n", addr);
}


static void pairing_complete(struct bt_conn *conn, bool bonded)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    printk("Pairing completed: %s, bonded: %d\n", addr, bonded);
}


static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
    char addr[BT_ADDR_LE_STR_LEN];
    struct pairing_data_mitm pairing_data;

    if (k_msgq_peek(&mitm_queue, &pairing_data) != 0) {
        return;
    }

    if (pairing_data.conn == conn) {
        bt_conn_unref(pairing_data.conn);
        k_msgq_get(&mitm_queue, &pairing_data, K_NO_WAIT);
    }

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    printk("Pairing failed conn: %s, reason %d\n", addr, reason);
}


static struct bt_conn_auth_cb conn_auth_callbacks = {
    .passkey_display = auth_passkey_display,
    .passkey_confirm = auth_passkey_confirm,
    .cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
    .pairing_complete = pairing_complete,
    .pairing_failed = pairing_failed
};
#else
static struct bt_conn_auth_cb conn_auth_callbacks;
static struct bt_conn_auth_info_cb conn_auth_info_callbacks;
#endif /* defined(CONFIG_BT_HIDS_SECURITY_ENABLED) */

static void num_comp_reply(bool accept)
{
    struct pairing_data_mitm pairing_data;
    struct bt_conn *conn;

    if (k_msgq_get(&mitm_queue, &pairing_data, K_NO_WAIT) != 0) {
        return;
    }

    conn = pairing_data.conn;

    if (accept) {
        bt_conn_auth_passkey_confirm(conn);
        printk("Numeric Match, conn %p\n", conn);
    } else {
        bt_conn_auth_cancel(conn);
        printk("Numeric Reject, conn %p\n", conn);
    }

    bt_conn_unref(pairing_data.conn);

    if (k_msgq_num_used_get(&mitm_queue)) {
        k_work_submit(&pairing_work);
    }
}

void button_changed(uint32_t button_state, uint32_t has_changed)
{
    uint32_t buttons = button_state & has_changed;

    if (IS_ENABLED(CONFIG_BT_HIDS_SECURITY_ENABLED)) {
        if (k_msgq_num_used_get(&mitm_queue)) {
            if (buttons & KEY_PAIRING_ACCEPT) {
                num_comp_reply(true);

                return;
            }

            if (buttons & KEY_PAIRING_REJECT) {
                num_comp_reply(false);

                return;
            }
        }
    }

    if (buttons & TOGGLE_START) {
        start = true;
    }
}

void configure_buttons(void)
{
    int err;

    err = dk_buttons_init(button_changed);
    if (err) {
        printk("Cannot init buttons (err: %d)\n", err);
    }
}

int main(void)
{
    for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
        conn_mode[i].conn = NULL;
    }

    int err;

    printk("Starting Bluetooth Peripheral HIDS mouse example\n");

    if (IS_ENABLED(CONFIG_BT_HIDS_SECURITY_ENABLED)) {
        err = bt_conn_auth_cb_register(&conn_auth_callbacks);
        if (err) {
            printk("Failed to register authorization callbacks.\n");
            return 0;
        }

        err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
        if (err) {
            printk("Failed to register authorization info callbacks.\n");
            return 0;
        }
    }

    /* DIS initialized at system boot with SYS_INIT macro. */
    hid_init();

    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return 0;
    }

    printk("Bluetooth initialized\n");

    k_work_init(&adv_work, advertising_process);
    if (IS_ENABLED(CONFIG_BT_HIDS_SECURITY_ENABLED)) {
        k_work_init(&pairing_work, pairing_process);
    }

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }

    advertising_start();

    configure_buttons();

    while (!k_msgq_num_used_get(&mitm_queue)) {
        k_sleep(K_SECONDS(1));
    }

    num_comp_reply(true);

    uint8_t count = 0;

    while (1) {
        k_sleep(K_MSEC(8));
        // if (start) {
        send_dummy_mouse_buttons_report(count);
        count++;
        if (count > 0xff) {
            count = 0;
        }
        // }
    }
}
