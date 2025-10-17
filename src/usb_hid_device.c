/* src/usb_hid_device.c
 *
 * USB HID Device implementation for Zephyr using composite HID
 * Supports both mouse and keyboard as separate interfaces
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/logging/log.h>
#include <errno.h>

LOG_MODULE_REGISTER(usb_hid_dev, LOG_LEVEL_INF);

#define HID_IOF_DATA                0x00  /* Data (0) */
#define HID_IOF_CONSTANT            0x01  /* Constant (1) */
#define HID_IOF_ARRAY               0x00  /* Array (0) */
#define HID_IOF_VARIABLE            0x02  /* Variable (1) */
#define HID_IOF_ABSOLUTE            0x00  /* Absolute (0) */
#define HID_IOF_RELATIVE            0x04  /* Relative (1) */
#define HID_IOF_WRAP                0x08  /* Wrap */
#define HID_IOF_NONLINEAR           0x10  /* Non-linear */
#define HID_IOF_NO_PREFERRED        0x20  /* No Preferred State */
#define HID_IOF_NULL_STATE          0x40  /* Null State */
#define HID_IOF_VOLATILE            0x80  /* Volatile */


/* HID Report Descriptors */
/* Note: Zephyr's HID helper macros for 16-bit values require two byte arguments:
 * HID_LOGICAL_MIN16(lsb, msb), HID_LOGICAL_MAX16(lsb, msb)
 */
static const uint8_t hid_mouse_report_desc[] = {
    HID_USAGE_PAGE(HID_USAGE_GEN_DESKTOP),
    HID_USAGE(HID_USAGE_GEN_DESKTOP_MOUSE),
    HID_COLLECTION(HID_COLLECTION_APPLICATION),
        HID_USAGE(HID_USAGE_GEN_DESKTOP_POINTER),
        HID_COLLECTION(HID_COLLECTION_PHYSICAL),
            /* Buttons (16 bits) */
            HID_USAGE_PAGE(HID_USAGE_GEN_BUTTON),
            HID_USAGE_MIN8(1),
            HID_USAGE_MAX8(16),
            HID_LOGICAL_MIN8(0),
            HID_LOGICAL_MAX8(1),
            HID_REPORT_COUNT(16),
            HID_REPORT_SIZE(1),
            HID_INPUT(HID_IOF_DATA | HID_IOF_VARIABLE | HID_IOF_ABSOLUTE),
            /* X, Y (16-bit signed) -- put LSB then MSB for the 16-bit macros */
            HID_USAGE_PAGE(HID_USAGE_GEN_DESKTOP),
            HID_USAGE(HID_USAGE_GEN_DESKTOP_X),
            HID_USAGE(HID_USAGE_GEN_DESKTOP_Y),
            HID_LOGICAL_MIN16(0x01, 0x80),   /* -32767 -> 0x8001 -> LSB=0x01 MSB=0x80 */
            HID_LOGICAL_MAX16(0xFF, 0x7F),   /*  32767 -> 0x7FFF -> LSB=0xFF MSB=0x7F */
            HID_REPORT_COUNT(2),
            HID_REPORT_SIZE(16),
            HID_INPUT(HID_IOF_DATA | HID_IOF_VARIABLE | HID_IOF_RELATIVE),
            /* Wheel (8-bit signed) */
            HID_USAGE(HID_USAGE_GEN_DESKTOP_WHEEL),
            HID_LOGICAL_MIN8(-127),
            HID_LOGICAL_MAX8(127),
            HID_REPORT_COUNT(1),
            HID_REPORT_SIZE(8),
            HID_INPUT(HID_IOF_DATA | HID_IOF_VARIABLE | HID_IOF_RELATIVE),
            /* Padding to 8 bytes total */
            HID_REPORT_COUNT(1),
            HID_REPORT_SIZE(8),
            HID_INPUT(HID_IOF_CONSTANT),
        HID_END_COLLECTION,
    HID_END_COLLECTION,
};

static const uint8_t hid_keyboard_report_desc[] = {
    HID_USAGE_PAGE(HID_USAGE_GEN_DESKTOP),
    HID_USAGE(HID_USAGE_GEN_DESKTOP_KEYBOARD),
    HID_COLLECTION(HID_COLLECTION_APPLICATION),
        /* Modifier keys */
        HID_USAGE_PAGE(HID_USAGE_GEN_DESKTOP_KEYPAD),
        HID_USAGE_MIN8(0xE0),
        HID_USAGE_MAX8(0xE7),
        HID_LOGICAL_MIN8(0),
        HID_LOGICAL_MAX8(1),
        HID_REPORT_COUNT(8),
        HID_REPORT_SIZE(1),
        HID_INPUT(HID_IOF_DATA | HID_IOF_VARIABLE | HID_IOF_ABSOLUTE),
        /* Reserved byte */
        HID_REPORT_COUNT(1),
        HID_REPORT_SIZE(8),
        HID_INPUT(HID_IOF_CONSTANT),
        /* LED output report */
        HID_REPORT_COUNT(5),
        HID_REPORT_SIZE(1),
        HID_USAGE_PAGE(HID_USAGE_GEN_LEDS),
        HID_USAGE_MIN8(1),
        HID_USAGE_MAX8(5),
        HID_OUTPUT(HID_IOF_DATA | HID_IOF_VARIABLE | HID_IOF_ABSOLUTE),
        HID_REPORT_COUNT(1),
        HID_REPORT_SIZE(3),
        HID_OUTPUT(HID_IOF_CONSTANT),
        /* Key arrays (6 bytes) */
        HID_REPORT_COUNT(6),
        HID_REPORT_SIZE(8),
        HID_LOGICAL_MIN8(0),
        HID_LOGICAL_MAX8(101),
        HID_USAGE_PAGE(HID_USAGE_GEN_DESKTOP_KEYPAD),
        HID_USAGE_MIN8(0),
        HID_USAGE_MAX8(101),
        HID_INPUT(HID_IOF_DATA | HID_IOF_ARRAY),
    HID_END_COLLECTION,
};

/* Device handles */
static const struct device *hid_mouse_dev;
static const struct device *hid_keyboard_dev;

/* Initialization state */
static bool usb_hid_initialized = false;

/* Device callbacks (required but can be empty for basic operation) */
static void mouse_int_in_ready_cb(const struct device *dev)
{
    ARG_UNUSED(dev);
}

static void keyboard_int_in_ready_cb(const struct device *dev)
{
    ARG_UNUSED(dev);
}

static const struct hid_ops mouse_ops = {
    .int_in_ready = mouse_int_in_ready_cb,
};

static const struct hid_ops keyboard_ops = {
    .int_in_ready = keyboard_int_in_ready_cb,
};

/* Initialize USB HID devices */
int usb_hid_device_init(void)
{
    int ret;

    if (usb_hid_initialized) {
        return 0;
    }

    /* Get device references
     *
     * NOTE: the devicetree / Kconfig in your project must expose the HID device(s)
     * with these names. If device_get_binding("HID_0")/("HID_1") fails, replace
     * those strings with the actual labels exposed by your board DTS or check
     * `dtdev` output or `device_get_binding()` logs.
     */
    hid_mouse_dev = device_get_binding("HID_0");
    if (!hid_mouse_dev) {
        LOG_ERR("Cannot get USB HID Mouse Device (HID_0)");
        return -ENODEV;
    }

    hid_keyboard_dev = device_get_binding("HID_1");
    if (!hid_keyboard_dev) {
        LOG_ERR("Cannot get USB HID Keyboard Device (HID_1)");
        return -ENODEV;
    }

    /* Register HID devices
     *
     * These helpers (usb_hid_register_device / usb_hid_init / hid_int_ep_write)
     * are thin wrappers in Zephyr; if your Zephyr version does not provide them
     * exactly, you'll get link errors. If that happens, paste the build error
     * and I will adapt to the Zephyr API available in your tree.
     */
    usb_hid_register_device(hid_mouse_dev,
                           hid_mouse_report_desc,
                           sizeof(hid_mouse_report_desc),
                           &mouse_ops);

    usb_hid_register_device(hid_keyboard_dev,
                           hid_keyboard_report_desc,
                           sizeof(hid_keyboard_report_desc),
                           &keyboard_ops);

    /* Initialize USB device stack */
    ret = usb_hid_init(hid_mouse_dev);
    if (ret) {
        LOG_ERR("Failed to init USB HID Mouse: %d", ret);
        return ret;
    }

    ret = usb_hid_init(hid_keyboard_dev);
    if (ret) {
        LOG_ERR("Failed to init USB HID Keyboard: %d", ret);
        return ret;
    }

    /* Enable USB */
    ret = usb_enable(NULL);
    if (ret) {
        LOG_ERR("Failed to enable USB: %d", ret);
        return ret;
    }

    usb_hid_initialized = true;
    LOG_INF("USB HID device initialized successfully");
    return 0;
}

/* Send HID report - called by forwarder */
int usb_hid_send_report(uint8_t interface_idx, uint8_t *report, uint32_t report_len)
{
    const struct device *dev;
    int ret;

    if (!usb_hid_initialized) {
        LOG_ERR("USB HID not initialized");
        return -ENODEV;
    }

    if (!report || report_len == 0) {
        return -EINVAL;
    }

    /* Select device based on interface index:
     * 0 = mouse (typically first CH375)
     * 1 = keyboard (typically second CH375)
     */
    if (interface_idx == 0) {
        dev = hid_mouse_dev;
        if (report_len != 8) {
            LOG_WRN("Mouse report length mismatch: expected 8, got %u", report_len);
        }
    } else if (interface_idx == 1) {
        dev = hid_keyboard_dev;
        if (report_len != 8) {
            LOG_WRN("Keyboard report length mismatch: expected 8, got %u", report_len);
        }
    } else {
        LOG_ERR("Invalid interface index: %u", interface_idx);
        return -EINVAL;
    }

    /* Send the report
     *
     * hid_int_ep_write() is the Zephyr helper that writes to the interrupt IN endpoint.
     * If your SDK's header declares a different prototype you will see a linker error.
     */
    ret = hid_int_ep_write(dev, report, report_len, NULL);
    if (ret) {
        LOG_DBG("Failed to send HID report (iface %u): %d", interface_idx, ret);
        return ret;
    }

    return 0;
}
