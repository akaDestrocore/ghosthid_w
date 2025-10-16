/*
 * ch375_zephyr_adapter.c
 *
 * Zephyr adapter implementing ch375_interface I/O (write_cmd, write_data, read_data, query_int)
 * - Uses STM32 register access to enable 9-bit mode on USART peripheral when available.
 * - Uses Zephyr device_get_binding() for device lookup and Zephyr GPIO API for INT handling.
 *
 * Notes:
 *  - This file intentionally re-applies the 9-bit configuration before each transaction
 *    to avoid race/ordering issues where other parts of the system (Zephyr UART driver)
 *    may reconfigure the peripheral after we set it.
 *  - Keep the debug/logging enabled while we debug. You can reduce CH375_VERBOSE later.
 */

#include "ch375_zephyr_adapter.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h> /* CONTAINER_OF */
#include <string.h>
#include <stdio.h>

LOG_MODULE_REGISTER(ch375_zephyr_adapter, LOG_LEVEL_INF);

/* CH375 API / error codes */
#include "ch375.h"
#include "ch375_interface.h"

/* Verbose logging toggle for extra traces (set to 0 to reduce log spam) */
#ifndef CH375_VERBOSE
#define CH375_VERBOSE 1
#endif

/* verbose wrappers */
#define VLOG_INF(...) do { if (CH375_VERBOSE) LOG_INF(__VA_ARGS__); } while (0)
#define VLOG_DBG(...) do { if (CH375_VERBOSE) LOG_DBG(__VA_ARGS__); } while (0)
#define VLOG_WRN(...) do { if (CH375_VERBOSE) LOG_WRN(__VA_ARGS__); } while (0)

/* Config: maximum number of CH375 instances supported by this adapter pool */
#ifndef MAX_CH375_INSTANCES
#define MAX_CH375_INSTANCES 2
#endif

/* Timeouts & retries (tune if needed) */
#define USARTRW_TX_TIMEOUT_MS 500
#define USARTRW_RX_TIMEOUT_MS 300
#define MBIT_RETRY 2
#define RECONFIG_RETRY_DELAY_MS 5

/* Forward declarations for STM32 register helpers (STM32F4 series) */
#if defined(CONFIG_SOC_SERIES_STM32F4X)
#include <stm32f4xx.h>
static USART_TypeDef *get_usart_from_label(const char *label);
static void dump_usart_registers(USART_TypeDef *usart, const char *title);
static void configure_usart_for_9bit(USART_TypeDef *usart);
static void ensure_brr_and_mbit(USART_TypeDef *usart, uint32_t desired_brr);
static int usart_write9(USART_TypeDef *usart, uint16_t value);
static int usart_read9(USART_TypeDef *usart, uint16_t *out, uint32_t timeout_ms);
static uint32_t compute_brr_from_pclk(uint32_t pclk, uint32_t baud);
#endif /* CONFIG_SOC_SERIES_STM32F4X */

/* priv structure passed to ch375_openContext() */
typedef struct {
    bool used;
    const struct device *uart;
    const struct device *gpio;
    gpio_pin_t int_pin;
    struct gpio_callback gcb;
    struct k_sem int_sem;
    uint32_t rx_timeout_ms;
    /* store labels for logging convenience */
    const char *uart_label;
    const char *gpio_label;
} ch375_adapter_priv_t;

/* static pool */
static ch375_adapter_priv_t s_pool[MAX_CH375_INSTANCES];

/* forward declarations for functions passed to ch375_openContext() */
static int z_write_cmd(CH375_Context_t *context, uint8_t cmd);
static int z_write_data(CH375_Context_t *context, uint8_t data);
static int z_read_data(CH375_Context_t *context, uint8_t *data);
static int z_query_int(CH375_Context_t *context);
static void ch375_gpio_callback(const struct device *gpiodev, struct gpio_callback *cb, gpio_port_pins_t pins);

/* helper: allocate a free slot from the static pool */
static ch375_adapter_priv_t *alloc_slot(const char *uart_label, const char *gpio_label)
{
    for (int i = 0; i < MAX_CH375_INSTANCES; ++i) {
        if (!s_pool[i].used) {
            memset(&s_pool[i], 0, sizeof(s_pool[i]));
            s_pool[i].used = true;
            s_pool[i].uart_label = uart_label;
            s_pool[i].gpio_label = gpio_label;
            return &s_pool[i];
        }
    }
    return NULL;
}

/* helper: free slot */
static void free_slot(ch375_adapter_priv_t *p)
{
    if (!p) return;
    if (p->gpio) {
        gpio_remove_callback(p->gpio, &p->gcb);
    }
    p->used = false;
    memset(p, 0, sizeof(*p));
}

/* ---------- ch375_zephyr_open ---------- */

int ch375_zephyr_open(const char *uart_label,
                      const char *gpio_label,
                      uint32_t int_pin,
                      CH375_Context_t **ctx_out)
{
    int ret;
    ch375_adapter_priv_t *priv = NULL;
    const struct device *uart_dev;
    const struct device *gpio_dev;

    if (!uart_label || !gpio_label || !ctx_out) {
        LOG_ERR("invalid params");
        return CH375_PARAM_INVALID;
    }

    uart_dev = device_get_binding(uart_label);
    if (!uart_dev) {
        LOG_ERR("UART device '%s' not found", uart_label);
        return CH375_PARAM_INVALID;
    }
    LOG_INF("UART binding ok: %s -> %p", uart_label, uart_dev);

    gpio_dev = device_get_binding(gpio_label);
    if (!gpio_dev) {
        LOG_ERR("GPIO device '%s' not found", gpio_label);
        return CH375_PARAM_INVALID;
    }
    LOG_INF("GPIO binding ok: %s -> %p", gpio_label, gpio_dev);

    /* allocate from static pool */
    priv = alloc_slot(uart_label, gpio_label);
    if (!priv) {
        LOG_ERR("no free CH375 adapter slots");
        return CH375_PARAM_INVALID;
    }

    priv->uart = uart_dev;
    priv->gpio = gpio_dev;
    priv->int_pin = (gpio_pin_t)int_pin;
    priv->rx_timeout_ms = USARTRW_RX_TIMEOUT_MS;

    k_sem_init(&priv->int_sem, 0, 1);

    /* ===== CONFIGURE USART FOR 9-BIT MODE ONCE AT OPEN ===== */
#if defined(CONFIG_SOC_SERIES_STM32F4X)
    USART_TypeDef *usart = get_usart_from_label(uart_label);
    if (usart) {
        LOG_INF("Configuring %s for 9-bit mode", uart_label);
        
        /* Disable Zephyr UART driver IRQs */
        #if defined(CONFIG_UART_INTERRUPT_DRIVEN)
            uart_irq_rx_disable(uart_dev);
            uart_irq_tx_disable(uart_dev);
        #endif
        
        /* Wait for any pending TX */
        uint32_t timeout = k_uptime_get_32() + 100;
        while (!(usart->SR & USART_SR_TC)) {
            if (k_uptime_get_32() > timeout) {
                LOG_WRN("Timeout waiting for TC");
                break;
            }
            k_yield();
        }
        
        dump_usart_registers(usart, "BEFORE 9-bit config");
        
        /* Configure for 9-bit mode */
        configure_usart_for_9bit(usart);
        
        /* Set BRR for 9600 baud */
        uint32_t pclk1;
        uint32_t ppre1 = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
        uint32_t hclk = SystemCoreClock;
        uint32_t div;
        if (ppre1 < 4) div = 1;
        else if (ppre1 == 4) div = 2;
        else if (ppre1 == 5) div = 4;
        else if (ppre1 == 6) div = 8;
        else div = 16;
        pclk1 = hclk / div;
        
        uint32_t brr = compute_brr_from_pclk(pclk1, 9600);
        usart->CR1 &= ~USART_CR1_UE;
        usart->BRR = brr;
        usart->CR1 |= USART_CR1_UE;
        
        LOG_INF("SYSCLK=%lu Hz, PCLK1=%lu Hz, BRR=0x%04X",
                (unsigned long)SystemCoreClock, (unsigned long)pclk1, brr);
        
        dump_usart_registers(usart, "AFTER 9-bit config");
        
    } else {
        LOG_WRN("No USART mapping for '%s', 8-bit fallback", uart_label);
    }
#endif
    /* ===== END ONE-TIME CONFIGURATION ===== */

    /* Configure GPIO */
    ret = gpio_pin_configure(priv->gpio, priv->int_pin, GPIO_INPUT | GPIO_PULL_UP);
    if (ret) {
        LOG_ERR("gpio_pin_configure failed (%d)", ret);
        free_slot(priv);
        return CH375_PARAM_INVALID;
    }

    gpio_init_callback(&priv->gcb, ch375_gpio_callback, BIT(priv->int_pin));
    ret = gpio_add_callback(priv->gpio, &priv->gcb);
    if (ret) {
        LOG_ERR("gpio_add_callback failed (%d)", ret);
        free_slot(priv);
        return CH375_PARAM_INVALID;
    }

    ret = gpio_pin_interrupt_configure(priv->gpio, priv->int_pin, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret) {
        LOG_ERR("gpio_pin_interrupt_configure failed (%d)", ret);
        gpio_remove_callback(priv->gpio, &priv->gcb);
        free_slot(priv);
        return CH375_PARAM_INVALID;
    }

    /* open ch375 context */
    ret = ch375_openContext((CH375_Context_t **)ctx_out,
                            z_write_cmd,
                            z_write_data,
                            z_read_data,
                            z_query_int,
                            (void *)priv);
    if (ret != CH375_SUCCESS) {
        LOG_ERR("ch375_openContext failed (%d)", ret);
        gpio_pin_interrupt_configure(priv->gpio, priv->int_pin, GPIO_INT_DISABLE);
        gpio_remove_callback(priv->gpio, &priv->gcb);
        free_slot(priv);
        return CH375_PARAM_INVALID;
    }

    LOG_INF("CH375 Zephyr adapter opened (UART=%s, GPIO=%s, INT=%u)",
            uart_label, gpio_label, (unsigned)int_pin);

    return CH375_SUCCESS;
}

int ch375_zephyr_close(CH375_Context_t *ctx)
{
    if (!ctx) return CH375_PARAM_INVALID;
    ch375_adapter_priv_t *priv = (ch375_adapter_priv_t *)ch375_getPriv(ctx);
    if (!priv) return ch375_writeCmd_FAILD;

    gpio_pin_interrupt_configure(priv->gpio, priv->int_pin, GPIO_INT_DISABLE);
    gpio_remove_callback(priv->gpio, &priv->gcb);

    ch375_closeContext(ctx);
    free_slot(priv);

    LOG_INF("CH375 Zephyr adapter closed");
    return CH375_SUCCESS;
}

/* ---------- STM32F4 register helpers ---------- */
#if defined(CONFIG_SOC_SERIES_STM32F4X)

/* crude label -> USART mapping (use device name you observed in logs) */
static USART_TypeDef *get_usart_from_label(const char *label)
{
    if (!label) return NULL;

    /* prefer node names that include the base address */
    if (strstr(label, "40004400") || strstr(label, "USART2") || strstr(label, "usart2") || strstr(label, "USART_2")) {
        return USART2;
    }
    if (strstr(label, "40004800") || strstr(label, "USART3") || strstr(label, "usart3") || strstr(label, "USART_3")) {
        return USART3;
    }
    return NULL;
}

/* compute BRR according to STM32F4 (oversampling by 16) */
static uint32_t compute_brr_from_pclk(uint32_t pclk, uint32_t baud)
{
    if (baud == 0) return 0;
    /* USARTDIV = pclk / (16 * baud) */
    double usartdiv = (double)pclk / ((double)16 * (double)baud);
    uint32_t mant = (uint32_t)usartdiv;
    uint32_t frac = (uint32_t)((usartdiv - (double)mant) * 16.0 + 0.5);
    if (frac >= 16) { mant++; frac = 0; }
    return (mant << 4) | (frac & 0xF);
}

static void dump_usart_registers(USART_TypeDef *usart, const char *title)
{
    if (!usart) return;
    LOG_INF("=== %s USART @%p ===", title, (void *)usart);
    LOG_INF("  CR1 = 0x%08X (M=%u PCE=%u TE=%u RE=%u UE=%u)",
            usart->CR1,
            !!(usart->CR1 & USART_CR1_M),
            !!(usart->CR1 & USART_CR1_PCE),
            !!(usart->CR1 & USART_CR1_TE),
            !!(usart->CR1 & USART_CR1_RE),
            !!(usart->CR1 & USART_CR1_UE));
    LOG_INF("  CR2 = 0x%08X", usart->CR2);
    LOG_INF("  CR3 = 0x%08X", usart->CR3);
    LOG_INF("  SR  = 0x%08X (TXE=%u TC=%u RXNE=%u FE=%u ORE=%u PE=%u)",
            usart->SR,
            !!(usart->SR & USART_SR_TXE),
            !!(usart->SR & USART_SR_TC),
            !!(usart->SR & USART_SR_RXNE),
            !!(usart->SR & USART_SR_FE),
            !!(usart->SR & USART_SR_ORE),
            !!(usart->SR & USART_SR_PE));
    LOG_INF("  BRR = 0x%08X", usart->BRR);
}

/* Minimal helper to set M = 1 (9-bit words) and enable TE/RE.
 * We purposely do minimal touching: do not change baud if already configured.
 */
static void configure_usart_for_9bit(USART_TypeDef *usart)
{
    if (!usart) return;

    /* disable USART */
    usart->CR1 &= ~USART_CR1_UE;

    /* set M = 1 (9-bit), parity disabled, enable TE/RE */
    usart->CR1 |= USART_CR1_M;
    usart->CR1 &= ~USART_CR1_PCE;
    usart->CR1 |= (USART_CR1_TE | USART_CR1_RE);

    /* keep STOP bits as-is; ensure CR2 STOP cleared to 1 stop bit (no harm) */
    usart->CR2 &= ~USART_CR2_STOP;

    /* disable HW flow control */
    usart->CR3 &= ~(USART_CR3_RTSE | USART_CR3_CTSE);

    /* enable USART */
    usart->CR1 |= USART_CR1_UE;

    /* flush */
    volatile uint32_t tmp = usart->SR;
    tmp = usart->DR;
    (void)tmp;
}

/* ensure BRR set to desired value and M bit present. Called before transactions */
static void ensure_brr_and_mbit(USART_TypeDef *usart, uint32_t desired_brr)
{
    if (!usart) return;

    /* if BRR already equals desired then only ensure M bit set */
    if (usart->BRR != desired_brr) {
        VLOG_WRN("Overwriting BRR: actual=0x%08X desired=0x%08X", (unsigned)usart->BRR, (unsigned)desired_brr);
        usart->CR1 &= ~USART_CR1_UE;
        usart->BRR = desired_brr;
        usart->CR1 |= USART_CR1_UE;
    }
    /* ensure 9-bit */
    if (!(usart->CR1 & USART_CR1_M)) {
        VLOG_WRN("M bit not set; setting it");
        configure_usart_for_9bit(usart);
    }
}

static int usart_write9(USART_TypeDef *usart, uint16_t value)
{
    if (!usart) return -1;
    uint32_t start = k_uptime_get_32();
    while (!(usart->SR & USART_SR_TXE)) {
        if ((k_uptime_get_32() - start) > USARTRW_TX_TIMEOUT_MS) {
            return -2;
        }
        k_yield();  // Don't k_msleep(0) in tight loop
    }
    usart->DR = (uint16_t)(value & 0x1FF);
    start = k_uptime_get_32();
    while (!(usart->SR & USART_SR_TC)) {
        if ((k_uptime_get_32() - start) > USARTRW_TX_TIMEOUT_MS) {
            break;
        }
        k_yield();
    }
    return 0;
}

static int usart_read9(USART_TypeDef *usart, uint16_t *out, uint32_t timeout_ms)
{
    if (!usart || !out) return -1;
    uint32_t start = k_uptime_get_32();
    while (!(usart->SR & USART_SR_RXNE)) {
        if ((k_uptime_get_32() - start) > timeout_ms) {
            return -2;
        }
        k_yield();
    }
    uint32_t sr = usart->SR;
    uint16_t v = (uint16_t)(usart->DR & 0x1FF);
    
    /* Only log errors */
    if (sr & (USART_SR_FE | USART_SR_ORE | USART_SR_PE)) {
        LOG_WRN("USART error: FE=%d ORE=%d PE=%d", 
                !!(sr & USART_SR_FE), !!(sr & USART_SR_ORE), !!(sr & USART_SR_PE));
    }
    
    *out = v;
    return 0;
}

#endif /* CONFIG_SOC_SERIES_STM32F4X */

/* ---------- Implementation of function pointers (z_* wrappers) ---------- */

static const struct device *lookup_uart_device_from_priv(ch375_adapter_priv_t *priv)
{
    if (!priv) return NULL;
    return priv->uart;
}

static USART_TypeDef *lookup_native_usart(ch375_adapter_priv_t *priv)
{
#if defined(CONFIG_SOC_SERIES_STM32F4X)
    if (!priv || !priv->uart_label) return NULL;
    return get_usart_from_label(priv->uart_label);
#else
    ARG_UNUSED(priv);
    return NULL;
#endif
}

/* helper to attempt to claim and prepare native USART for 9-bit transfer */
static void prepare_usart_for_transfer_if_possible(ch375_adapter_priv_t *priv, uint32_t baud)
{
#if defined(CONFIG_SOC_SERIES_STM32F4X)
    USART_TypeDef *usart = lookup_native_usart(priv);
    if (!usart) return;

    /* compute expected BRR from PCLK1 (read PCLK1 from RCC registers) */
    uint32_t pclk1;
#if defined(SYSCLK_FREQ)
    pclk1 = HAL_RCC_GetPCLK1Freq(); /* if HAL available */
#else
    /* derive PCLK1 from RCC registers (CMSIS) */
    uint32_t ppre1 = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
    uint32_t hclk = SystemCoreClock; /* SystemCoreClock should be maintained in Zephyr build too */
    /* decode prescaler: from ppre1 bits to divider */
    uint32_t div;
    if (ppre1 < 4) div = 1;
    else if (ppre1 == 4) div = 2;
    else if (ppre1 == 5) div = 4;
    else if (ppre1 == 6) div = 8;
    else div = 16;
    pclk1 = hclk / div;
#endif

    uint32_t expected_brr_9600 = compute_brr_from_pclk(pclk1, 9600);
    uint32_t expected_brr_115200 = compute_brr_from_pclk(pclk1, 115200);
    VLOG_INF("Clock debug: SYSCLK=%lu Hz, PCLK1=%lu Hz, test_baud=%lu",
             (unsigned long)SystemCoreClock, (unsigned long)pclk1, (unsigned long)baud);

    /* Attempt a small retry loop to set M bit and BRR */
    for (int i = 0; i < MBIT_RETRY; ++i) {
        configure_usart_for_9bit(usart);
        ensure_brr_and_mbit(usart, expected_brr_9600);
        if ((usart->CR1 & USART_CR1_M) && usart->BRR == expected_brr_9600) break;
        k_msleep(RECONFIG_RETRY_DELAY_MS);
    }
    /* Also try 115200 in case target switched */
    for (int i = 0; i < MBIT_RETRY; ++i) {
        ensure_brr_and_mbit(usart, expected_brr_115200);
        if ((usart->CR1 & USART_CR1_M) && usart->BRR == expected_brr_115200) break;
        k_msleep(RECONFIG_RETRY_DELAY_MS);
    }
    dump_usart_registers(usart, "AFTER configure attempt");
#else
    ARG_UNUSED(priv);
    ARG_UNUSED(baud);
#endif
}

/* z_write_cmd: command writes set 9th bit = 1 */
static int z_write_cmd(CH375_Context_t *context, uint8_t cmd)
{
    ch375_adapter_priv_t *priv = (ch375_adapter_priv_t *)ch375_getPriv(context);
    if (!priv || !priv->uart) {
        return ch375_writeCmd_FAILD;
    }

    /* Ensure native USART is in 9-bit mode + BRR (try to be defensive) */
    prepare_usart_for_transfer_if_possible(priv, 9600);

#if defined(CONFIG_SOC_SERIES_STM32F4X)
    USART_TypeDef *usart = get_usart_from_label(priv->uart_label);
    if (usart) {
        uint16_t txval = 0x100 | (uint16_t)cmd; /* 9th bit = 1 */
        VLOG_DBG("z_write_cmd: usart write 0x%03X", txval);
        int rc = usart_write9(usart, txval);
        return (rc == 0) ? CH375_SUCCESS : ch375_writeCmd_FAILD;
    }
#endif

    VLOG_DBG("z_write_cmd: uart_poll_out 0x%02X", cmd);
    uart_poll_out(priv->uart, cmd);
    return CH375_SUCCESS;
}

/* z_write_data: data writes set 9th bit = 0 */
static int z_write_data(CH375_Context_t *context, uint8_t data)
{
    ch375_adapter_priv_t *priv = (ch375_adapter_priv_t *)ch375_getPriv(context);
    if (!priv || !priv->uart) {
        return ch375_writeCmd_FAILD;
    }

    /* Ensure native USART is in 9-bit mode + BRR (defensive) */
    prepare_usart_for_transfer_if_possible(priv, 9600);

#if defined(CONFIG_SOC_SERIES_STM32F4X)
    USART_TypeDef *usart = get_usart_from_label(priv->uart_label);
    if (usart) {
        uint16_t txval = (uint16_t)data & 0xFF; /* 9th bit = 0 */
        VLOG_DBG("z_write_data: usart write 0x%03X", txval);
        int rc = usart_write9(usart, txval);
        return (rc == 0) ? CH375_SUCCESS : ch375_writeCmd_FAILD;
    }
#endif

    VLOG_DBG("z_write_data: uart_poll_out 0x%02X", data);
    uart_poll_out(priv->uart, data);
    return CH375_SUCCESS;
}

/* z_read_data: read 9-bit value, return lower 8 bits */
static int z_read_data(CH375_Context_t *context, uint8_t *data)
{
    ch375_adapter_priv_t *priv = (ch375_adapter_priv_t *)ch375_getPriv(context);
    if (!priv || !priv->uart || !data) {
        return ch375_readData_FAILD;
    }

    /* Ensure USART configured for reading as well */
    prepare_usart_for_transfer_if_possible(priv, 9600);

#if defined(CONFIG_SOC_SERIES_STM32F4X)
    USART_TypeDef *usart = get_usart_from_label(priv->uart_label);
    if (usart) {
        uint16_t v;
        int rc = usart_read9(usart, &v, priv->rx_timeout_ms);
        if (rc == 0) {
            *data = (uint8_t)(v & 0xFF);
            VLOG_DBG("z_read_data: got 0x%03X -> 0x%02X", v, *data);
            return CH375_SUCCESS;
        }
        VLOG_WRN("z_read_data: usart_read9 rc=%d", rc);
        return ch375_readData_FAILD;
    }
#endif

    unsigned char c;
    int rc = uart_poll_in(priv->uart, &c);
    if (rc == 0) {
        *data = (uint8_t)c;
        VLOG_DBG("z_read_data: uart_poll_in -> 0x%02X", *data);
        return CH375_SUCCESS;
    }
    VLOG_WRN("z_read_data: uart_poll_in rc=%d", rc);
    return ch375_readData_FAILD;
}


/* non-blocking query for interrupt - returns 0 if no int, non-zero if int pending */
static int z_query_int(CH375_Context_t *context)
{
    ch375_adapter_priv_t *priv = (ch375_adapter_priv_t *)ch375_getPriv(context);
    if (!priv) return 0;

    if (k_sem_take(&priv->int_sem, K_NO_WAIT) == 0) {
        return 1;
    }
    return 0;
}

/* GPIO callback - called from ISR context */
static void ch375_gpio_callback(const struct device *gpiodev, struct gpio_callback *cb, gpio_port_pins_t pins)
{
    ARG_UNUSED(gpiodev);
    ch375_adapter_priv_t *priv = CONTAINER_OF(cb, ch375_adapter_priv_t, gcb);
    k_sem_give(&priv->int_sem);
}

/* end of file */
