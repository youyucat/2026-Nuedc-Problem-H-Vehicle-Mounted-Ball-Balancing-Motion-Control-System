#include "drivers/uart_async.h"

#include <stddef.h>
#include <string.h>

#include "hal/hal_uart.h"
#include "hal/hal_system.h"

#define UART_RX_SIZE  256U
#define UART_TX_SIZE  1024U
#define UART_RX_MASK  (UART_RX_SIZE - 1U)
#define UART_TX_MASK  (UART_TX_SIZE - 1U)

static volatile uint8_t s_rx[UART_RX_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static volatile bool s_rx_overflow;
static volatile uint32_t s_rx_received_count;
static uint8_t s_tx[UART_TX_SIZE];
static volatile uint16_t s_tx_head;
static volatile uint16_t s_tx_tail;
static bool s_tx_overflow;

static void service_tx_fifo(void)
{
    while ((s_tx_tail != s_tx_head) &&
           hal_uart_tx_space(HAL_UART_HOST)) {
        hal_uart_tx_write(HAL_UART_HOST, s_tx[s_tx_tail]);
        s_tx_tail = (uint16_t)((s_tx_tail + 1U) & UART_TX_MASK);
    }
}

static void kick_tx(void)
{
    uint32_t interrupt_state = hal_system_critical_save();

    service_tx_fifo();
    hal_uart_tx_irq_clear(HAL_UART_HOST);
    if (s_tx_tail != s_tx_head) {
        hal_uart_tx_irq_set(HAL_UART_HOST, true);
    } else {
        hal_uart_tx_irq_set(HAL_UART_HOST, false);
    }
    hal_system_critical_restore(interrupt_state);
}

static void reset_buffers(void)
{
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_rx_overflow = false;
    s_rx_received_count = 0U;
    s_tx_head = 0U;
    s_tx_tail = 0U;
    s_tx_overflow = false;
}

static void uart_async_store_rx_byte(uint8_t value)
{
    uint16_t head = s_rx_head;
    uint16_t next = (uint16_t)((head + 1U) & UART_RX_MASK);

    ++s_rx_received_count;

    if (next == s_rx_tail) {
        s_rx_overflow = true;
    } else {
        s_rx[head] = value;
        s_rx_head = next;
    }
}

static void on_rx_byte(uint8_t byte)
{
    uart_async_store_rx_byte(byte);
}

static void on_tx_empty(void)
{
    service_tx_fifo();
    if (s_tx_tail == s_tx_head) {
        hal_uart_tx_irq_set(HAL_UART_HOST, false);
    }
}

void uart_async_init(void)
{
    reset_buffers();

    /* RX must be interrupt driven in the complete template. A foreground
     * polling receiver loses bytes while the eight-channel gray ADC scan is
     * in progress. The HAL keeps the FIFO threshold at one byte and delivers
     * every byte through the RX callback below. */
    hal_uart_register(HAL_UART_HOST, on_rx_byte, on_tx_empty);
    hal_uart_init(HAL_UART_HOST);
}

void uart_async_init_polling(void)
{
    reset_buffers();
    hal_uart_init_polling(HAL_UART_HOST);
}

bool uart_async_read(char *value)
{
    uint16_t tail = s_rx_tail;

    if ((value == NULL) || (tail == s_rx_head)) return false;
    *value = (char)s_rx[tail];
    s_rx_tail = (uint16_t)((tail + 1U) & UART_RX_MASK);
    return true;
}

void uart_async_poll_rx(void)
{
    while (hal_uart_rx_available(HAL_UART_HOST)) {
        uart_async_store_rx_byte(hal_uart_rx_read(HAL_UART_HOST));
    }
}

static uint16_t tx_free(void)
{
    return (uint16_t)((s_tx_tail - s_tx_head - 1U) & UART_TX_MASK);
}

bool uart_async_write(const char *data, uint16_t length)
{
    uint16_t head;

    if ((data == NULL) || (length == 0U)) return false;
    if (length > tx_free()) {
        s_tx_overflow = true;
        return false;
    }
    head = s_tx_head;
    for (uint16_t i = 0U; i < length; ++i) {
        s_tx[head] = (uint8_t)data[i];
        head = (uint16_t)((head + 1U) & UART_TX_MASK);
    }
    s_tx_head = head;
    kick_tx();
    return true;
}

bool uart_async_write_line(const char *text)
{
    uint16_t length;
    uint16_t required;
    uint16_t head;

    if (text == NULL) return false;
    length = (uint16_t)strlen(text);
    required = (uint16_t)(length + 2U);
    if (required > tx_free()) {
        s_tx_overflow = true;
        return false;
    }
    head = s_tx_head;
    for (uint16_t i = 0U; i < length; ++i) {
        s_tx[head] = (uint8_t)text[i];
        head = (uint16_t)((head + 1U) & UART_TX_MASK);
    }
    s_tx[head] = '\r';
    head = (uint16_t)((head + 1U) & UART_TX_MASK);
    s_tx[head] = '\n';
    s_tx_head = (uint16_t)((head + 1U) & UART_TX_MASK);
    kick_tx();
    return true;
}

void uart_async_poll_tx(void)
{
    if (s_tx_tail != s_tx_head) kick_tx();
}

bool uart_async_rx_overflowed(void)
{
    bool value = s_rx_overflow;
    s_rx_overflow = false;
    return value;
}

bool uart_async_tx_overflowed(void)
{
    bool value = s_tx_overflow;
    s_tx_overflow = false;
    return value;
}

bool uart_async_tx_idle(void)
{
    return (s_tx_tail == s_tx_head) &&
           !hal_uart_tx_busy(HAL_UART_HOST);
}

uint32_t uart_async_rx_received_count(void)
{
    return s_rx_received_count;
}
