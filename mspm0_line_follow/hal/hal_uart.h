#ifndef HAL_HAL_UART_H
#define HAL_HAL_UART_H

#include <stdbool.h>
#include <stdint.h>

/* Asynchronous UART channels used by the template. Each channel is
 * interrupt-driven: RX bytes are delivered to the registered callback from
 * the ISR, and the TX-empty event is reported so the ring-buffer layer can
 * pump more data. The callbacks are invoked from interrupt context and must
 * be short. */

typedef enum {
    HAL_UART_HOST = 0,   /* UART0: PC host / debug link（PA10/PA11） */
    /* 本工程只有调试串口一路（Vision 链路由 MaixCAM 侧独立负责），
     * 原版工程里的 HAL_UART_VISION(UART2) 通道已裁掉。 */
    HAL_UART_COUNT
} hal_uart_channel_t;

typedef void (*hal_uart_rx_fn)(uint8_t byte);
typedef void (*hal_uart_tx_empty_fn)(void);

/* Configure the channel (FIFO thresholds, RX interrupt, NVIC priority) and
 * clear stale RX data. Baud rate and pin routing come from SysConfig. */
void hal_uart_init(hal_uart_channel_t ch);

/* Disable the channel interrupts and NVIC line (polling-only mode). */
void hal_uart_init_polling(hal_uart_channel_t ch);

/* Attach the RX byte and TX-empty callbacks. Either may be NULL. */
void hal_uart_register(hal_uart_channel_t ch,
                       hal_uart_rx_fn rx,
                       hal_uart_tx_empty_fn tx_empty);

/* Discard any bytes currently in the hardware RX FIFO. */
void hal_uart_flush_rx(hal_uart_channel_t ch);

bool hal_uart_rx_available(hal_uart_channel_t ch);
uint8_t hal_uart_rx_read(hal_uart_channel_t ch);

bool hal_uart_tx_space(hal_uart_channel_t ch);
void hal_uart_tx_write(hal_uart_channel_t ch, uint8_t byte);

void hal_uart_tx_irq_clear(hal_uart_channel_t ch);
void hal_uart_tx_irq_set(hal_uart_channel_t ch, bool enabled);

/* True while the UART shift register still has data in flight. */
bool hal_uart_tx_busy(hal_uart_channel_t ch);

#endif
