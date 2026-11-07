#ifndef DRIVERS_UART_ASYNC_H
#define DRIVERS_UART_ASYNC_H

#include <stdbool.h>
#include <stdint.h>

void uart_async_init(void);
void uart_async_init_polling(void);
void uart_async_poll_rx(void);
void uart_async_poll_tx(void);
bool uart_async_read(char *value);
bool uart_async_write(const char *data, uint16_t length);
bool uart_async_write_line(const char *text);
bool uart_async_rx_overflowed(void);
bool uart_async_tx_overflowed(void);
bool uart_async_tx_idle(void);
/* Number of bytes observed on the UART RX path since initialization. */
uint32_t uart_async_rx_received_count(void);

#endif
