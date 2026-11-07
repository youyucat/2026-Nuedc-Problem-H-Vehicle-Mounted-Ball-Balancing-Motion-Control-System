#include "hal/hal_uart.h"

#include <stddef.h>

#include "ti_msp_dl_config.h"

/*
 * MSPM0G3507 implementation of the hal_uart interface.
 *
 * HOST = UART0 (PC link, PA10 TX / PA11 RX, priority 0)
 * 本工程只保留 HOST 通道（VISION/UART2 已裁掉）。
 *
 * Baud rate, clocking and pin routing come from SysConfig
 * (SYSCFG_DL_UART_0_init in hal_system_bringup).
 * This file only owns the FIFO thresholds, NVIC configuration and the ISRs;
 * the byte-level protocol/ring buffers live in drivers/.
 */

typedef struct {
    UART_Regs *instance;
    uint32_t irqn;
    uint32_t priority;
} hal_uart_dev_t;

static const hal_uart_dev_t s_devices[HAL_UART_COUNT] = {
    { UART_0_INST,       UART_0_INST_INT_IRQN,       0U }
};

static hal_uart_rx_fn s_rx_cb[HAL_UART_COUNT];
static hal_uart_tx_empty_fn s_tx_empty_cb[HAL_UART_COUNT];

static UART_Regs *instance_of(hal_uart_channel_t ch)
{
    if (ch >= HAL_UART_COUNT) return NULL;
    return s_devices[ch].instance;
}

static void drain_rx(hal_uart_channel_t ch)
{
    UART_Regs *inst = instance_of(ch);
    hal_uart_rx_fn rx = s_rx_cb[ch];

    if ((inst == NULL) || (rx == NULL)) return;
    while (!DL_UART_Main_isRXFIFOEmpty(inst)) {
        rx((uint8_t)DL_UART_Main_receiveData(inst));
    }
}

static void service_isr(hal_uart_channel_t ch)
{
    UART_Regs *inst = instance_of(ch);
    hal_uart_tx_empty_fn tx_empty = s_tx_empty_cb[ch];

    if (inst == NULL) return;
    for (;;) {
        uint32_t pending = DL_UART_Main_getPendingInterrupt(inst);

        if (pending == DL_UART_MAIN_IIDX_RX) {
            drain_rx(ch);
        } else if (pending == DL_UART_MAIN_IIDX_TX) {
            if (tx_empty != NULL) tx_empty();
        } else if (!DL_UART_Main_isRXFIFOEmpty(inst)) {
            drain_rx(ch);
        } else {
            break;
        }
    }
}

void UART_0_INST_IRQHandler(void)
{
    service_isr(HAL_UART_HOST);
}

void hal_uart_init(hal_uart_channel_t ch)
{
    UART_Regs *inst = instance_of(ch);

    if (inst == NULL) return;
    DL_UART_Main_disableInterrupt(
        inst, DL_UART_MAIN_INTERRUPT_RX | DL_UART_MAIN_INTERRUPT_TX);
    while (!DL_UART_Main_isRXFIFOEmpty(inst)) {
        (void)DL_UART_Main_receiveData(inst);
    }
    DL_UART_Main_clearInterruptStatus(inst, DL_UART_MAIN_INTERRUPT_RX);
    DL_UART_Main_enableFIFOs(inst);
    DL_UART_Main_setRXFIFOThreshold(
        inst, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_setTXFIFOThreshold(inst, DL_UART_TX_FIFO_LEVEL_EMPTY);
    DL_UART_Main_enableInterrupt(inst, DL_UART_MAIN_INTERRUPT_RX);

    NVIC_SetPriority((IRQn_Type)s_devices[ch].irqn,
                     (uint32_t)s_devices[ch].priority);
    NVIC_ClearPendingIRQ((IRQn_Type)s_devices[ch].irqn);
    NVIC_EnableIRQ((IRQn_Type)s_devices[ch].irqn);
}

void hal_uart_init_polling(hal_uart_channel_t ch)
{
    UART_Regs *inst = instance_of(ch);

    if (inst == NULL) return;
    DL_UART_Main_disableInterrupt(
        inst, DL_UART_MAIN_INTERRUPT_RX | DL_UART_MAIN_INTERRUPT_TX);
    NVIC_DisableIRQ((IRQn_Type)s_devices[ch].irqn);
    NVIC_ClearPendingIRQ((IRQn_Type)s_devices[ch].irqn);
}

void hal_uart_register(hal_uart_channel_t ch,
                       hal_uart_rx_fn rx,
                       hal_uart_tx_empty_fn tx_empty)
{
    if (ch >= HAL_UART_COUNT) return;
    s_rx_cb[ch] = rx;
    s_tx_empty_cb[ch] = tx_empty;
}

void hal_uart_flush_rx(hal_uart_channel_t ch)
{
    UART_Regs *inst = instance_of(ch);

    if (inst == NULL) return;
    while (!DL_UART_Main_isRXFIFOEmpty(inst)) {
        (void)DL_UART_Main_receiveData(inst);
    }
}

bool hal_uart_rx_available(hal_uart_channel_t ch)
{
    UART_Regs *inst = instance_of(ch);

    return (inst != NULL) && !DL_UART_Main_isRXFIFOEmpty(inst);
}

uint8_t hal_uart_rx_read(hal_uart_channel_t ch)
{
    UART_Regs *inst = instance_of(ch);

    if (inst == NULL) return 0U;
    return (uint8_t)DL_UART_Main_receiveData(inst);
}

bool hal_uart_tx_space(hal_uart_channel_t ch)
{
    UART_Regs *inst = instance_of(ch);

    return (inst != NULL) && !DL_UART_Main_isTXFIFOFull(inst);
}

void hal_uart_tx_write(hal_uart_channel_t ch, uint8_t byte)
{
    UART_Regs *inst = instance_of(ch);

    if (inst == NULL) return;
    DL_UART_Main_transmitData(inst, byte);
}

void hal_uart_tx_irq_clear(hal_uart_channel_t ch)
{
    UART_Regs *inst = instance_of(ch);

    if (inst == NULL) return;
    DL_UART_Main_clearInterruptStatus(inst, DL_UART_MAIN_INTERRUPT_TX);
}

void hal_uart_tx_irq_set(hal_uart_channel_t ch, bool enabled)
{
    UART_Regs *inst = instance_of(ch);

    if (inst == NULL) return;
    if (enabled) {
        DL_UART_Main_enableInterrupt(inst, DL_UART_MAIN_INTERRUPT_TX);
    } else {
        DL_UART_Main_disableInterrupt(inst, DL_UART_MAIN_INTERRUPT_TX);
    }
}

bool hal_uart_tx_busy(hal_uart_channel_t ch)
{
    UART_Regs *inst = instance_of(ch);

    return (inst != NULL) && DL_UART_Main_isBusy(inst);
}
