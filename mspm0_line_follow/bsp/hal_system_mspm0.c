#include "hal/hal_system.h"

#include "ti_msp_dl_config.h"

/*
 * MSPM0G3507 implementation of the hal_system interface（循迹专用工程）.
 *
 * hal_system_bringup() owns the SysConfig-generated power/clock/peripheral
 * initializers. 本工程生成 SYSCTL（80 MHz 时钟树）、ADC0（灰度 MUX 输出）、
 * UART0（调试串口 PA10/PA11 115200）和 SysTick 四块。
 *   - UART0 由 SysConfig 配置（时钟 MFPCLK 32MHz 自动推导分频 + 引脚复用），
 *     2026-11-07 手写 UART 初始化反复踩波特率坑后改为生成代码接管。
 *   - 源工程里的 UART_VISION / I2C1 / DMA init 依旧删除（对应外设不在本工程）。
 *   - SYSCFG_DL_SYSCTL_CLK_init() 与源工程一样**不调用**：上游 bringup 只靠
 *     SYSCFG_DL_SYSCTL_init() 完成时钟树配置，多调一行会改变启动时序。
 */

void hal_system_bringup(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_ADC_0_init();
    SYSCFG_DL_UART_0_init();
    SYSCFG_DL_SYSTICK_init();
}

uint32_t hal_system_critical_save(void)
{
    uint32_t state = __get_PRIMASK();

    __disable_irq();
    return state;
}

void hal_system_critical_restore(uint32_t state)
{
    __set_PRIMASK(state);
}

void hal_system_critical_enter(void)
{
    __disable_irq();
}

void hal_system_critical_exit(void)
{
    __enable_irq();
}

void hal_system_reset(void)
{
    __DSB();
    DL_SYSCTL_resetDevice(DL_SYSCTL_RESET_SYSRST);
}
