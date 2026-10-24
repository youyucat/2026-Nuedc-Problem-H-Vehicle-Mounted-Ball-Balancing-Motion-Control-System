#include "hal/hal_system.h"

#include "ti_msp_dl_config.h"

/*
 * MSPM0G3507 implementation of the hal_system interface（循迹专用工程）.
 *
 * hal_system_bringup() owns the SysConfig-generated power/clock/peripheral
 * initializers. 本工程只生成 SYSCTL（80 MHz 时钟树）、ADC0（灰度 MUX 输出）
 * 和 SysTick 三块，因此这里也只调用这三个 init：
 *   - 源工程里的 UART0/UART_VISION RX 上拉、I2C1 SDA/SCL 上拉与
 *     SYSCFG_DL_UART_*_init()/SYSCFG_DL_I2C_1_init()/SYSCFG_DL_DMA_init()
 *     全部删除（对应外设已从 .syscfg 移除，生成代码里也不再有这些函数）。
 *   - SYSCFG_DL_SYSCTL_CLK_init() 与源工程一样**不调用**：上游 bringup 只靠
 *     SYSCFG_DL_SYSCTL_init() 完成时钟树配置，多调一行会改变启动时序。
 */

void hal_system_bringup(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_ADC_0_init();
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
