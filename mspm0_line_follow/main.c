/*
 * line_follow_mspm0g3507 —— 双芯架构中的「循迹芯片」固件入口。
 *
 * 本工程由 empty_LP_MSPM0G3507_nortos_ticlang（循迹 + 步进球平衡一体机）
 * 抽出的循迹分支，只负责：
 *   灰度 8 路（MUX + ADC 中断异步帧）→ 加权重心 → PD 循迹 → 差速
 *   → 轮速增量 PI → TB6612 双直流电机
 * 另一颗芯片负责步进电机维持小球平衡（不在本工程内，两者通过车体机械
 * 解耦，互不共享外设）。
 *
 * 上电时序：
 *   1. 关中断做 SysConfig 电源/时钟/ADC/SysTick bring-up；
 *   2. 开中断后做板级初始化（灰度 MUX 引脚、ADC、TB6612、编码器）并
 *      同步预热一帧灰度 —— 这一帧必须靠 ADC 中断完成，所以放在开中断之后
 *      （源工程在关中断区间里做，预热帧实际会超时失败，这里修正）；
 *   3. 主循环跑 5 ms 节拍的循迹任务，等待 TEMPLATE_STARTUP_HOLD_MS 后自动起步。
 */

#include "hal/hal.h"

#include "app/line_task.h"

int main(void)
{
    hal_system_critical_enter();

    /* SysConfig 生成：initPower / GPIO / SYSCTL(80 MHz) / ADC_0 / SysTick */
    hal_system_bringup();

    hal_system_critical_exit();

    /* 板级初始化 + 灰度预热 + 电机待机（电机零输出，不会自己冲出去） */
    line_task_init();

    while (1) {
        line_task_step(hal_time_millis());
    }
}
