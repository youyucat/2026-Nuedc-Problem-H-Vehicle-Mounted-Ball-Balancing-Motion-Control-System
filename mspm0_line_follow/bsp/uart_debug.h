#ifndef BSP_UART_DEBUG_H
#define BSP_UART_DEBUG_H

#include <stdbool.h>
#include <stdint.h>

/*
 * 串口调试模块（UART0，PA10=TX / PA11=RX，115200 8N1）。
 *
 * 本工程此前没有任何输出通道：循迹误差、轮速、灰度原始值全都不可见，
 * 调参只能靠"车跑得像不像"。本模块补上这条观察路径，纯调试用途，
 * 不参与控制 —— 拔掉串口车照常跑。
 *
 * 发送走 TX 中断 + 环形缓冲，printf 不阻塞 5ms 控制拍；
 * 接收由主循环轮询（主循环一圈远快于 115200 的字节间隔，不会丢）。
 *
 * 命令以回车结束，例如 "stat\r\n"。串口助手勾上"发送新行"即可。
 */

void uart_debug_init(void);   /* hal_board_init 之后、主循环之前调用 */
void uart_debug_poll(void);   /* 主循环里每圈调用（收命令 + 周期打印） */

#endif
