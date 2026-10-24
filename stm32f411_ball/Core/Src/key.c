/**
  ******************************************************************************
  * @file    key.c
  * @brief   唯一按键（PA0）消抖
  *
  * 本文件属于用户代码，CubeMX 重新生成工程时不会被覆盖。
  ******************************************************************************
  */

#include "key.h"

#define KEY_ACTIVE_LEVEL   GPIO_PIN_RESET    /* 低电平按下 */

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint8_t       stable;        /* 消抖后的稳定状态：1=按下 */
    uint8_t       last_raw;      /* 上一次采样值 */
    uint16_t      cnt;           /* 连续相同次数 */
    uint8_t       evt_press;     /* 按下事件 */
    uint8_t       evt_release;   /* 松开事件 */
} key_t;

static key_t s_key;

static void key_hw_init(key_t *k, GPIO_TypeDef *port, uint16_t pin)
{
    k->port        = port;
    k->pin         = pin;
    k->stable      = 0;
    k->last_raw    = 0;
    k->cnt         = 0;
    k->evt_press   = 0;
    k->evt_release = 0;
}

void key_init(void)
{
    key_hw_init(&s_key, KEY_FWD_GPIO_Port, KEY_FWD_Pin);
}

static void key_update(key_t *k)
{
    uint8_t raw = (HAL_GPIO_ReadPin(k->port, k->pin) == KEY_ACTIVE_LEVEL) ? 1U : 0U;

    if (raw != k->last_raw) {          /* 电平变了，重新计时 */
        k->last_raw = raw;
        k->cnt      = 0;
        return;
    }

    if (raw != k->stable) {            /* 还没稳定，攒够时间再认 */
        k->cnt++;
        if (k->cnt >= KEY_DEBOUNCE_MS) {
            k->cnt    = 0;
            k->stable = raw;
            if (raw) k->evt_press   = 1U;
            else     k->evt_release = 1U;
        }
    } else {
        k->cnt = 0;
    }
}

void key_scan(void)
{
    key_update(&s_key);
}

uint8_t key1_pressed(void)  { return s_key.stable; }

uint8_t key1_clicked(void)
{
    uint8_t e = s_key.evt_press;
    s_key.evt_press = 0;
    return e;
}

uint8_t key1_released(void)
{
    uint8_t e = s_key.evt_release;
    s_key.evt_release = 0;
    return e;
}
