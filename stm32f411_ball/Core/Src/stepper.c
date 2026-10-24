/**
  ******************************************************************************
  * @file    stepper.c
  * @brief   步进电机脉冲发生 + 梯形加减速
  *
  * 原理：
  *   TIM1_CH1 输出固定脉宽的脉冲当 STEP，改 ARR 就是改频率（改速度）。
  *   TIM1 更新中断每发完一个脉冲触发一次，用来计数、判断是否该减速。
  *   主循环每 1ms 调 stepper_tick_1ms()，按加速度把当前速度往目标速度靠。
  *
  * 本文件属于用户代码，CubeMX 重新生成工程时不会被覆盖。
  ******************************************************************************
  */

#include "stepper.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;

/* ===================== 内部状态 ===================== */
static volatile int32_t  s_pos         = 0;    /* 已输出脉冲累计位置 */
static volatile int32_t  s_left        = 0;    /* 定长移动：剩余脉冲 */
static volatile int32_t  s_move_total  = 0;    /* 定长移动：本次总脉冲 */
static volatile uint32_t s_cur_sps     = 0;    /* 当前速度(脉冲/秒) */
static volatile uint32_t s_run_sps     = STEPPER_RUN_SPEED;
static volatile uint8_t  s_run_req     = 0;    /* 1 = 要求运行 */
static volatile uint8_t  s_move_active = 0;    /* 1 = 定长移动进行中 */
static volatile uint8_t  s_pulse_on    = 0;    /* 1 = 正在输出脉冲 */
static volatile uint8_t  s_decel_latch = 0;    /* 1 = 已进入减速段（锁存） */
static volatile int8_t   s_pend_dir    = -1;   /* >=0 表示有一个待切换的方向 */

static volatile stepper_dir_t s_dir    = STEP_DIR_CW;  /* 当前 DIR 电平对应方向 */
static stepper_dir_t     s_run_dir     = STEP_DIR_CW;  /* 期望运行方向 */
static uint8_t           s_enabled     = 0;

/* TIM1 实际计数频率（初始化时按真实时钟算出来，换主频不用改代码） */
static uint32_t          s_tick_hz     = STEPPER_TIM_TICK_HZ;

/* ===================== 底层硬件操作 ===================== */

/** 取 TIM1 的定时器时钟频率（APB2，预分频不为 1 时要乘 2） */
static uint32_t stepper_tim1_clock_hz(void)
{
    uint32_t clk = HAL_RCC_GetPCLK2Freq();

    if ((RCC->CFGR & RCC_CFGR_PPRE2) != 0U) {
        clk *= 2U;
    }
    return clk;
}

/** 设定频率（脉冲/秒）：只改 ARR，CCR（脉宽）全程不动 */
static void stepper_hw_set_speed(uint32_t sps)
{
    uint32_t arr;

    if (sps < 1U) sps = 1U;

    /* f = s_tick_hz / (ARR+1)  =>  ARR = s_tick_hz / f - 1 */
    arr = s_tick_hz / sps;

    /* ARR 必须大于 CCR(固定脉宽)，否则输出一直高电平，脉冲就丢了 */
    if (arr < (STEPPER_PULSE_TICKS + 3U)) arr = STEPPER_PULSE_TICKS + 3U;
    if (arr > 65536U) arr = 65536U;
    arr -= 1U;

    __HAL_TIM_SET_AUTORELOAD(&htim1, arr);

    /* "PWM" 脚输出：占空比跟随速度，0~100% */
    stepper_set_pwm_ref((uint16_t)(((uint64_t)sps * 1000ULL) / STEPPER_SPS_MAX));
}

/** 打开 / 关闭 STEP 脉冲输出
 *
 *  关的时候不去改 CCR（CCR 有预装载，要等下一个周期才生效，会多冒一个脉冲，
 *  位置就会一点点漂），而是直接把 CNT 顶到 ARR —— CNT >= CCR 时 PWM1 输出恒为低，
 *  立刻见效；然后把计数器停掉，CNT 就冻在那儿，绝不再产生边沿。
 */
static void stepper_pulse_output(uint8_t on)
{
    if (on) {
        /* 手动产生一次更新事件，把刚写的 ARR 立刻刷进影子寄存器，
         * 保证第一个脉冲的周期就是想要的频率 */
        htim1.Instance->EGR = TIM_EGR_UG;
        __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
        __HAL_TIM_SET_COUNTER(&htim1, 0U);          /* CNT < CCR -> 输出高 */
        __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
        __HAL_TIM_ENABLE(&htim1);                   /* 开始计数 */
        s_pulse_on = 1U;
    } else {
        __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);
        /* 注意：不能用 __HAL_TIM_DISABLE() —— HAL 里这个宏发现有 PWM 通道
         * 使能（CC1E=1）时是空操作，不会清 CEN。结果就是"停脉冲"以后定时器
         * 还在跑、PA8 一直输出 PWM，电机停不下来（上电就匀速正转的根因）。
         * 这里必须直接清 CEN：CNT 冻在 ARR 上，PWM1 输出恒为低，不再有边沿。 */
        htim1.Instance->CR1 &= ~TIM_CR1_CEN;
        __HAL_TIM_SET_COUNTER(&htim1, __HAL_TIM_GET_AUTORELOAD(&htim1));  /* CNT >= CCR -> 低 */
        __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
        s_pulse_on = 0U;
        stepper_set_pwm_ref(0U);
    }
}

/** 设置 DIR 引脚 */
static void stepper_apply_dir(stepper_dir_t dir)
{
    uint8_t lvl = (dir == STEP_DIR_CW) ? 0U : 1U;

#if STEPPER_DIR_INVERT
    lvl = (uint8_t)(!lvl);
#endif

    s_dir = dir;
    HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, lvl ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ===================== 对外接口 ===================== */

void stepper_init(void)
{
    s_pos = 0; s_left = 0; s_move_total = 0;
    s_cur_sps = 0; s_run_req = 0; s_move_active = 0;
    s_pend_dir = -1; s_pulse_on = 0; s_enabled = 0; s_decel_latch = 0;
    s_dir = STEP_DIR_CW; s_run_dir = STEP_DIR_CW;

    /* 按真实时钟和 PSC 反算计数频率，主频变了也不用改宏。
       注意：F4 的 HAL 没有 __HAL_TIM_GET_PRESCALER（只有 SET 版），
       这里直接读 HAL 句柄里缓存的 PSC。调速只改 ARR、PSC 全程恒定，所以可靠。 */
    s_tick_hz = stepper_tim1_clock_hz() / (htim1.Init.Prescaler + 1U);
    if (s_tick_hz == 0U) s_tick_hz = STEPPER_TIM_TICK_HZ;

    /* 兜底：万一 CubeMX 里的 Pulse 没配过来，这里强制设成固定脉宽 */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, STEPPER_PULSE_TICKS);

    stepper_apply_dir(STEP_DIR_CW);
    stepper_enable(0);

    /* 先把 STEP 拉低再让计数器跑，避免上电瞬间蹦一个脉冲出去 */
    stepper_hw_set_speed(STEPPER_RUN_SPEED);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    stepper_pulse_output(0);

    /* PWM 参考输出 */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    stepper_set_pwm_ref(0U);
}

void stepper_enable(uint8_t en)
{
    s_enabled = en ? 1U : 0U;

#if STEPPER_EN_ACTIVE_LOW
    HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, en ? GPIO_PIN_RESET : GPIO_PIN_SET);
#else
    HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, en ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
}

uint8_t stepper_is_enabled(void) { return s_enabled; }

void stepper_set_run_speed(uint32_t sps)
{
    if (sps < STEPPER_SPS_MIN) sps = STEPPER_SPS_MIN;
    if (sps > STEPPER_SPS_MAX) sps = STEPPER_SPS_MAX;
    s_run_sps = sps;
}

uint32_t stepper_get_run_speed(void) { return s_run_sps; }

void stepper_run(stepper_dir_t dir)
{
    if (!s_enabled) return;

    s_move_active = 0;
    s_left        = 0;
    s_move_total  = 0;
    s_run_dir     = dir;
    s_run_req     = 1;
    s_decel_latch = 0;

    /* 正在动而且方向不一致 -> 先减速停下再换向 */
    if (s_pulse_on && (dir != s_dir)) {
        s_pend_dir = (int8_t)dir;
    }
}

void stepper_stop(void)
{
    s_run_req     = 0;
    s_move_active = 0;
    s_left        = 0;
    s_move_total  = 0;
    s_pend_dir    = -1;
    s_decel_latch = 0;
}

void stepper_estop(void)
{
    s_run_req = 0; s_move_active = 0; s_left = 0;
    s_cur_sps = 0; s_pend_dir = -1; s_decel_latch = 0;
    stepper_pulse_output(0);
    stepper_enable(0);
}

void stepper_move(int32_t steps)
{
    if (!s_enabled || steps == 0) return;

    s_run_dir     = (steps > 0) ? STEP_DIR_CW : STEP_DIR_CCW;
    s_move_total  = (steps > 0) ? steps : -steps;
    s_move_active = 1;
    s_run_req     = 1;
    s_decel_latch = 0;

    if (s_pulse_on || s_cur_sps > 0U) {
        /* 还在动：先减速停下，换好方向、重新装载步数再出发 */
        s_pend_dir = (int8_t)s_run_dir;
        s_left     = 0;
    } else {
        s_pend_dir = -1;
        s_left     = s_move_total;
    }
}

void stepper_goto(int32_t pos)
{
    int32_t delta = pos - s_pos;
    if (delta != 0) stepper_move(delta);
}

void stepper_servo_update(int32_t target)
{
    int32_t delta;

    if (s_move_active || s_cur_sps > 0U) {
        return;                     /* 上一段微动还在走：走完再补差，不叠加 */
    }

    delta = target - s_pos;
    if (delta > (int32_t)STEPPER_SERVO_DEADBAND ||
        delta < -(int32_t)STEPPER_SERVO_DEADBAND) {
        stepper_goto(target);       /* 死区内不动，防抖 */
    }
}

int32_t       stepper_position(void)       { return s_pos; }
uint32_t      stepper_current_speed(void)  { return s_cur_sps; }
stepper_dir_t stepper_direction(void)      { return s_dir; }
uint8_t       stepper_is_move_active(void) { return s_move_active; }
uint8_t       stepper_is_busy(void)        { return (s_pulse_on || s_cur_sps > 0U) ? 1U : 0U; }
int32_t       stepper_steps_left(void)    { return s_left; }

void stepper_set_pwm_ref(uint16_t duty_permille)
{
    uint32_t arr;

    if (duty_permille > 1000U) duty_permille = 1000U;

    arr = __HAL_TIM_GET_AUTORELOAD(&htim4) + 1U;
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, (arr * duty_permille) / 1000U);
}

/* ===================== 1ms 周期任务：速度斜坡 + 状态机 ===================== */

void stepper_tick_1ms(void)
{
    uint32_t dv = STEPPER_ACCEL / 1000U;    /* 每 1ms 的速度变化量 */
    uint32_t target;

    if (dv < 1U) dv = 1U;

    /* ---------- 1. 速度为 0：处理状态机（停车 / 换向 / 走完 / 起步） ----------
     * 这段必须放在速度斜坡【之前】：否则从静止起步时，速度先被斜坡加到 dv，
     * cur_sps 再也不等于 0，下面的起步分支永远进不去 -> 脉冲一直不输出，
     * 长行程时电机会一步都不走（按键无反应）。 */
    if (s_cur_sps == 0U) {
        if (s_pulse_on) stepper_pulse_output(0);

        /* 1.1 用户要求停 */
        if (!s_run_req) {
            s_move_active = 0;
            s_left        = 0;
            s_pend_dir    = -1;
            s_decel_latch = 0;
#if STEPPER_DISABLE_WHEN_IDLE
            if (s_enabled) stepper_enable(0);
#endif
            return;
        }

        /* 1.2 换向：设定 DIR 后本 tick 直接返回，留出 1ms 给驱动器建立方向 */
        if (s_pend_dir >= 0) {
            stepper_apply_dir((stepper_dir_t)s_pend_dir);
            s_pend_dir = -1;
            if (s_move_active) s_left = s_move_total;
            s_decel_latch = 0;
            return;
        }

        /* 1.3 定长移动走完了 */
        if (s_move_active && s_left <= 0) {
            s_move_active = 0;
            s_run_req     = 0;
            s_decel_latch = 0;
            return;
        }

        /* 1.4 起步：速度为 0 且还有活 -> 立刻开脉冲，别等斜坡
         * 起步瞬时速度用 STEPPER_SPS_START：微动定位（伺服跟踪）时一步
         * 只有几个脉冲，60 脉冲/秒的起步速度会让每个小段拖 100ms，
         * 闭环根本跟不上；300 脉冲/秒起步又快又稳。 */
        s_decel_latch = 0;
#if STEPPER_DISABLE_WHEN_IDLE
        if (!s_enabled) stepper_enable(1);
#endif
        stepper_apply_dir(s_run_dir);
        s_cur_sps = (s_run_sps < STEPPER_SPS_START) ? s_run_sps : STEPPER_SPS_START;
        stepper_hw_set_speed(s_cur_sps);
        stepper_pulse_output(1);
        return;                     /* 第一个脉冲周期用起步速度跑满 1ms */
    }

    /* ---------- 2. 目标速度 ---------- */
    if (!s_run_req) {
        target = 0U;                                     /* 要停 */
    } else if (s_pend_dir >= 0) {
        target = 0U;                                     /* 有换向/重启请求：先减速 */
    } else if (s_move_active) {
        /* 定长移动：剩余路程刚好够减速时就该松油门了。
         * 减速标志必须锁存，否则速度降下来后 decel 变小、判断会翻转，速度来回抖。 */
        if (!s_decel_latch) {
            uint32_t decel = (uint32_t)(((uint64_t)s_cur_sps * s_cur_sps) / (2ULL * STEPPER_ACCEL));
            if ((uint32_t)s_left <= decel) {
                s_decel_latch = 1;
            }
        }
        target = s_decel_latch ? 0U : s_run_sps;
    } else {
        target = s_run_sps;
    }

    /* ---------- 3. 速度斜坡 ---------- */
    if (s_cur_sps < target) {
        s_cur_sps += dv;
        if (s_cur_sps > target) s_cur_sps = target;
    } else if (s_cur_sps > target) {
        s_cur_sps = (s_cur_sps > dv) ? (s_cur_sps - dv) : 0U;
    }

    /* ---------- 4. 本 tick 刚好减到 0：立刻封脉冲（收尾），下一 tick 由第 1 段状态机接管 ---------- */
    if (s_cur_sps == 0U) {
        if (s_pulse_on) stepper_pulse_output(0);
        return;
    }

    /* ---------- 5. 运行中：刷新 STEP 脉冲频率 ---------- */
    stepper_hw_set_speed(s_cur_sps);
}

/* ===================== 脉冲中断：每发完一个脉冲进来一次 ===================== */

void stepper_pulse_isr(void)
{
    s_pos += (s_dir == STEP_DIR_CW) ? 1 : -1;

    if (s_move_active && s_left > 0) {
        s_left--;

        /* 走到目标了：立刻封脉冲，保证不多走一步 */
        if (s_left == 0) {
            stepper_pulse_output(0);
            s_cur_sps     = 0U;
            s_run_req     = 0U;
            s_move_active = 0U;
            s_decel_latch = 0U;
        }
    }
}
