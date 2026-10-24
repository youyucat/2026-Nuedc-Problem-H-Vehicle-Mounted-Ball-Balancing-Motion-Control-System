#include "hal/hal.h"

#include <limits.h>
#include <stddef.h>

#include "ti_msp_dl_config.h"
#include "bsp/board_profile.h"
#include "config/template_config.h"
#define ENCODER_LEFT_MASK   (ENCODER_LEFT_A_PIN | ENCODER_LEFT_B_PIN)
#define ENCODER_RIGHT_MASK  (ENCODER_RIGHT_A_PIN | ENCODER_RIGHT_B_PIN)
#define ENCODER_MASK        (ENCODER_LEFT_MASK | ENCODER_RIGHT_MASK)

#define ADC_WAIT_TIMEOUT    100000U

/* 灰度异步采样的中断驱动实现（源工程注释里的 "DMA 后台采样" 说法已过时：
 * 实际路径是 ADC 的 MEM0_RESULT_LOADED 中断，见 ADC0_IRQHandler）。 */
#define GRAY_SAMPLES_PER_CHANNEL 8U
/* 平均样本数 = 每通道样本数减首样本（首样本吸收 MUX 切换残留，不参与
 * 累加，见 ADC0_IRQHandler）。必须与累加数一致，否则均值越出 12 位量程。 */
#define GRAY_AVERAGED_SAMPLES    (GRAY_SAMPLES_PER_CHANNEL - 1U)

_Static_assert(TEMPLATE_ADC_AVERAGE_SAMPLES > 0U,
               "ADC averaging requires at least one sample");
_Static_assert(TEMPLATE_SENSOR_CHANNELS == BOARD_PROFILE_SENSOR_CHANNELS,
               "Update board_profile.h when changing sensor channel count");

/*
 * MSPM0G3507 (Tianmengxing) implementation of the hal_time, hal_gray,
 * hal_motor and hal_encoder interfaces.
 *
 * 双芯架构的「循迹芯片」固件：本文件只服务灰度循迹 + TB6612 + 车轮编码器。
 * 源工程里挂在这个 GPIO 组中断上的步进 Z 相钩子（hal_stepper_z_service）
 * 已移除 —— 步进电机由另一颗芯片负责。
 */

static volatile uint32_t s_millis;
static volatile int32_t s_encoder_left_delta;
static volatile int32_t s_encoder_right_delta;
static volatile int32_t s_encoder_left_total;
static volatile int32_t s_encoder_right_total;
static volatile uint32_t s_encoder_left_invalid;
static volatile uint32_t s_encoder_right_invalid;
static volatile uint8_t s_encoder_left_previous;
static volatile uint8_t s_encoder_right_previous;

/* A is bit 1 and B is bit 0. Positive sequence:
 * 00 -> 01 -> 11 -> 10 -> 00. */
static const int8_t s_quadrature_step[16] = {
     0,  1, -1,  0,
    -1,  0,  0,  1,
     1,  0,  0, -1,
     0, -1,  1,  0
};

/* Address scan order for the installed sensor board. The MUX scan is reversed
 * and the board's physical harness has CH2/CH3 and CH6/CH7 crossed. Values are
 * remapped here so the host and the line controller both see the physical
 * left-to-right order CH1..CH8. */
static const uint8_t s_output_index[TEMPLATE_SENSOR_CHANNELS] =
    BOARD_SENSOR_OUTPUT_MAP;

static const int16_t s_positions[TEMPLATE_SENSOR_CHANNELS] =
    BOARD_SENSOR_POSITION_MAP;

void SysTick_Handler(void)
{
    ++s_millis;
}

uint32_t hal_time_millis(void)
{
    return s_millis;
}

void hal_time_delay_us(uint32_t microseconds)
{
    DL_Common_delayCycles(
        (TEMPLATE_BUS_CLOCK_HZ * microseconds) / 1000000U);
}

static uint8_t read_encoder_state(uint32_t pin_a, uint32_t pin_b)
{
    uint32_t pins = DL_GPIO_readPins(ENCODER_PORT, pin_a | pin_b);
    uint8_t state = 0U;

    if ((pins & pin_a) != 0U) state |= 2U;
    if ((pins & pin_b) != 0U) state |= 1U;
    return state;
}

static void add_saturated_i32(volatile int32_t *value, int8_t step)
{
    if ((step > 0) && (*value < INT32_MAX)) {
        ++(*value);
    } else if ((step < 0) && (*value > INT32_MIN)) {
        --(*value);
    }
}

static void update_encoder(volatile uint8_t *previous,
                           volatile int32_t *delta,
                           volatile int32_t *total,
                           volatile uint32_t *invalid,
                           uint8_t current,
                           bool inverted)
{
    uint8_t old = *previous;
    int8_t step = s_quadrature_step[(old << 2U) | current];

    if ((old != current) && (step == 0)) {
        if (*invalid != UINT32_MAX) ++(*invalid);
    } else if (step != 0) {
        if (inverted) step = (int8_t)-step;
        add_saturated_i32(delta, step);
        add_saturated_i32(total, step);
    }
    *previous = current;
}

void hal_encoder_init(void)
{
    DL_GPIO_initDigitalInputFeatures(ENCODER_LEFT_A_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(ENCODER_LEFT_B_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(ENCODER_RIGHT_A_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(ENCODER_RIGHT_B_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_setLowerPinsPolarity(ENCODER_PORT,
        DL_GPIO_PIN_6_EDGE_RISE_FALL |
        DL_GPIO_PIN_7_EDGE_RISE_FALL |
        DL_GPIO_PIN_8_EDGE_RISE_FALL |
        DL_GPIO_PIN_9_EDGE_RISE_FALL);

    s_encoder_left_delta = 0;
    s_encoder_right_delta = 0;
    s_encoder_left_total = 0;
    s_encoder_right_total = 0;
    s_encoder_left_invalid = 0U;
    s_encoder_right_invalid = 0U;
    s_encoder_left_previous = read_encoder_state(
        ENCODER_LEFT_A_PIN, ENCODER_LEFT_B_PIN);
    s_encoder_right_previous = read_encoder_state(
        ENCODER_RIGHT_A_PIN, ENCODER_RIGHT_B_PIN);

    DL_GPIO_clearInterruptStatus(ENCODER_PORT, ENCODER_MASK);
    DL_GPIO_enableInterrupt(ENCODER_PORT, ENCODER_MASK);
    NVIC_SetPriority(ENCODER_INT_IRQN, 1U);
    NVIC_ClearPendingIRQ(ENCODER_INT_IRQN);
    NVIC_EnableIRQ(ENCODER_INT_IRQN);
}

void hal_encoder_reset(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_encoder_left_delta = 0;
    s_encoder_right_delta = 0;
    s_encoder_left_total = 0;
    s_encoder_right_total = 0;
    s_encoder_left_invalid = 0U;
    s_encoder_right_invalid = 0U;
    s_encoder_left_previous = read_encoder_state(
        ENCODER_LEFT_A_PIN, ENCODER_LEFT_B_PIN);
    s_encoder_right_previous = read_encoder_state(
        ENCODER_RIGHT_A_PIN, ENCODER_RIGHT_B_PIN);
    DL_GPIO_clearInterruptStatus(ENCODER_PORT, ENCODER_MASK);
    __set_PRIMASK(primask);
}

void hal_encoder_snapshot(hal_encoder_snapshot_t *snapshot,
                          bool reset_delta)
{
    uint32_t primask;

    if (snapshot == NULL) return;
    primask = __get_PRIMASK();
    __disable_irq();
    snapshot->left_delta = s_encoder_left_delta;
    snapshot->right_delta = s_encoder_right_delta;
    snapshot->left_total = s_encoder_left_total;
    snapshot->right_total = s_encoder_right_total;
    snapshot->left_invalid_transitions = s_encoder_left_invalid;
    snapshot->right_invalid_transitions = s_encoder_right_invalid;
    snapshot->left_state = s_encoder_left_previous;
    snapshot->right_state = s_encoder_right_previous;
    if (reset_delta) {
        s_encoder_left_delta = 0;
        s_encoder_right_delta = 0;
    }
    __set_PRIMASK(primask);
}

void GROUP1_IRQHandler(void)
{
    uint32_t status = DL_GPIO_getEnabledInterruptStatus(
        ENCODER_PORT, ENCODER_MASK);

    if (status != 0U) {
        DL_GPIO_clearInterruptStatus(ENCODER_PORT, status);
    }

    if ((status & ENCODER_LEFT_MASK) != 0U) {
        update_encoder(&s_encoder_left_previous,
            &s_encoder_left_delta, &s_encoder_left_total,
            &s_encoder_left_invalid,
            read_encoder_state(ENCODER_LEFT_A_PIN, ENCODER_LEFT_B_PIN),
            TEMPLATE_LEFT_ENCODER_INVERTED != 0U);
    }
    if ((status & ENCODER_RIGHT_MASK) != 0U) {
        update_encoder(&s_encoder_right_previous,
            &s_encoder_right_delta, &s_encoder_right_total,
            &s_encoder_right_invalid,
            read_encoder_state(ENCODER_RIGHT_A_PIN, ENCODER_RIGHT_B_PIN),
            TEMPLATE_RIGHT_ENCODER_INVERTED != 0U);
    }
}

static void init_gray_gpio(void)
{
    DL_GPIO_initDigitalOutput(GRAY_AD0_IOMUX);
    DL_GPIO_initDigitalOutput(GRAY_AD1_IOMUX);
    DL_GPIO_initDigitalOutput(GRAY_AD2_IOMUX);
    DL_GPIO_initDigitalOutput(GRAY_EN_IOMUX);
    DL_GPIO_initDigitalInput(GRAY_ERR_IOMUX);
    DL_GPIO_clearPins(GRAY_AD0_PORT, GRAY_AD0_PIN);
    DL_GPIO_clearPins(GRAY_AD1_PORT, GRAY_AD1_PIN);
    DL_GPIO_clearPins(GRAY_AD2_PORT, GRAY_AD2_PIN);
    DL_GPIO_clearPins(GRAY_EN_PORT, GRAY_EN_PIN);
    DL_GPIO_enableOutput(GRAY_AD0_PORT, GRAY_AD0_PIN);
    DL_GPIO_enableOutput(GRAY_AD1_PORT, GRAY_AD1_PIN);
    DL_GPIO_enableOutput(GRAY_AD2_PORT, GRAY_AD2_PIN);
    DL_GPIO_enableOutput(GRAY_EN_PORT, GRAY_EN_PIN);
    hal_gray_enable(true);
}

static void init_gray_adc(void)
{
    /* SysConfig assigns PA27 to ADC0 channel 0. Configure the pin and the
     * converter explicitly so this board layer does not depend on ADC reset
     * defaults. A longer sample window is useful for the multiplexed sensor
     * output, whose source impedance is higher than a direct voltage source.
     * 采样走传统"单 MEM0 完成中断"模式：SysConfig 生成的 FIFO/DMA/DMA_DONE
     * 路径在实机上不可靠（无时无 DMA_DONE 中断、帧永不完成），这里显式
     * 关掉并改开 MEM0_RESULT_LOADED 中断 —— 每次转换完成即中断读值，
     * 与 32MHz 老固件同一套被长期验证的采集路径。 */
    DL_ADC12_disableConversions(GRAY_ADC_INST);
    DL_GPIO_initPeripheralAnalogFunction(GRAY_ADC_IOMUX);
    DL_ADC12_initSingleSample(GRAY_ADC_INST,
        DL_ADC12_REPEAT_MODE_ENABLED,
        DL_ADC12_SAMPLING_SOURCE_AUTO,
        DL_ADC12_TRIG_SRC_SOFTWARE,
        DL_ADC12_SAMP_CONV_RES_12_BIT,
        DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED);
    DL_ADC12_configConversionMem(GRAY_ADC_INST, GRAY_ADC_MEM,
        GRAY_ADC_INPUT,
        DL_ADC12_REFERENCE_VOLTAGE_VDDA,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0,
        DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED,
        DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_setSampleTime0(GRAY_ADC_INST,
                            TEMPLATE_ADC_SAMPLE_TIME_CYCLES);
    DL_ADC12_setPowerDownMode(GRAY_ADC_INST,
                              DL_ADC12_POWER_DOWN_MODE_MANUAL);
    /* 关闭 SysConfig 默认开启的 FIFO/DMA 路径（见上方注释） */
    DL_ADC12_disableFIFO(GRAY_ADC_INST);
    DL_ADC12_setDMASamplesCnt(GRAY_ADC_INST, 0U);
    DL_ADC12_disableDMA(GRAY_ADC_INST);
    DL_ADC12_disableDMATrigger(GRAY_ADC_INST,
        DL_ADC12_DMA_MEM10_RESULT_LOADED);
    DL_ADC12_clearInterruptStatus(GRAY_ADC_INST,
        DL_ADC12_INTERRUPT_DMA_DONE);
    DL_ADC12_disableInterrupt(GRAY_ADC_INST,
        DL_ADC12_INTERRUPT_DMA_DONE);
    /* 单 MEM0 完成中断（每次转换完成触发一次） */
    DL_ADC12_clearInterruptStatus(GRAY_ADC_INST,
        DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    DL_ADC12_enableInterrupt(GRAY_ADC_INST,
        DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    NVIC_EnableIRQ(ADC_0_INST_INT_IRQN);
}

static void init_motor_gpio(void)
{
    DL_GPIO_initDigitalOutput(MOTOR_AIN1_IOMUX);
    DL_GPIO_initDigitalOutput(MOTOR_AIN2_IOMUX);
    DL_GPIO_initDigitalOutput(MOTOR_BIN1_IOMUX);
    DL_GPIO_initDigitalOutput(MOTOR_BIN2_IOMUX);
    DL_GPIO_clearPins(MOTOR_DIRECTION_PORT, MOTOR_AIN1_PIN | MOTOR_AIN2_PIN |
        MOTOR_BIN1_PIN | MOTOR_BIN2_PIN);
    DL_GPIO_enableOutput(MOTOR_DIRECTION_PORT, MOTOR_AIN1_PIN | MOTOR_AIN2_PIN |
        MOTOR_BIN1_PIN | MOTOR_BIN2_PIN);
}

static void init_motor_pwm(void)
{
    DL_TimerA_ClockConfig clock_config = {
        .clockSel = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale = 0U
    };
    DL_TimerA_PWMConfig pwm_config = {
        .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN_UP,
        .period = TEMPLATE_PWM_PERIOD_COUNTS,
        .isTimerWithFourCC = false,
        .startTimer = DL_TIMER_START
    };

    DL_TimerA_reset(MOTOR_PWM_INST);
    DL_TimerA_enablePower(MOTOR_PWM_INST);
    DL_GPIO_initPeripheralOutputFunction(
        MOTOR_LEFT_PWM_IOMUX, MOTOR_LEFT_PWM_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        MOTOR_RIGHT_PWM_IOMUX, MOTOR_RIGHT_PWM_FUNC);
    DL_GPIO_enableOutput(MOTOR_PWM_PORT,
                         MOTOR_LEFT_PWM_PIN | MOTOR_RIGHT_PWM_PIN);

    DL_TimerA_setClockConfig(MOTOR_PWM_INST, &clock_config);
    DL_TimerA_initPWMMode(MOTOR_PWM_INST, &pwm_config);
    DL_TimerA_setCaptureCompareOutCtl(MOTOR_PWM_INST,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        DL_TIMERA_CAPTURE_COMPARE_0_INDEX);
    DL_TimerA_setCaptCompUpdateMethod(MOTOR_PWM_INST,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
        DL_TIMERA_CAPTURE_COMPARE_0_INDEX);
    DL_TimerA_setCaptureCompareOutCtl(MOTOR_PWM_INST,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        DL_TIMERA_CAPTURE_COMPARE_1_INDEX);
    DL_TimerA_setCaptCompUpdateMethod(MOTOR_PWM_INST,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
        DL_TIMERA_CAPTURE_COMPARE_1_INDEX);
    DL_TimerA_setCaptureCompareValue(
        MOTOR_PWM_INST, 0U, DL_TIMER_CC_0_INDEX);
    DL_TimerA_setCaptureCompareValue(
        MOTOR_PWM_INST, 0U, DL_TIMER_CC_1_INDEX);
    DL_TimerA_enableClock(MOTOR_PWM_INST);
    DL_TimerA_setCCPDirection(
        MOTOR_PWM_INST, DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT);
}

void hal_board_init(void)
{
    init_gray_gpio();
    init_gray_adc();
    init_motor_gpio();
    init_motor_pwm();
    hal_motor_standby(false);
    hal_motor_safe_inputs();
    hal_motor_set_pwm(0U, 0U);
    hal_encoder_init();
}

void hal_gray_enable(bool enabled)
{
    bool pin_high = (GRAY_ENABLE_ACTIVE_LOW != 0U) ? !enabled : enabled;
    if (pin_high) {
        DL_GPIO_setPins(GRAY_EN_PORT, GRAY_EN_PIN);
    } else {
        DL_GPIO_clearPins(GRAY_EN_PORT, GRAY_EN_PIN);
    }
}

static void set_address_bit(GPIO_Regs *port, uint32_t pin, bool bit_set)
{
    bool pin_high = (GRAY_ADDRESS_ACTIVE_LOW != 0U) ? !bit_set : bit_set;
    if (pin_high) {
        DL_GPIO_setPins(port, pin);
    } else {
        DL_GPIO_clearPins(port, pin);
    }
}

static void hal_gray_select_nowait(uint8_t address)
{
    set_address_bit(GRAY_AD0_PORT, GRAY_AD0_PIN,
                    (address & 0x01U) != 0U);
    set_address_bit(GRAY_AD1_PORT, GRAY_AD1_PIN,
                    (address & 0x02U) != 0U);
    set_address_bit(GRAY_AD2_PORT, GRAY_AD2_PIN,
                    (address & 0x04U) != 0U);
}

void hal_gray_select(uint8_t address)
{
    hal_gray_select_nowait(address);
    for (volatile uint32_t wait = 0U;
         wait < TEMPLATE_MUX_SETTLE_CYCLES; ++wait) {
    }
}

/* ---------- 灰传感器中断异步采样 ----------
 *
 * ADC0 runs in repeat mode; every completed conversion raises
 * MEM0_RESULT_LOADED and ADC0_IRQHandler reads one 12-bit value. Per channel
 * the first sample is discarded (absorbs the external MUX switching residue)
 * and the remaining 7 are summed; after 8 samples the average is stored and
 * the MUX advances to the next channel. A full 8-channel frame therefore
 * needs 64 conversion interrupts (~0.2 ms at ULPCLK 40 MHz). The control
 * loop only calls hal_gray_sampling_begin() (fire-and-forget) and
 * hal_gray_sampling_take() and never blocks on conversions. */

/* 一帧 8 路（每路 8 个样本）正常在 1~2 ms 内完成；超过该阈值视为异步
 * 链路卡死（IRQ 丢失），take() 会整帧重启一轮自愈。 */
#define GRAY_ASYNC_STALL_MS 20U

static volatile bool s_gray_busy;
static volatile uint8_t s_gray_channel;
static volatile bool s_gray_frame_ready;
static volatile bool s_gray_frame_error;
static volatile uint16_t s_gray_frame[TEMPLATE_SENSOR_CHANNELS];
static volatile uint32_t s_gray_start_ms;
/* 中断模式下当前通道的已采样数与累加和（首样本不计入）。 */
static volatile uint8_t s_gray_burst_count;
static volatile uint32_t s_gray_burst_sum;

void ADC0_IRQHandler(void)
{
    uint16_t sample;

    if (DL_ADC12_getPendingInterrupt(GRAY_ADC_INST) !=
        DL_ADC12_IIDX_MEM0_RESULT_LOADED) {
        return;
    }
    if (!s_gray_busy) return;

    sample = (uint16_t)DL_ADC12_getMemResult(GRAY_ADC_INST, GRAY_ADC_MEM);
    if (s_gray_burst_count > 0U) {
        s_gray_burst_sum += sample;
    }
    s_gray_burst_count++;
    if (s_gray_burst_count < GRAY_SAMPLES_PER_CHANNEL) {
        /* repeat 模式持续转换，等待下一个完成中断即可 */
        return;
    }

    s_gray_frame[s_gray_channel] =
        (uint16_t)((s_gray_burst_sum + GRAY_AVERAGED_SAMPLES / 2U) /
                   GRAY_AVERAGED_SAMPLES);
    s_gray_channel++;
    s_gray_burst_count = 0U;
    s_gray_burst_sum = 0U;

    if (s_gray_channel >= TEMPLATE_SENSOR_CHANNELS) {
        DL_ADC12_stopConversion(GRAY_ADC_INST);
        DL_ADC12_disableConversions(GRAY_ADC_INST);
        s_gray_busy = false;
        s_gray_frame_error =
            DL_GPIO_readPins(GRAY_ERR_PORT, GRAY_ERR_PIN) != 0U;
        s_gray_frame_ready = true;
        return;
    }

    hal_gray_select_nowait(s_gray_channel);
}

bool hal_gray_sampling_begin(void)
{
    if (s_gray_busy) return false;
    s_gray_busy = true;
    s_gray_channel = 0U;
    s_gray_burst_count = 0U;
    s_gray_burst_sum = 0U;
    s_gray_frame_ready = false;
    s_gray_start_ms = hal_time_millis();
    hal_gray_select_nowait(0U);
    DL_ADC12_clearInterruptStatus(GRAY_ADC_INST,
        DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    DL_ADC12_enableConversions(GRAY_ADC_INST);
    DL_ADC12_startConversion(GRAY_ADC_INST);
    return true;
}

bool hal_gray_sampling_take(uint16_t values[8], bool *error)
{
    bool ready;
    uint32_t state;

    if (values == NULL) return false;
    state = hal_system_critical_save();
    ready = s_gray_frame_ready;
    if (!ready && s_gray_busy &&
        (hal_time_millis() - s_gray_start_ms) > GRAY_ASYNC_STALL_MS) {
        /* 异步链路卡死（转换完成中断丢失）：critical 区内整帧重启一轮。 */
        s_gray_busy = false;
        s_gray_channel = 0U;
        s_gray_burst_count = 0U;
        s_gray_burst_sum = 0U;
        s_gray_frame_ready = false;
        hal_gray_select_nowait(0U);
        DL_ADC12_clearInterruptStatus(GRAY_ADC_INST,
            DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
        DL_ADC12_enableConversions(GRAY_ADC_INST);
        DL_ADC12_startConversion(GRAY_ADC_INST);
        s_gray_start_ms = hal_time_millis();
        s_gray_busy = true;
    }
    if (ready) {
        for (uint8_t i = 0U; i < TEMPLATE_SENSOR_CHANNELS; ++i) {
            values[i] = s_gray_frame[i];
        }
        if (error != NULL) *error = s_gray_frame_error;
        s_gray_frame_ready = false;
    }
    hal_system_critical_restore(state);
    return ready;
}

static bool gray_adc_convert_once(uint16_t *value)
{
    uint32_t timeout = ADC_WAIT_TIMEOUT;

    if (value == 0) return false;
    DL_ADC12_clearInterruptStatus(GRAY_ADC_INST,
        DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    DL_ADC12_enableConversions(GRAY_ADC_INST);
    DL_ADC12_startConversion(GRAY_ADC_INST);
    while ((DL_ADC12_getRawInterruptStatus(GRAY_ADC_INST,
                DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED) == 0U) &&
           (timeout != 0U)) {
        --timeout;
    }
    DL_ADC12_stopConversion(GRAY_ADC_INST);
    DL_ADC12_disableConversions(GRAY_ADC_INST);
    if (timeout == 0U) return false;
    *value = (uint16_t)DL_ADC12_getMemResult(
        GRAY_ADC_INST, GRAY_ADC_MEM);
    DL_ADC12_clearInterruptStatus(GRAY_ADC_INST,
        DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    return *value <= TEMPLATE_ADC_MAXIMUM;
}

bool hal_gray_read_adc(uint16_t *value)
{
    uint16_t sample;
    uint32_t sum = 0U;

    if (value == 0) return false;

    /* 同步兜底：仅在启动/诊断时使用单次转换（异步帧为常规路径）。 */
    if (!gray_adc_convert_once(&sample)) return false;
    for (uint8_t i = 0U; i < TEMPLATE_ADC_AVERAGE_SAMPLES; ++i) {
        if (!gray_adc_convert_once(&sample)) return false;
        sum += sample;
    }
    *value = (uint16_t)((sum + (TEMPLATE_ADC_AVERAGE_SAMPLES / 2U)) /
                        TEMPLATE_ADC_AVERAGE_SAMPLES);
    return true;
}

bool hal_gray_error_active(void)
{
    return DL_GPIO_readPins(GRAY_ERR_PORT, GRAY_ERR_PIN) != 0U;
}

uint8_t hal_gray_output_index(uint8_t address)
{
    return (address < TEMPLATE_SENSOR_CHANNELS) ?
        s_output_index[address] : 0U;
}

const int16_t *hal_gray_positions(void)
{
    return s_positions;
}

void hal_motor_standby(bool enabled)
{
    /* STBY is a hardware +5 V connection on this board. */
    (void)enabled;
}

void hal_motor_safe_inputs(void)
{
    DL_GPIO_clearPins(MOTOR_DIRECTION_PORT, MOTOR_AIN1_PIN | MOTOR_AIN2_PIN |
        MOTOR_BIN1_PIN | MOTOR_BIN2_PIN);
}

void hal_motor_set_left_direction(bool forward)
{
    if (forward) {
        DL_GPIO_setPins(MOTOR_DIRECTION_PORT, MOTOR_AIN1_PIN);
        DL_GPIO_clearPins(MOTOR_DIRECTION_PORT, MOTOR_AIN2_PIN);
    } else {
        DL_GPIO_clearPins(MOTOR_DIRECTION_PORT, MOTOR_AIN1_PIN);
        DL_GPIO_setPins(MOTOR_DIRECTION_PORT, MOTOR_AIN2_PIN);
    }
}

void hal_motor_set_right_direction(bool forward)
{
    if (forward) {
        DL_GPIO_setPins(MOTOR_DIRECTION_PORT, MOTOR_BIN1_PIN);
        DL_GPIO_clearPins(MOTOR_DIRECTION_PORT, MOTOR_BIN2_PIN);
    } else {
        DL_GPIO_clearPins(MOTOR_DIRECTION_PORT, MOTOR_BIN1_PIN);
        DL_GPIO_setPins(MOTOR_DIRECTION_PORT, MOTOR_BIN2_PIN);
    }
}

void hal_motor_set_pwm(uint16_t left_permille, uint16_t right_permille)
{
    uint32_t left;
    uint32_t right;

    if (left_permille > 1000U) left_permille = 1000U;
    if (right_permille > 1000U) right_permille = 1000U;
    left = ((uint32_t)left_permille * TEMPLATE_PWM_PERIOD_COUNTS) / 1000U;
    right = ((uint32_t)right_permille * TEMPLATE_PWM_PERIOD_COUNTS) / 1000U;
    DL_TimerA_setCaptureCompareValue(
        MOTOR_PWM_INST, left, DL_TIMER_CC_0_INDEX);
    DL_TimerA_setCaptureCompareValue(
        MOTOR_PWM_INST, right, DL_TIMER_CC_1_INDEX);
}
