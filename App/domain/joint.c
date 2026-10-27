/**
 * joint.c
 *
 * Step generation: the joint's timer runs in PWM mode on the STEP pin, one
 * PWM period per step (50% duty). Joint_Update() produces a desired step
 * FREQUENCY (steps/sec) + direction and writes it into the timer's
 * auto-reload (period) and compare registers. No interrupt per step.
 *
 * Expected CubeMX settings for every step timer (verify in tim.c):
 *   - the STEP channel set to "PWM Generation CHx", PWM mode 1, polarity high
 *   - "Auto-reload preload": Enable   (period changes apply at the next
 *                                      period boundary, no glitched pulse)
 *   - prescaler chosen so the count rate is step_timer_clk_hz in the config
 * The period (ARR) set in CubeMX does not matter: this code overwrites it.
 */

#include "joint.h"
#include <math.h>

#define MAX_STEP_FREQ_HZ      50000.0f  // sanity clamp, raise once tuned
#define MIN_MOVE_DEG          0.01f     // deadband to avoid dithering at rest
#define FULL_STEPS_PER_REV    200.0f    // 1.8 degree motor (NEMA17)
#define STEP_ARR_MAX          65535.0f  // 16-bit limit; fits every timer, incl. the 32-bit ones
#define MAX_INTEGRAL_TERM_DPS 30.0f     // anti-windup: cap on the I term's velocity contribution

static uint8_t current_to_31(float fraction)
{
    if (fraction < 0.0f) fraction = 0.0f;
    if (fraction > 1.0f) fraction = 1.0f;
    return (uint8_t)(fraction * 31.0f + 0.5f);
}

static void joint_stop_steps(Joint_t *joint)
{
    if (joint->step_running) {
        HAL_TIM_PWM_Stop(joint->cfg.step_timer, joint->cfg.step_channel);
        joint->step_running = false;
    }
}

bool Joint_Init(Joint_t *joint, const Joint_Config_t *cfg)
{
    joint->cfg = *cfg;
    joint->target_deg = 0.0f;
    joint->current_deg = 0.0f;
    joint->current_vel_dps = 0.0f;
    joint->integral = 0.0f;
    joint->prev_error = 0.0f;
    joint->fault = false;
    joint->ready = false;
    joint->step_running = false;

    /* Make sure no pulses are running, then pick a known direction. */
    HAL_TIM_PWM_Stop(cfg->step_timer, cfg->step_channel);
    HAL_GPIO_WritePin(cfg->dir_port, cfg->dir_pin, GPIO_PIN_RESET);

    /* Configure the driver. Every write is confirmed via IFCNT, and an
     * unsupported microstep value is rejected instead of silently replaced. */
    tmc_dev_init(&joint->tmc, cfg->uart, cfg->tmc_address);
    tmc_cfg_t tc = tmc_cfg_default();
    tc.microsteps   = cfg->microsteps;
    tc.run_current  = current_to_31(cfg->run_current);
    tc.hold_current = current_to_31(cfg->hold_current);

    joint->tmc_status = tmc_configure(&joint->tmc, &tc);
    if (joint->tmc_status != TMC_OK) {
        joint->fault = true;
        return false;
    }

    if (cfg->open_loop) {          /* bring-up mode: no encoder involved */
        joint->ready = true;
        return true;
    }

    AS5047P_Init(&joint->encoder, cfg->spi, cfg->encoder_cs_port, cfg->encoder_cs_pin);

    /* Prime current_deg with a real encoder reading so the first PID
     * tick doesn't see a huge bogus error from a stale 0.0. */
    float initial_deg;
    if (!AS5047P_ReadAngleDeg(&joint->encoder, &initial_deg)) {
        joint->fault = true;
        return false;
    }
    joint->current_deg = initial_deg;
    joint->target_deg = initial_deg; // don't leap to 0 on boot

    joint->ready = true;
    return true;
}

void Joint_SetTargetDeg(Joint_t *joint, float target_deg)
{
    joint->target_deg = target_deg;
}

/* One PWM period = one step, so step_hz = timer_clk / (ARR + 1). */
static void joint_set_step_frequency(Joint_t *joint, float freq_hz, bool positive_dir)
{
    TIM_HandleTypeDef *tim = joint->cfg.step_timer;
    uint32_t ch = joint->cfg.step_channel;

    if (freq_hz > MAX_STEP_FREQ_HZ) {
        freq_hz = MAX_STEP_FREQ_HZ;
    }

    float arr_f = (freq_hz >= 1.0f) ? (joint->cfg.step_timer_clk_hz / freq_hz - 1.0f) : STEP_ARR_MAX + 1.0f;

    /* Too slow for this timer's range (or ~zero): stop. Pick the prescaler
     * so this limit is below any speed you care about. */
    if (arr_f > STEP_ARR_MAX) {
        joint_stop_steps(joint);
        return;
    }
    if (arr_f < 3.0f) {            /* keep a few counts for a clean 50% pulse */
        arr_f = 3.0f;
    }

    uint32_t arr = (uint32_t)(arr_f + 0.5f);
    uint32_t cmp = (arr + 1u) / 2u;   /* 50% duty */

    HAL_GPIO_WritePin(joint->cfg.dir_port, joint->cfg.dir_pin,
                      positive_dir ? GPIO_PIN_SET : GPIO_PIN_RESET);

    if (!joint->step_running) {
        /* Starting from stopped: load fresh values and force them in. */
        __HAL_TIM_SET_AUTORELOAD(tim, arr);
        __HAL_TIM_SET_COMPARE(tim, ch, cmp);
        __HAL_TIM_SET_COUNTER(tim, 0);
        HAL_TIM_GenerateEvent(tim, TIM_EVENTSOURCE_UPDATE);
        HAL_TIM_PWM_Start(tim, ch);
        joint->step_running = true;
    } else {
        /* Already running: change only the period and compare. With
         * auto-reload preload on, they take effect at the next period
         * boundary. Never touch the counter here: doing it every control
         * tick would stop slow step rates from ever completing a period. */
        __HAL_TIM_SET_AUTORELOAD(tim, arr);
        __HAL_TIM_SET_COMPARE(tim, ch, cmp);
    }
}

void Joint_RunOpenLoop(Joint_t *joint, float step_hz, bool positive_dir)
{
    if (!joint->ready || !joint->cfg.open_loop) {
        return;
    }
    joint_set_step_frequency(joint, step_hz, positive_dir);
}

void Joint_Update(Joint_t *joint, float dt)
{
    if (!joint->ready || joint->cfg.open_loop) {
        return;
    }

    float measured_deg;
    if (!AS5047P_ReadAngleDeg(&joint->encoder, &measured_deg)) {
        joint->fault = true;
        joint_stop_steps(joint);   // stop moving on sensor loss
        return;
    }

    /* Handle 0/360 wraparound so velocity/error don't spike near the seam. */
    float delta = measured_deg - joint->current_deg;
    if (delta > 180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;

    joint->current_vel_dps = (dt > 0.0f) ? (delta / dt) : 0.0f;
    joint->current_deg += delta;

    float error = joint->target_deg - joint->current_deg;

    if (fabsf(error) < MIN_MOVE_DEG) {
        joint_stop_steps(joint);
        joint->integral = 0.0f; // avoid windup while parked
        joint->prev_error = error;
        return;
    }

    joint->integral += error * dt;
    if (joint->cfg.ki > 0.0f) {                       // anti-windup clamp
        float limit = MAX_INTEGRAL_TERM_DPS / joint->cfg.ki;
        if (joint->integral >  limit) joint->integral =  limit;
        if (joint->integral < -limit) joint->integral = -limit;
    }
    float derivative = (dt > 0.0f) ? ((error - joint->prev_error) / dt) : 0.0f;
    joint->prev_error = error;

    /* PID output is a velocity command in deg/sec (this is a
     * position->velocity controller, common for stepper position loops). */
    float vel_dps = joint->cfg.kp * error + joint->cfg.ki * joint->integral
                   + joint->cfg.kd * derivative;

    bool positive_dir = vel_dps >= 0.0f;
    float vel_abs_dps = fabsf(vel_dps);

    /* Convert output-shaft deg/sec -> motor step frequency:
     *   motor_rev/sec = (vel_dps/360) * gear_ratio
     *   steps/sec     = motor_rev/sec * microsteps * full steps per rev */
    float motor_rev_per_sec = (vel_abs_dps / 360.0f) * joint->cfg.gear_ratio;
    float step_freq_hz = motor_rev_per_sec * (float)joint->cfg.microsteps * FULL_STEPS_PER_REV;

    joint_set_step_frequency(joint, step_freq_hz, positive_dir);
}

float Joint_GetPositionDeg(const Joint_t *joint) { return joint->current_deg; }
float Joint_GetVelocityDps(const Joint_t *joint) { return joint->current_vel_dps; }
bool Joint_HasFault(const Joint_t *joint) { return joint->fault; }
tmc_status_t Joint_GetTmcStatus(const Joint_t *joint) { return joint->tmc_status; }