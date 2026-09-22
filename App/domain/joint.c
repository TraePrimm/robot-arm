/**
 * joint.c
 *
 * Step generation approach: Joint_Update() runs the position PID and
 * produces a desired step FREQUENCY (steps/sec) + direction. It writes
 * that frequency into the joint's hardware timer's auto-reload register,
 * so the timer's period-elapsed interrupt fires at exactly that rate.
 * Joint_StepISR() (called from that interrupt) just toggles the STEP pin -
 * it does no math, so it's safe to call from an ISR at high frequency.
 *
 * TODO before this compiles for real: set STEP_TIMER_CLK_HZ to your
 * actual timer input clock divided by whatever prescaler you configured
 * in CubeMX for each step_timer (e.g. if APB timer clock is 200MHz and
 * you set PSC=199, STEP_TIMER_CLK_HZ = 1,000,000).
 */

#include "joint.h"
#include <math.h>

#define STEP_TIMER_CLK_HZ   1000000.0f  // TODO: match your CubeMX PSC setting
#define MAX_STEP_FREQ_HZ     50000.0f   // sanity clamp, raise once tuned
#define MIN_MOVE_DEG          0.01f     // deadband to avoid dithering at rest

bool Joint_Init(Joint_t *joint, const Joint_Config_t *cfg)
{
    joint->cfg = *cfg;
    joint->target_deg = 0.0f;
    joint->current_deg = 0.0f;
    joint->current_vel_dps = 0.0f;
    joint->integral = 0.0f;
    joint->prev_error = 0.0f;
    joint->fault = false;

    HAL_GPIO_WritePin(cfg->dir_port, cfg->dir_pin, GPIO_PIN_RESET);

    TMC2209_Init(&joint->motor, cfg->uart, cfg->tmc_address);
    AS5047P_Init(&joint->encoder, cfg->spi, cfg->encoder_cs_port, cfg->encoder_cs_pin);

    if (!TMC2209_IsConnected(&joint->motor)) {
        joint->fault = true;
        return false;
    }
    if (!TMC2209_Configure(&joint->motor, cfg->microsteps)) {
        joint->fault = true;
        return false;
    }
    uint8_t run_31  = (uint8_t)(cfg->run_current * 31.0f);
    uint8_t hold_31 = (uint8_t)(cfg->hold_current * 31.0f);
    if (!TMC2209_SetRunCurrent(&joint->motor, run_31, hold_31)) {
        joint->fault = true;
        return false;
    }

    /* Prime current_deg with a real encoder reading so the first PID
     * tick doesn't see a huge bogus error from a stale 0.0. */
    float initial_deg;
    if (AS5047P_ReadAngleDeg(&joint->encoder, &initial_deg)) {
        joint->current_deg = initial_deg;
        joint->target_deg = initial_deg; // don't leap to 0 on boot
    } else {
        joint->fault = true;
        return false;
    }

    /* Step timer starts stopped - Joint_Update enables/sets its rate. */
    HAL_TIM_Base_Stop_IT(cfg->step_timer);

    return true;
}

void Joint_SetTargetDeg(Joint_t *joint, float target_deg)
{
    joint->target_deg = target_deg;
}

static void joint_set_step_frequency(Joint_t *joint, float freq_hz, bool positive_dir)
{
    TIM_HandleTypeDef *tim = joint->cfg.step_timer;

    if (freq_hz < 1.0f) {
        HAL_TIM_Base_Stop_IT(tim);
        return;
    }
    if (freq_hz > MAX_STEP_FREQ_HZ) {
        freq_hz = MAX_STEP_FREQ_HZ;
    }

    /* We toggle the pin once per interrupt, so a full step needs TWO
     * interrupts (rise+fall) -> interrupt rate = 2x step_hz. */
    uint32_t arr = (uint32_t)(STEP_TIMER_CLK_HZ / (2.0f * freq_hz)) - 1;
    __HAL_TIM_SET_AUTORELOAD(tim, arr);
    __HAL_TIM_SET_COUNTER(tim, 0);

    HAL_GPIO_WritePin(joint->cfg.dir_port, joint->cfg.dir_pin,
                       positive_dir ? GPIO_PIN_SET : GPIO_PIN_RESET);

    HAL_TIM_Base_Start_IT(tim);
}

void Joint_Update(Joint_t *joint, float dt)
{
    float measured_deg;
    if (!AS5047P_ReadAngleDeg(&joint->encoder, &measured_deg)) {
        joint->fault = true;
        joint_set_step_frequency(joint, 0.0f, true); // stop moving on sensor loss
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
        joint_set_step_frequency(joint, 0.0f, true);
        joint->integral = 0.0f; // avoid windup while parked
        joint->prev_error = error;
        return;
    }

    joint->integral += error * dt;
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
     *   steps/sec     = motor_rev/sec * microsteps * 200 (full steps/rev, NEMA17) */
    float motor_rev_per_sec = (vel_abs_dps / 360.0f) * joint->cfg.gear_ratio;
    float step_freq_hz = motor_rev_per_sec * (float)joint->cfg.microsteps * 200.0f;

    joint_set_step_frequency(joint, step_freq_hz, positive_dir);
}

float Joint_GetPositionDeg(const Joint_t *joint) { return joint->current_deg; }
float Joint_GetVelocityDps(const Joint_t *joint) { return joint->current_vel_dps; }
bool Joint_HasFault(const Joint_t *joint) { return joint->fault; }

void Joint_StepISR(Joint_t *joint)
{
    HAL_GPIO_TogglePin(joint->cfg.step_port, joint->cfg.step_pin);
}