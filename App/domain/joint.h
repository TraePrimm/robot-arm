
/**
 * joint.h
 *
 * One joint = one TMC2209 + one AS5047P + a gear ratio + a position PID
 * loop that turns "target angle" into STEP pulses.
 *
 * Everything OUTSIDE this file talks in output-shaft degrees. Gear ratio
 * and microstep math are private to this class - RobotArm and RosInterface
 * never need to know a joint is geared 24:1 vs 1:1.
 *
 * The encoder is assumed to measure the OUTPUT shaft. For a bench test with
 * the magnet on the bare motor shaft, set gear_ratio = 1.0 for that joint.
 *
 * STEP generation: the joint's timer runs in PWM mode on the STEP pin
 * (the pin is a timer alternate-function pin, per the pinout table). One PWM
 * period = one step. Joint_Update() only changes the timer's period, so the
 * hardware produces the pulses with no interrupt per step.
 */
#ifndef JOINT_H
#define JOINT_H
 
#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"
#include "tmc2209_cfg.h"
#include "encoder.h"
 
typedef struct {
    /* --- wiring / hardware, set once at startup --- */
    UART_HandleTypeDef *uart;      // TMC2209 UART bus
    uint8_t tmc_address;           // 0-3 on that bus
    SPI_HandleTypeDef *spi;        // AS5047P SPI bus
    GPIO_TypeDef *encoder_cs_port;
    uint16_t encoder_cs_pin;
    uint8_t encoder_id;            // 0-5, unique per encoder (use the joint index)
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
 
    TIM_HandleTypeDef *step_timer; // timer whose PWM channel drives STEP
    uint32_t step_channel;         // TIM_CHANNEL_1 / _2 / _3 ...
    float step_timer_clk_hz;       // timer COUNT rate = timer kernel clock / (PSC+1)
 
    /* --- joint-specific constants --- */
    float gear_ratio;      // motor revolutions per output-shaft revolution
    uint16_t microsteps;   // 1,2,4,...,256 (power of two)
    float run_current;     // 0.0-1.0, fraction of driver's full-scale current
    float hold_current;    // 0.0-1.0
    int8_t encoder_sign;   // +1 or -1: makes "positive DIR" = "angle increases"
    float encoder_offset_deg; // added to the reading (after sign) to set joint zero
 
    /* --- PID gains (tune per-joint once assembled) --- */
    float kp, ki, kd;
 
    /* --- closed-loop motion limits (0 = use the defaults in joint.c) --- */
    float max_step_hz;     // speed cap in STEP pulses/sec (motor side)
    float accel_hz_per_s;  // ramp: how fast the step rate may change
 
    /* --- bring-up --- */
    bool open_loop;        // true: driver only, no PID. Drive with Joint_RunOpenLoop().
    bool encoder_present;  // true: init + read the encoder (also in open-loop mode)
} Joint_Config_t;
 
typedef struct {
    Joint_Config_t cfg;
    tmc_dev_t tmc;
    Encoder_t encoder;
 
    float target_deg;      // desired output-shaft position
    float current_deg;     // last measured output-shaft position (unwrapped)
    float current_vel_dps; // degrees/sec, estimated from consecutive readings
 
    /* PID internal state */
    float integral;
    float prev_error;
 
    float cmd_hz;            // signed step rate actually applied (ramped)
    uint8_t enc_fail_streak; // consecutive failed encoder reads
    bool encoder_ok;         // encoder initialized and reading
    bool ready;              // true only after a fully successful Joint_Init
    bool step_running;       // PWM currently running on the STEP pin
    bool fault;              // latched: driver/encoder failure, joint stays stopped
    tmc_status_t tmc_status; // result of the driver configure at init (debugging)
} Joint_t;
 
/* Configures the TMC2209 (verified over UART) and, if cfg->encoder_present,
 * initializes the AS5047P. Closed-loop joints need both; open-loop joints
 * still read the encoder if present (for bring-up / publishing). */
bool Joint_Init(Joint_t *joint, const Joint_Config_t *cfg);
 
/* Sets the desired output-shaft position. Does not move anything by
 * itself - Joint_Update() does the actual work on the next tick. */
void Joint_SetTargetDeg(Joint_t *joint, float target_deg);
 
/* Call at a fixed rate (e.g. 1kHz) from the control task. Reads the encoder
 * (if present), and in closed-loop mode runs the PID loop and updates the
 * step timer's period and the DIR pin. dt in seconds.
 * Do NOT talk to the TMC2209 from here: UART calls block for milliseconds. */
void Joint_Update(Joint_t *joint, float dt);
 
/* Open-loop bring-up: run the motor at a fixed STEP rate. step_hz = 0 stops.
 * Only works when cfg->open_loop is true. */
void Joint_RunOpenLoop(Joint_t *joint, float step_hz, bool positive_dir);
 
/* Read-only state accessors, for RosInterface to publish. */
float Joint_GetPositionDeg(const Joint_t *joint);
float Joint_GetVelocityDps(const Joint_t *joint);
float Joint_GetTargetDeg(const Joint_t *joint);
bool Joint_HasFault(const Joint_t *joint);
bool Joint_EncoderOk(const Joint_t *joint);
tmc_status_t Joint_GetTmcStatus(const Joint_t *joint);
 
#endif // JOINT_H
 