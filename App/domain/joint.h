/**
 * joint.h
 *
 * One joint = one TMC2209 + one AS5047P + a gear ratio + a position PID
 * loop that turns "target angle" into STEP pulses.
 *
 * Everything OUTSIDE this file talks in output-shaft degrees. Gear ratio
 * and microstep math are private to this class - RobotArm and RosInterface
 * never need to know a joint is geared 24:1 vs 1:1.
 */
#ifndef JOINT_H
#define JOINT_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"
#include "tmc2209.h"
#include "as5047p.h"

typedef struct {
    /* --- wiring / hardware, set once at startup --- */
    UART_HandleTypeDef *uart;      // TMC2209 UART bus
    uint8_t tmc_address;           // 0-3 on that bus
    SPI_HandleTypeDef *spi;        // AS5047P SPI bus
    GPIO_TypeDef *encoder_cs_port;
    uint16_t encoder_cs_pin;
    GPIO_TypeDef *step_port;
    uint16_t step_pin;
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
    TIM_HandleTypeDef *step_timer; // generates the STEP pulse train

    /* --- joint-specific constants --- */
    float gear_ratio;      // motor revolutions per output-shaft revolution
    uint16_t microsteps;   // e.g. 16, 256
    float run_current;     // 0.0-1.0, fraction of driver's max current
    float hold_current;    // 0.0-1.0

    /* --- PID gains (tune per-joint once assembled) --- */
    float kp, ki, kd;
} Joint_Config_t;

typedef struct {
    Joint_Config_t cfg;
    TMC2209_t motor;
    AS5047P_t encoder;

    float target_deg;      // desired output-shaft position
    float current_deg;     // last measured output-shaft position
    float current_vel_dps; // degrees/sec, estimated from consecutive readings

    /* PID internal state */
    float integral;
    float prev_error;

    bool fault;             // set if driver/encoder comms fail
} Joint_t;

/* Stores config + initializes the TMC2209 and AS5047P sub-drivers.
 * Returns false if either device fails to respond (check wiring). */
bool Joint_Init(Joint_t *joint, const Joint_Config_t *cfg);

/* Sets the desired output-shaft position. Does not move anything by
 * itself - Joint_Update() does the actual work on the next tick. */
void Joint_SetTargetDeg(Joint_t *joint, float target_deg);

/* Call at a fixed rate (e.g. 1kHz) from the control task.
 * Reads the encoder, runs the PID loop, updates the step timer's
 * frequency and the DIR pin. dt in seconds. */
void Joint_Update(Joint_t *joint, float dt);

/* Read-only state accessors, for RosInterface to publish. */
float Joint_GetPositionDeg(const Joint_t *joint);
float Joint_GetVelocityDps(const Joint_t *joint);
bool Joint_HasFault(const Joint_t *joint);

/* Called from the step timer's period-elapsed interrupt (wire this up in
 * your HAL_TIM_PeriodElapsedCallback dispatch in stm32h7xx_it.c). Toggles
 * the STEP pin. Kept tiny and ISR-safe - no floating point, no HAL calls
 * other than the single GPIO toggle. */
void Joint_StepISR(Joint_t *joint);

#endif // JOINT_H