
/**
 * robot_arm_config.c
 *
 * THE FILE YOU EDIT. Everything else in App/ is generic - this is where
 * your specific wiring, gear ratios, and PID gains live.
 *
 * Wiring below comes from the project pinout table (authoritative):
 *
 *   Joint | UART/addr   | STEP (timer)       | DIR  | Encoder CS
 *   ------+-------------+--------------------+------+-----------
 *     0   | huart1 / 0  | PA5  TIM2  CH1     | PC0  | PA4
 *     1   | huart1 / 1  | PC8  TIM3  CH3     | PC9  | PB2
 *     2   | huart1 / 2  | PD12 TIM4  CH1     | PC3  | PC2
 *     3   | huart1 / 3  | PC6  TIM8  CH1     | PC11 | PC10
 *     4   | huart2 / 0  | PB15 TIM12 CH2     | PD1  | PD0
 *     5   | huart2 / 1  | PA0  TIM5  CH1     | PD3  | PD2
 *
 * All encoders share SPI1 (SCK PB3, MISO PA6, MOSI PD7), one CS each.
 * Every encoder module needs its JP1 (5V/3V3) bridged for 3.3 V supply.
 *
 * Only joints 0 and 1 are wired for bring-up. Joints 2-5 will fail
 * Joint_Init() cleanly (RobotArm_Init keeps going), which means
 * RobotArm_HasFault() stays true until they are wired.
 */
 
#include "robot_arm.h"
 
extern UART_HandleTypeDef huart1; // joints 0-3
extern UART_HandleTypeDef huart2; // joints 4-5
extern SPI_HandleTypeDef hspi1;   // all 6 encoders, per-joint CS
extern TIM_HandleTypeDef htim2;   // joint 0 step timer
extern TIM_HandleTypeDef htim3;   // joint 1 step timer
extern TIM_HandleTypeDef htim4;   // joint 2 step timer
extern TIM_HandleTypeDef htim8;   // joint 3 step timer
extern TIM_HandleTypeDef htim12;  // joint 4 step timer
extern TIM_HandleTypeDef htim5;   // joint 5 step timer
 
/* Count rate of the step timers = timer kernel clock / (PSC + 1).
 * 275 MHz timer kernel clock, PSC = 274 -> 1 MHz.
 * The slowest step rate is about STEP_TIMER_COUNT_HZ / 65536 (~15 Hz). */
#define STEP_TIMER_COUNT_HZ   1000000.0f
 
/* Joints still in bring-up: no encoder, no PID. */
#define BRINGUP_OPEN_LOOP     true
 
/* ---- Joint 0 bench test -------------------------------------------------
 * JOINT0_BENCH:     magnet on the bare motor shaft, so the encoder sees the
 *                   motor, not the 24:1 output -> gear_ratio 1.0, P-only gains.
 *                   Set false once the encoder sits on the real joint output.
 * JOINT0_OPEN_LOOP: phase 2 = true  (drive with arm/test_step_hz, encoder only
 *                                    reported on arm/joint_state)
 *                   phase 3 = false (closed loop, targets from arm/joint_cmd)
 * JOINT0_ENC_SIGN:  +1 or -1. In phase 2, send a POSITIVE test_step_hz: if the
 *                   angle on arm/joint_state goes DOWN, set this to -1.
 *                   Wrong sign in closed loop = the motor runs away. */
#define JOINT0_BENCH          true
#define JOINT0_OPEN_LOOP      true
#define JOINT0_ENC_SIGN       (+1)
 
void RobotArm_GetJointConfigs(Joint_Config_t out[ROBOT_ARM_NUM_JOINTS])
{
    /* ---- Joint 0: base, 24:1 gear ratio ---- */
    out[0] = (Joint_Config_t){
        .uart = &huart1,           .tmc_address = 0,
        .spi = &hspi1,
        .encoder_cs_port = GPIOA,  .encoder_cs_pin = GPIO_PIN_4,
        .encoder_id = 0,
        .dir_port = GPIOC,         .dir_pin = GPIO_PIN_0,
        .step_timer = &htim2,      .step_channel = TIM_CHANNEL_1,
        .step_timer_clk_hz = STEP_TIMER_COUNT_HZ,
#if JOINT0_BENCH
        .gear_ratio = 1.0f,                     // encoder on the motor shaft
        .kp = 5.0f, .ki = 0.0f, .kd = 0.0f,     // P only: 5 deg/s per deg of error
#else
        .gear_ratio = 24.0f,
        .kp = 4.0f, .ki = 0.5f, .kd = 0.1f,     // starting guess - you WILL retune this
#endif
        .microsteps = 16,
        .run_current = 0.6f,       .hold_current = 0.3f,
        .encoder_sign = JOINT0_ENC_SIGN,
        .encoder_offset_deg = 0.0f,
        .max_step_hz = 2000.0f,                 // same cap as open-loop bring-up
        .accel_hz_per_s = 20000.0f,             // 0 -> 2000 Hz in 100 ms
        .open_loop = JOINT0_OPEN_LOOP,
        .encoder_present = true,
    };
 
    /* ---- Joint 1: shoulder, 24:1 ---- */
    out[1] = (Joint_Config_t){
        .uart = &huart1,           .tmc_address = 1,
        .spi = &hspi1,
        .encoder_cs_port = GPIOB,  .encoder_cs_pin = GPIO_PIN_2,
        .encoder_id = 1,
        .dir_port = GPIOC,         .dir_pin = GPIO_PIN_9,
        .step_timer = &htim3,      .step_channel = TIM_CHANNEL_3,
        .step_timer_clk_hz = STEP_TIMER_COUNT_HZ,
        .gear_ratio = 24.0f,
        .microsteps = 16,
        .run_current = 0.6f,       .hold_current = 0.3f,
        .kp = 4.0f, .ki = 0.5f, .kd = 0.1f,
        .encoder_sign = +1,
        .open_loop = BRINGUP_OPEN_LOOP,
        .encoder_present = false,               // true once its encoder is wired
    };
 
    /* ---- Joints 2-5: wiring from the table; gear ratios are placeholders
     *      (1:1) until you know the real ones. ---- */
    out[2] = (Joint_Config_t){
        .uart = &huart1,           .tmc_address = 2,
        .spi = &hspi1,
        .encoder_cs_port = GPIOC,  .encoder_cs_pin = GPIO_PIN_2,
        .encoder_id = 2,
        .dir_port = GPIOC,         .dir_pin = GPIO_PIN_3,
        .step_timer = &htim4,      .step_channel = TIM_CHANNEL_1,
        .step_timer_clk_hz = STEP_TIMER_COUNT_HZ,
        .gear_ratio = 1.0f,        // TODO: real ratio
        .microsteps = 16,
        .run_current = 0.6f,       .hold_current = 0.3f,
        .kp = 4.0f, .ki = 0.5f, .kd = 0.1f,
        .encoder_sign = +1,
        .open_loop = BRINGUP_OPEN_LOOP,
        .encoder_present = false,
    };
 
    out[3] = (Joint_Config_t){
        .uart = &huart1,           .tmc_address = 3,
        .spi = &hspi1,
        .encoder_cs_port = GPIOC,  .encoder_cs_pin = GPIO_PIN_10,
        .encoder_id = 3,
        .dir_port = GPIOC,         .dir_pin = GPIO_PIN_11,
        .step_timer = &htim8,      .step_channel = TIM_CHANNEL_1,
        .step_timer_clk_hz = STEP_TIMER_COUNT_HZ,
        .gear_ratio = 1.0f,        // TODO: real ratio
        .microsteps = 16,
        .run_current = 0.6f,       .hold_current = 0.3f,
        .kp = 4.0f, .ki = 0.5f, .kd = 0.1f,
        .encoder_sign = +1,
        .open_loop = BRINGUP_OPEN_LOOP,
        .encoder_present = false,
    };
 
    out[4] = (Joint_Config_t){
        .uart = &huart2,           .tmc_address = 0,
        .spi = &hspi1,
        .encoder_cs_port = GPIOD,  .encoder_cs_pin = GPIO_PIN_0,
        .encoder_id = 4,
        .dir_port = GPIOD,         .dir_pin = GPIO_PIN_1,
        .step_timer = &htim12,     .step_channel = TIM_CHANNEL_2,
        .step_timer_clk_hz = STEP_TIMER_COUNT_HZ,
        .gear_ratio = 1.0f,        // TODO: real ratio
        .microsteps = 16,
        .run_current = 0.6f,       .hold_current = 0.3f,
        .kp = 4.0f, .ki = 0.5f, .kd = 0.1f,
        .encoder_sign = +1,
        .open_loop = BRINGUP_OPEN_LOOP,
        .encoder_present = false,
    };
 
    out[5] = (Joint_Config_t){
        .uart = &huart2,           .tmc_address = 1,
        .spi = &hspi1,
        .encoder_cs_port = GPIOD,  .encoder_cs_pin = GPIO_PIN_2,
        .encoder_id = 5,
        .dir_port = GPIOD,         .dir_pin = GPIO_PIN_3,
        .step_timer = &htim5,      .step_channel = TIM_CHANNEL_1,
        .step_timer_clk_hz = STEP_TIMER_COUNT_HZ,
        .gear_ratio = 1.0f,        // TODO: real ratio
        .microsteps = 16,
        .run_current = 0.6f,       .hold_current = 0.3f,
        .kp = 4.0f, .ki = 0.5f, .kd = 0.1f,
        .encoder_sign = +1,
        .open_loop = BRINGUP_OPEN_LOOP,
        .encoder_present = false,
    };
}
 