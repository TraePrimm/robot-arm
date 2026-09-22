/**
 * robot_arm_config.c
 *
 * THE FILE YOU EDIT. Everything else in App/ is generic - this is where
 * your specific wiring, gear ratios, and PID gains live.
 *
 * All the huartX/hspiX handles and GPIO port/pin defines below come from
 * your CubeMX-generated main.h/main.c (or wherever CubeMX put them) -
 * replace the placeholders with your actual generated names.
 *
 * UART bus plan (TMC2209 allows max 4 addresses per shared bus):
 *   huart1: joints 0-3 (addresses 0,1,2,3)
 *   huart2: joints 4-5 (addresses 0,1)
 *
 * For TONIGHT (steps 3-4 of your list: 1 motor, then 2 motors), you only
 * need joints[0] and joints[1] wired correctly - the rest can stay as
 * placeholders and will just fail Joint_Init() cleanly (RobotArm_Init
 * keeps going and reports which ones faulted).
 */

#include "robot_arm.h"

/* TODO: replace with your real CubeMX extern handles */
extern UART_HandleTypeDef huart1; // joints 0-3
extern UART_HandleTypeDef huart2; // joints 4-5
extern SPI_HandleTypeDef hspi1;   // all 6 encoders, if sharing one SPI bus with per-joint CS
extern TIM_HandleTypeDef htim2;   // joint 0 step timer
extern TIM_HandleTypeDef htim3;   // joint 1 step timer
extern TIM_HandleTypeDef htim4;   // joint 2 step timer
extern TIM_HandleTypeDef htim5;   // joint 3 step timer
extern TIM_HandleTypeDef htim6;   // joint 4 step timer  (note: TIM6/7 are basic timers, no GPIO - verify these support the mode you need, else pick different timers)
extern TIM_HandleTypeDef htim7;   // joint 5 step timer

void RobotArm_GetJointConfigs(Joint_Config_t out[ROBOT_ARM_NUM_JOINTS])
{
    /* ---- Joint 0: base, 24:1 gear ratio, TONIGHT'S TEST MOTOR ---- */
    out[0] = (Joint_Config_t){
        .uart = &huart1,           .tmc_address = 0,
        .spi = &hspi1,
        .encoder_cs_port = GPIOA,  .encoder_cs_pin = GPIO_PIN_4,   // TODO: your CS pin
        .step_port = GPIOB,        .step_pin = GPIO_PIN_0,          // TODO
        .dir_port = GPIOB,         .dir_pin = GPIO_PIN_1,           // TODO
        .step_timer = &htim2,
        .gear_ratio = 24.0f,
        .microsteps = 16,
        .run_current = 0.6f,       .hold_current = 0.3f,
        .kp = 4.0f, .ki = 0.5f, .kd = 0.1f,   // starting guess - you WILL retune this
    };

    /* ---- Joint 1: shoulder, second test motor for tonight ---- */
    out[1] = (Joint_Config_t){
        .uart = &huart1,           .tmc_address = 1,
        .spi = &hspi1,
        .encoder_cs_port = GPIOA,  .encoder_cs_pin = GPIO_PIN_5,   // TODO
        .step_port = GPIOB,        .step_pin = GPIO_PIN_2,          // TODO
        .dir_port = GPIOB,         .dir_pin = GPIO_PIN_3,           // TODO
        .step_timer = &htim3,
        .gear_ratio = 24.0f,
        .microsteps = 16,
        .run_current = 0.6f,       .hold_current = 0.3f,
        .kp = 4.0f, .ki = 0.5f, .kd = 0.1f,
    };

    /* ---- Joints 2-5: placeholders until you wire the rest ----
     * Fill these in the same pattern once you're past tonight's 2-motor
     * test. Using address 2/3 on huart1 and 0/1 on huart2 per the plan
     * above. Ratios below are guesses (1:1 direct-drive) - replace with
     * your actual per-joint ratios. */
    for (int i = 2; i < ROBOT_ARM_NUM_JOINTS; i++) {
        out[i] = (Joint_Config_t){
            .uart = (i < 4) ? &huart1 : &huart2,
            .tmc_address = (i < 4) ? (uint8_t)i : (uint8_t)(i - 4),
            .spi = &hspi1,
            .encoder_cs_port = GPIOA, .encoder_cs_pin = (uint16_t)(GPIO_PIN_6 << (i - 2)), // TODO
            .step_port = GPIOC,       .step_pin = (uint16_t)(GPIO_PIN_0 << (i - 2)),       // TODO
            .dir_port = GPIOC,        .dir_pin = (uint16_t)(GPIO_PIN_1 << (i - 2)),        // TODO
            .step_timer = (i == 2) ? &htim4 : (i == 3) ? &htim5 : (i == 4) ? &htim6 : &htim7,
            .gear_ratio = 1.0f,       // TODO: real ratio for this joint
            .microsteps = 16,
            .run_current = 0.6f,      .hold_current = 0.3f,
            .kp = 4.0f, .ki = 0.5f, .kd = 0.1f,
        };
    }
}