#ifndef ROBOT_ARM_H
#define ROBOT_ARM_H

#include <stdint.h>
#include <stdbool.h>
#include "joint.h"

#define ROBOT_ARM_NUM_JOINTS 6

typedef struct {
    Joint_t joints[ROBOT_ARM_NUM_JOINTS];
    bool any_fault;

    /* Open-loop command mailbox (bring-up).
     * Writer: any task, via RobotArm_SetOpenLoopCmd().
     * Reader: RobotArm_Update() only. That function (control task) is the one
     * place that touches the step timers, so there are no races on them. */
    volatile float    ol_cmd_hz[ROBOT_ARM_NUM_JOINTS]; /* signed step rate, sign = direction */
    volatile uint32_t ol_cmd_ms[ROBOT_ARM_NUM_JOINTS]; /* HAL tick when the command arrived */
    float             ol_applied_hz[ROBOT_ARM_NUM_JOINTS]; /* control task only */
} RobotArm_t;

bool RobotArm_Init(RobotArm_t *arm);
void RobotArm_SetTargets(RobotArm_t *arm, const float targets_deg[ROBOT_ARM_NUM_JOINTS]);

/* Closed-loop work and the open-loop command timeout. Call at a fixed rate. */
void RobotArm_Update(RobotArm_t *arm, float dt);

void RobotArm_GetState(const RobotArm_t *arm, float positions_deg[ROBOT_ARM_NUM_JOINTS],
                       float velocities_dps[ROBOT_ARM_NUM_JOINTS]);
bool RobotArm_HasFault(const RobotArm_t *arm);
void RobotArm_GetJointConfigs(Joint_Config_t out[ROBOT_ARM_NUM_JOINTS]);

/* Bring-up only (needs cfg.open_loop == true on that joint).
 * Posts a signed step rate in Hz (positive/negative = direction, 0 = stop).
 * It is a dead-man command: if no new command arrives within the timeout
 * (500 ms), RobotArm_Update() stops the joint by itself. Values are clamped
 * (there is no acceleration ramp yet) and NaN is treated as 0.
 * Does not touch hardware; safe to call from the ROS task. */
void RobotArm_SetOpenLoopCmd(RobotArm_t *arm, uint8_t joint, float signed_step_hz);

#endif // ROBOT_ARM_H
