/**
 * robot_arm.c
 */

#include "robot_arm.h"
#include <math.h>
#include <string.h>

#define OPEN_LOOP_TIMEOUT_MS  500      /* no fresh command for this long -> stop */
#define OPEN_LOOP_MAX_HZ      2000.0f  /* bring-up cap: no acceleration ramp yet, so a big
                                          jump from 0 would just stall the motor */

bool RobotArm_Init(RobotArm_t *arm)
{
    Joint_Config_t configs[ROBOT_ARM_NUM_JOINTS];
    RobotArm_GetJointConfigs(configs);

    /* No memset of *arm here on purpose: the mailbox may already hold a command,
     * and g_arm is a global, so it starts zeroed anyway. */
    arm->any_fault = false;
    bool all_ok = true;

    for (int i = 0; i < ROBOT_ARM_NUM_JOINTS; i++) {
        if (!Joint_Init(&arm->joints[i], &configs[i])) {
            all_ok = false;
            arm->any_fault = true;
            // keep going so we init the joints that DO work, useful when
            // you're bringing up motor 1 of 6 and the rest aren't wired yet
        }
    }
    return all_ok;
}

void RobotArm_SetTargets(RobotArm_t *arm, const float targets_deg[ROBOT_ARM_NUM_JOINTS])
{
    for (int i = 0; i < ROBOT_ARM_NUM_JOINTS; i++) {
        Joint_SetTargetDeg(&arm->joints[i], targets_deg[i]);
    }
}

void RobotArm_SetOpenLoopCmd(RobotArm_t *arm, uint8_t joint, float signed_step_hz)
{
    if (joint >= ROBOT_ARM_NUM_JOINTS) {
        return;
    }
    if (signed_step_hz != signed_step_hz) {            /* NaN */
        signed_step_hz = 0.0f;
    } else if (signed_step_hz > OPEN_LOOP_MAX_HZ) {
        signed_step_hz = OPEN_LOOP_MAX_HZ;
    } else if (signed_step_hz < -OPEN_LOOP_MAX_HZ) {
        signed_step_hz = -OPEN_LOOP_MAX_HZ;
    }

    /* Value first, then the timestamp that makes it "fresh". Both fields are
     * volatile, so the compiler keeps this order; single core, so no barrier. */
    arm->ol_cmd_hz[joint] = signed_step_hz;
    arm->ol_cmd_ms[joint] = HAL_GetTick();
}

void RobotArm_Update(RobotArm_t *arm, float dt)
{
    const uint32_t now = HAL_GetTick();

    arm->any_fault = false;
    for (int i = 0; i < ROBOT_ARM_NUM_JOINTS; i++) {

        /* Open-loop mailbox. Read the stamp first, then the value. The age is
         * signed so a stamp written just after 'now' was read counts as fresh,
         * not as 49 days old. Joint_RunOpenLoop() ignores joints that are not
         * ready or not in open-loop mode, so this is harmless in closed loop. */
        const uint32_t stamp = arm->ol_cmd_ms[i];
        float hz = arm->ol_cmd_hz[i];
        if ((int32_t)(now - stamp) > (int32_t)OPEN_LOOP_TIMEOUT_MS) {
            hz = 0.0f;                                  /* dead-man: command went stale */
        }
        if (hz != arm->ol_applied_hz[i]) {              /* only touch the timer on change */
            Joint_RunOpenLoop(&arm->joints[i], fabsf(hz), hz >= 0.0f);
            arm->ol_applied_hz[i] = hz;
        }

        Joint_Update(&arm->joints[i], dt);
        if (Joint_HasFault(&arm->joints[i])) {
            arm->any_fault = true;
        }
    }
}

void RobotArm_GetState(const RobotArm_t *arm, float positions_deg[ROBOT_ARM_NUM_JOINTS],
                       float velocities_dps[ROBOT_ARM_NUM_JOINTS])
{
    for (int i = 0; i < ROBOT_ARM_NUM_JOINTS; i++) {
        if (positions_deg)  positions_deg[i]  = Joint_GetPositionDeg(&arm->joints[i]);
        if (velocities_dps) velocities_dps[i] = Joint_GetVelocityDps(&arm->joints[i]);
    }
}

bool RobotArm_HasFault(const RobotArm_t *arm)
{
    return arm->any_fault;
}
