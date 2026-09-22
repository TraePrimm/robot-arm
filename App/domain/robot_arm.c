/**
 * robot_arm.c
 */

#include "robot_arm.h"
#include <string.h>

bool RobotArm_Init(RobotArm_t *arm)
{
    Joint_Config_t configs[ROBOT_ARM_NUM_JOINTS];
    RobotArm_GetJointConfigs(configs);

    arm->any_fault = false;
    bool all_ok = true;

    for (int i = 0; i < ROBOT_ARM_NUM_JOINTS; i++) {
        if (!Joint_Init(&arm->joints[i], &configs[i])) {
            all_ok = false;
            arm->any_fault = true;
            // keep going so we init the joints that DO work, useful when
            // you're bringing up motor 1 of 6 tonight and the rest aren't wired yet
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

void RobotArm_Update(RobotArm_t *arm, float dt)
{
    arm->any_fault = false;
    for (int i = 0; i < ROBOT_ARM_NUM_JOINTS; i++) {
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