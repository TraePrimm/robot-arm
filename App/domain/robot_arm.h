#ifndef ROBOT_ARM_H
#define ROBOT_ARM_H

#include <stdint.h>
#include <stdbool.h>
#include "joint.h"

#define ROBOT_ARM_NUM_JOINTS 6

typedef struct {
    Joint_t joints[ROBOT_ARM_NUM_JOINTS];
    bool any_fault;
} RobotArm_t;

bool RobotArm_Init(RobotArm_t *arm);
void RobotArm_SetTargets(RobotArm_t *arm, const float targets_deg[ROBOT_ARM_NUM_JOINTS]);
void RobotArm_Update(RobotArm_t *arm, float dt);
void RobotArm_GetState(const RobotArm_t *arm, float positions_deg[ROBOT_ARM_NUM_JOINTS],
                        float velocities_dps[ROBOT_ARM_NUM_JOINTS]);
bool RobotArm_HasFault(const RobotArm_t *arm);
void RobotArm_GetJointConfigs(Joint_Config_t out[ROBOT_ARM_NUM_JOINTS]);

#endif // ROBOT_ARM_H