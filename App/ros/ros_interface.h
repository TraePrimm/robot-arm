/**
 * ros_interface.h
 *
 * The only file that knows ROS exists. Talks to RobotArm_t through its
 * public getters/setters only - RobotArm has no idea this file exists.
 *
 * Topics (deliberately simple for bring-up - swap for JointState/
 * JointTrajectory later once the pipeline is proven):
 *   subscribes  "joint_commands"  std_msgs/Float32MultiArray[6]  (deg)
 *   publishes   "joint_states"    std_msgs/Float32MultiArray[12] (deg, positions[0:6] then velocities[6:12])
 *
 * Run RosInterface_Task as its own FreeRTOS task, separate from whatever
 * task calls RobotArm_Update() at your control rate - a slow ROS callback
 * should never be able to stall your step timing.
 */
#ifndef ROS_INTERFACE_H
#define ROS_INTERFACE_H

#include <stdbool.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include "robot_arm.h"

typedef struct {
    RobotArm_t *arm; // not owned - just a pointer to the one the app created

    rcl_allocator_t allocator;
    rclc_support_t support;
    rcl_node_t node;

    rcl_publisher_t state_publisher;
    rcl_subscription_t command_subscriber;
    rcl_timer_t publish_timer;
    rclc_executor_t executor;

    std_msgs__msg__Float32MultiArray state_msg;
    std_msgs__msg__Float32MultiArray command_msg;

    /* Statically-owned backing arrays for the message sequences above -
     * avoids needing micro_ros_utilities' dynamic allocation setup. */
    float state_data[2 * ROBOT_ARM_NUM_JOINTS];   // positions then velocities
    float command_data[ROBOT_ARM_NUM_JOINTS];

    bool agent_connected;
} RosInterface_t;

/* Brings up the micro-ROS node, publisher, subscriber, and executor.
 * Call this AFTER your transport (UDP/serial/USB) is already working -
 * it blocks briefly waiting for the agent to respond. Returns false if
 * the agent isn't reachable. */
bool RosInterface_Init(RosInterface_t *ros, RobotArm_t *arm);

/* Run this as a FreeRTOS task (xTaskCreate(RosInterface_Task, ...)).
 * Never returns. */
void RosInterface_Task(void *argument);

#endif // ROS_INTERFACE_H