/**
 * ros_interface.h
 *
 * The only file that knows ROS exists. Talks to RobotArm_t through its
 * public functions only - RobotArm has no idea this file exists.
 *
 * Topics (names are relative to the root namespace, so /arm/...):
 *   publishes   arm/heartbeat      std_msgs/Int32               1 Hz counter, best-effort
 *   publishes   arm/joint_state    std_msgs/Float32MultiArray   20 Hz, [6 positions deg, 6 velocities dps], best-effort
 *   subscribes  arm/test_step_hz   std_msgs/Float32             signed step rate for joint 0, OPEN LOOP, 500 ms dead-man
 *   subscribes  arm/joint_cmd      std_msgs/Float32MultiArray   6 targets in deg (only used once closed loop works)
 *
 * Run RosInterface_Task as its own FreeRTOS task, separate from whatever
 * task calls RobotArm_Update(): a slow ROS call must never stall step timing
 * or the command timeout.
 */
#ifndef ROS_INTERFACE_H
#define ROS_INTERFACE_H

#include <stdbool.h>
#include <stdint.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include "robot_arm.h"

typedef struct {
    RobotArm_t *arm;                 // not owned

    rcl_allocator_t allocator;
    rclc_support_t support;
    rcl_node_t node;

    rcl_publisher_t heartbeat_publisher;
    rcl_publisher_t state_publisher;
    rcl_subscription_t step_subscriber;
    rcl_subscription_t command_subscriber;
    rcl_timer_t publish_timer;
    rclc_executor_t executor;

    std_msgs__msg__Int32 heartbeat_msg;
    std_msgs__msg__Float32 step_msg;
    std_msgs__msg__Float32MultiArray state_msg;
    std_msgs__msg__Float32MultiArray command_msg;

    /* Statically-owned backing arrays: no dynamic allocation for messages. */
    float state_data[2 * ROBOT_ARM_NUM_JOINTS];   // positions then velocities
    float command_data[ROBOT_ARM_NUM_JOINTS];

    uint32_t tick;                   // publish-timer ticks, used to make the 1 Hz heartbeat
    bool agent_connected;
} RosInterface_t;

/* Waits for the agent (retries forever, 500 ms apart), then creates the node,
 * publishers, subscribers, timer and executor. Returns false only on a real
 * init error; g_ros_fail_line / g_ros_fail_rc say which call failed.
 * Call after the custom transport and allocators are registered. */
bool RosInterface_Init(RosInterface_t *ros, RobotArm_t *arm);

/* Spins the executor. Never returns. */
void RosInterface_Task(void *argument);

#endif // ROS_INTERFACE_H