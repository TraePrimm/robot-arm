/**
 * ros_interface.c
 *
 * NOTE on the subscription callback: rclc's callback signature is
 * void(*)(const void *msgin) with no user-context pointer, so we keep a
 * single file-scope pointer to the active RosInterface_t. Fine for a
 * single-arm firmware; if you ever run two arm instances in one binary
 * you'd need rclc_executor_add_subscription_with_context instead.
 */

#include "ros_interface.h"
#include "cmsis_os.h" // vTaskDelay / osDelay, whichever your project uses
#include <string.h>

#define RCCHECK(fn) do { rcl_ret_t rc = (fn); if (rc != RCL_RET_OK) { return false; } } while (0)

#define PUBLISH_PERIOD_MS 50   // 20Hz state publish - plenty for testing tonight

static RosInterface_t *s_active_ros = NULL; // see note above

static void command_callback(const void *msgin)
{
    const std_msgs__msg__Float32MultiArray *msg = (const std_msgs__msg__Float32MultiArray *)msgin;
    if (s_active_ros == NULL || msg->data.size < ROBOT_ARM_NUM_JOINTS) {
        return; // ignore malformed/short commands rather than reading OOB
    }
    RobotArm_SetTargets(s_active_ros->arm, msg->data.data);
}

static void publish_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer == NULL || s_active_ros == NULL) return;

    float positions[ROBOT_ARM_NUM_JOINTS];
    float velocities[ROBOT_ARM_NUM_JOINTS];
    RobotArm_GetState(s_active_ros->arm, positions, velocities);

    for (int i = 0; i < ROBOT_ARM_NUM_JOINTS; i++) {
        s_active_ros->state_data[i] = positions[i];
        s_active_ros->state_data[ROBOT_ARM_NUM_JOINTS + i] = velocities[i];
    }
    rcl_publish(&s_active_ros->state_publisher, &s_active_ros->state_msg, NULL);
}

bool RosInterface_Init(RosInterface_t *ros, RobotArm_t *arm)
{
    memset(ros, 0, sizeof(*ros));
    ros->arm = arm;
    s_active_ros = ros;

    ros->allocator = rcl_get_default_allocator();

    /* Wait for the agent - this is where a broken transport (untested
     * Ethernet setup!) will hang, so test transport in isolation first
     * if this stalls. */
    if (rmw_uros_ping_agent(1000, 3) != RMW_RET_OK) {
        return false;
    }

    RCCHECK(rclc_support_init(&ros->support, 0, NULL, &ros->allocator));
    RCCHECK(rclc_node_init_default(&ros->node, "robot_arm_node", "", &ros->support));

    RCCHECK(rclc_publisher_init_default(
        &ros->state_publisher, &ros->node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "joint_states"));

    RCCHECK(rclc_subscription_init_default(
        &ros->command_subscriber, &ros->node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "joint_commands"));

    RCCHECK(rclc_timer_init_default(
        &ros->publish_timer, &ros->support,
        RCL_MS_TO_NS(PUBLISH_PERIOD_MS), publish_timer_callback));

    /* Statically bind message backing arrays - no malloc, no
     * micro_ros_utilities needed since sizes are fixed and known. */
    ros->state_msg.data.data = ros->state_data;
    ros->state_msg.data.size = 2 * ROBOT_ARM_NUM_JOINTS;
    ros->state_msg.data.capacity = 2 * ROBOT_ARM_NUM_JOINTS;
    ros->state_msg.layout.dim.data = NULL;
    ros->state_msg.layout.dim.size = 0;
    ros->state_msg.layout.dim.capacity = 0;

    ros->command_msg.data.data = ros->command_data;
    ros->command_msg.data.size = ROBOT_ARM_NUM_JOINTS;
    ros->command_msg.data.capacity = ROBOT_ARM_NUM_JOINTS;
    ros->command_msg.layout.dim.data = NULL;
    ros->command_msg.layout.dim.size = 0;
    ros->command_msg.layout.dim.capacity = 0;

    /* Executor needs room for: 1 subscription + 1 timer. */
    RCCHECK(rclc_executor_init(&ros->executor, &ros->support.context, 2, &ros->allocator));
    RCCHECK(rclc_executor_add_subscription(
        &ros->executor, &ros->command_subscriber, &ros->command_msg,
        &command_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&ros->executor, &ros->publish_timer));

    ros->agent_connected = true;
    return true;
}

void RosInterface_Task(void *argument)
{
    (void)argument;
    RosInterface_t *ros = s_active_ros; // set by RosInterface_Init before this task starts

    for (;;) {
        rclc_executor_spin_some(&ros->executor, RCL_MS_TO_NS(10));
        osDelay(10); // yield - this task doesn't need to be tight
    }
}