/**
 * ros_interface.c
 *
 * NOTE on callbacks: rclc's callback signature is void(*)(const void *msgin)
 * with no user-context pointer, so we keep one file-scope pointer to the
 * active RosInterface_t. Fine for a single-arm firmware.
 *
 * If the agent is restarted while the board is running, the micro-ROS
 * session is gone and this code does not rebuild it: reset the board.
 */

#include "ros_interface.h"
#include "cmsis_os.h"
#include <rmw_microros/rmw_microros.h>
#include <string.h>

/* Debug variables: read with the debugger or STM32_Programmer_CLI -r32.
 * Look up addresses with:  arm-none-eabi-nm build/*.elf | grep g_ros_ */
volatile int      g_ros_fail_line   = 0;   /* line of the first failed init call (0 = none) */
volatile int      g_ros_fail_rc     = 0;   /* its rcl return code */
volatile uint32_t g_ros_ping_tries  = 0;   /* climbs while the agent is not answering */
volatile uint32_t g_ros_cmd_count   = 0;   /* test_step_hz messages received */
volatile float    g_ros_last_cmd_hz = 0.0f;/* last value received (800.0f reads as 0x44480000) */

#define RCCHECK(fn) do { rcl_ret_t rc = (fn); if (rc != RCL_RET_OK) { \
    g_ros_fail_line = __LINE__; g_ros_fail_rc = (int)rc; return false; } } while (0)

#define PUBLISH_PERIOD_MS   50                          /* 20 Hz state publish */
#define HEARTBEAT_EVERY     (1000 / PUBLISH_PERIOD_MS)  /* every 20th tick = 1 Hz */

static RosInterface_t *s_active_ros = NULL;

/* arm/test_step_hz: signed step rate for joint 0 (open loop). This only posts
 * the command; RobotArm_Update() applies it and enforces the 500 ms timeout. */
static void step_callback(const void *msgin)
{
    const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
    if (s_active_ros == NULL) return;

    g_ros_last_cmd_hz = msg->data;
    g_ros_cmd_count++;
    RobotArm_SetOpenLoopCmd(s_active_ros->arm, 0, msg->data);
}

/* arm/joint_cmd: degree targets. Harmless in open-loop mode (Joint_Update
 * returns early); becomes the real command path after the encoder works. */
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
    RosInterface_t *ros = s_active_ros;

    float positions[ROBOT_ARM_NUM_JOINTS];
    float velocities[ROBOT_ARM_NUM_JOINTS];
    RobotArm_GetState(ros->arm, positions, velocities);

    for (int i = 0; i < ROBOT_ARM_NUM_JOINTS; i++) {
        ros->state_data[i] = positions[i];
        ros->state_data[ROBOT_ARM_NUM_JOINTS + i] = velocities[i];
    }
    rcl_publish(&ros->state_publisher, &ros->state_msg, NULL);

    if (++ros->tick % HEARTBEAT_EVERY == 0) {
        ros->heartbeat_msg.data++;
        rcl_publish(&ros->heartbeat_publisher, &ros->heartbeat_msg, NULL);
    }
}

bool RosInterface_Init(RosInterface_t *ros, RobotArm_t *arm)
{
    memset(ros, 0, sizeof(*ros));
    ros->arm = arm;
    s_active_ros = ros;

    ros->allocator = rcl_get_default_allocator();

    /* At boot the agent is often not running yet (or the cable is not in).
     * Keep trying instead of giving up. */
    while (rmw_uros_ping_agent(200, 3) != RMW_RET_OK) {
        g_ros_ping_tries++;
        osDelay(500);
    }

    RCCHECK(rclc_support_init(&ros->support, 0, NULL, &ros->allocator));
    RCCHECK(rclc_node_init_default(&ros->node, "robot_arm_node", "", &ros->support));

    /* Streams are best-effort: a reliable publish can wait for an ack, and on
     * a dead link that stalls this task. */
    RCCHECK(rclc_publisher_init_best_effort(
        &ros->heartbeat_publisher, &ros->node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "arm/heartbeat"));

    RCCHECK(rclc_publisher_init_best_effort(
        &ros->state_publisher, &ros->node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "arm/joint_state"));

    RCCHECK(rclc_subscription_init_default(
        &ros->step_subscriber, &ros->node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "arm/test_step_hz"));

    RCCHECK(rclc_subscription_init_default(
        &ros->command_subscriber, &ros->node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "arm/joint_cmd"));

    RCCHECK(rclc_timer_init_default(
        &ros->publish_timer, &ros->support,
        RCL_MS_TO_NS(PUBLISH_PERIOD_MS), publish_timer_callback));

    /* Bind the static backing arrays. Everything else was zeroed by memset. */
    ros->state_msg.data.data = ros->state_data;
    ros->state_msg.data.size = 2 * ROBOT_ARM_NUM_JOINTS;
    ros->state_msg.data.capacity = 2 * ROBOT_ARM_NUM_JOINTS;

    ros->command_msg.data.data = ros->command_data;
    ros->command_msg.data.size = ROBOT_ARM_NUM_JOINTS;
    ros->command_msg.data.capacity = ROBOT_ARM_NUM_JOINTS;

    /* Executor handles: 2 subscriptions + 1 timer = 3. */
    RCCHECK(rclc_executor_init(&ros->executor, &ros->support.context, 3, &ros->allocator));
    RCCHECK(rclc_executor_add_subscription(
        &ros->executor, &ros->step_subscriber, &ros->step_msg,
        &step_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &ros->executor, &ros->command_subscriber, &ros->command_msg,
        &command_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&ros->executor, &ros->publish_timer));

    ros->agent_connected = true;
    return true;
}

volatile uint32_t g_ros_spin_count = 0;

void RosInterface_Task(void *argument)
{
    (void)argument;
    RosInterface_t *ros = s_active_ros; // set by RosInterface_Init

    for (;;) {
        g_ros_spin_count++;
        rclc_executor_spin_some(&ros->executor, RCL_MS_TO_NS(10));
        osDelay(1);
    }
}