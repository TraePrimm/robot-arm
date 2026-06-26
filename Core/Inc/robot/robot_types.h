typedef struct {
    float joint_pos[6];
    float joint_vel[6];
} RobotState;

typedef struct {
    float joint_target[6];
    float max_velocity;
} MotionCommand;