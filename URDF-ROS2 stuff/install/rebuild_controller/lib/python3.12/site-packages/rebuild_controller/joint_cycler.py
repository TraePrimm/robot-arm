import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointCycler(Node):

    def __init__(self):
        super().__init__('joint_cycler')

        self.publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # Joint name : [minimum, maximum]
        self.joints = [
            ('Revolute 7', -1.0, 1.0),
            ('Revolute 8', -1.134464, 1.134464),
            ('Revolute 9', -1.221730, 1.221730),
            ('Revolute 10', -1.0, 1.0),
            ('Revolute 11', -1.431170, 1.431170),
            ('Revolute 12', -1.0, 1.0),
            ('Slider 14', 0.0, 0.015),
        ]

        # Store the current position of every joint
        self.positions = {}

        for joint_name, minimum, maximum in self.joints:
            self.positions[joint_name] = 0.0

        # Start with the first joint
        self.current_joint = 0

        # Direction:
        # +1 = moving toward maximum
        # -1 = moving toward minimum
        self.direction = 1

        # Run update_robot every 0.1 seconds = 10 Hz
        self.timer = self.create_timer(
            0.1,
            self.update_robot
        )

    def update_robot(self):

        # Get information about the joint we're currently moving
        joint_name, minimum, maximum = self.joints[self.current_joint]

        # Change its position
        self.positions[joint_name] += 0.01 * self.direction

        # Hit maximum
        if self.positions[joint_name] >= maximum:
            self.positions[joint_name] = maximum
            self.direction = -1

        # Hit minimum
        if self.positions[joint_name] <= minimum:
            self.positions[joint_name] = minimum

            # Move to the next joint
            self.current_joint += 1

            # If we've tested every joint, start over
            if self.current_joint >= len(self.joints):
                self.current_joint = 0

            self.direction = 1

        # Create JointState message
        msg = JointState()

        msg.header.stamp = self.get_clock().now().to_msg()

        # Publish every joint
        msg.name = [
            'Revolute 7',
            'Revolute 8',
            'Revolute 9',
            'Revolute 10',
            'Revolute 11',
            'Revolute 12',
            'Slider 14'
        ]

        msg.position = [
            self.positions['Revolute 7'],
            self.positions['Revolute 8'],
            self.positions['Revolute 9'],
            self.positions['Revolute 10'],
            self.positions['Revolute 11'],
            self.positions['Revolute 12'],
            self.positions['Slider 14']
        ]

        self.publisher.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = JointCycler()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
