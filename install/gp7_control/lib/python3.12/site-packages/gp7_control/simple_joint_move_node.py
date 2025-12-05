import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class SimpleJointMover(Node):
    def __init__(self):
        super().__init__('simple_joint_mover')

        # TODO: replace these with your robot's actual joint names
        self.joint_names = [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6',
        ]

        # Topic for your joint trajectory controller
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Wait a bit then send command once
        self.timer = self.create_timer(2.0, self.send_command_once)
        self.sent = False

    def send_command_once(self):
        if self.sent:
            return
        self.sent = True

        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        # Example command:
        # Move joint_1 to 30 deg, joint_2 to -20 deg, others stay at 0
        # (convert degrees to radians)
        import math
        target_positions = [
            math.radians(30.0),   # joint_1
            math.radians(-20.0),  # joint_2
            0.0,                  # joint_3
            0.0,                  # joint_4
            0.0,                  # joint_5
            0.0,                  # joint_6
        ]

        point = JointTrajectoryPoint()
        point.positions = target_positions

        # Reach the target in 3 seconds
        point.time_from_start.sec = 3
        point.time_from_start.nanosec = 0

        traj.points.append(point)

        self.get_logger().info(f"Sending joint target: {target_positions}")
        self.publisher.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleJointMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
