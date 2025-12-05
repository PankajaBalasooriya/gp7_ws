import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np


class TestMovementNode(Node):
    def __init__(self):
        super().__init__('gp7_test_movement')

        # TODO: change these to match your URDF/controller joint names
        self.joint_names = [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6',
        ]

        # Topic for your joint trajectory controller
        # Change if your controller uses a different topic name
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Give Gazebo time to start & controller to come up
        self.timer = self.create_timer(3.0, self.send_test_trajectory)
        self.already_sent = False

        self.get_logger().info("TestMovementNode initialized, will send trajectory once.")

    def send_test_trajectory(self):
        if self.already_sent:
            return
        self.already_sent = True

        self.get_logger().info("Sending test trajectory...")

        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        # Define some simple poses (in radians)
        q_home = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        q_pose1 = np.array([0.2, -0.4, 0.3, 0.0, 0.2, 0.0])
        q_pose2 = np.array([-0.2, -0.2, 0.4, 0.1, -0.2, 0.1])

        # Waypoint 1: stay at home
        pt0 = JointTrajectoryPoint()
        pt0.positions = q_home.tolist()
        pt0.time_from_start.sec = 0
        pt0.time_from_start.nanosec = 0

        # Waypoint 2: move to pose1 in 3 seconds
        pt1 = JointTrajectoryPoint()
        pt1.positions = q_pose1.tolist()
        pt1.time_from_start.sec = 3
        pt1.time_from_start.nanosec = 0

        # Waypoint 3: move to pose2 in 6 seconds
        pt2 = JointTrajectoryPoint()
        pt2.positions = q_pose2.tolist()
        pt2.time_from_start.sec = 6
        pt2.time_from_start.nanosec = 0

        # Waypoint 4: go back to home in 9 seconds
        pt3 = JointTrajectoryPoint()
        pt3.positions = q_home.tolist()
        pt3.time_from_start.sec = 9
        pt3.time_from_start.nanosec = 0

        traj.points.append(pt0)
        traj.points.append(pt1)
        traj.points.append(pt2)
        traj.points.append(pt3)

        self.traj_pub.publish(traj)
        self.get_logger().info("Test trajectory published.")


def main(args=None):
    rclpy.init(args=args)
    node = TestMovementNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
