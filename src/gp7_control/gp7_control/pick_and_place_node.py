import rclpy
from rclpy.node import Node
import numpy as np

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from gp7_control.kinematics import inverse_kinematics_position_only
from gp7_control.trajectory import trapezoidal_multi


class PickAndPlaceNode(Node):

    def __init__(self):
        super().__init__('gp7_pick_and_place')

        # TODO: replace with your actual joint names
        self.joint_names = [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6',
        ]

        self.pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',  # <-- use your real controller topic
            10
        )


        self.dt = 0.02      # 50 Hz trajectory
        self.vmax = 0.8     # rad/s
        self.amax = 1.0     # rad/s^2

        # Example poses (you must adjust for your actual box positions)
        # Box B is small box on the ground:
        # World coordinates (as in SDF)
        box_B_world = np.array([0.7, -0.4])
        box_A_world = np.array([0.3, 0.4])

        # Robot is yawed by pi in the world, so base = -world
        self.box_B_xy = -box_B_world
        self.box_A_xy = -box_A_world

        self.box_height_B = 0.05
        self.box_height_A = 0.2

        # fixed tool orientation to keep box upright
        self.ee_rpy = np.array([0.0, np.pi, 0.0])  # example: flip tool down

        self.timer = self.create_timer(3.0, self.run_sequence_once)
        self.done = False

    def run_sequence_once(self):
        if self.done:
            return
        self.done = True
        self.get_logger().info("Starting pick-and-place sequence")

        # 1. Define key cartesian poses
        z_clear = 0.3  # height above boxes
        z_grasp_B = self.box_height_B / 2.0 + 0.02  # slightly above mid-height
        z_place_A = self.box_height_A + self.box_height_B / 2.0 + 0.02

        # Positions
        pos_above_B = np.array([self.box_B_xy[0], self.box_B_xy[1], z_clear])
        pos_grasp_B = np.array([self.box_B_xy[0], self.box_B_xy[1], z_grasp_B])

        pos_above_A = np.array([self.box_A_xy[0], self.box_A_xy[1], z_clear])
        pos_place_A = np.array([self.box_A_xy[0], self.box_A_xy[1], z_place_A])

        # 2. Compute IK for each pose (keep orientation constant)
        waypoints_cart = [
            pos_above_B,
            pos_grasp_B,
            pos_above_B,
            pos_above_A,
            pos_place_A,
            pos_above_A,
        ]

        joint_waypoints = []
        q_current = np.zeros(6)

        for i, p in enumerate(waypoints_cart):
            self.get_logger().info(f"Computing IK (position only) for waypoint {i}: {p}")
            q_sol, ok = inverse_kinematics_position_only(p, q_init=q_current)
            if not ok:
                self.get_logger().error(f"IK (position-only) failed at waypoint {i}")
                return
            joint_waypoints.append(q_sol)
            q_current = q_sol


        # 3. Build full joint trajectory using trapezoidal profiles between each waypoint pair
        full_times = []
        full_q = []

        t_offset = 0.0
        q_prev = joint_waypoints[0]
        full_times.append(t_offset)
        full_q.append(q_prev)

        for i in range(1, len(joint_waypoints)):
            q_next = joint_waypoints[i]
            t_segment, q_segment = trapezoidal_multi(q_prev, q_next, self.vmax, self.amax, self.dt)

            # skip first sample to avoid duplicate time
            for k in range(1, len(t_segment)):
                full_times.append(t_offset + t_segment[k])
                full_q.append(q_segment[k])

            t_offset = full_times[-1]
            q_prev = q_next

        full_times = np.array(full_times)
        full_q = np.array(full_q)

        # 4. Publish as JointTrajectory
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        for t, q in zip(full_times, full_q):
            pt = JointTrajectoryPoint()
            pt.positions = q.tolist()
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
            traj.points.append(pt)

        self.get_logger().info("Publishing trajectory with %d points" % len(traj.points))
        self.pub.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# import rclpy
# from rclpy.node import Node
# import numpy as np

# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# from gp7_control.kinematics import inverse_kinematics_position_only
# from gp7_control.trajectory import trapezoidal_multi


# class PickAndPlaceNode(Node):

#     def __init__(self):
#         super().__init__('gp7_pick_and_place')

#         # Joint names must match your controller config
#         self.joint_names = [
#             'joint_1',
#             'joint_2',
#             'joint_3',
#             'joint_4',
#             'joint_5',
#             'joint_6',
#         ]

#         self.pub = self.create_publisher(
#             JointTrajectory,
#             '/arm_controller/joint_trajectory',
#             10
#         )

#         self.dt = 0.02      # 50 Hz trajectory
#         self.vmax = 0.8     # rad/s
#         self.amax = 1.0     # rad/s^2

#         # World coordinates (as in SDF)
#         box_B_world = np.array([0.9, -0.4])  # small box
#         box_A_world = np.array([0.3, 0.4])  # unused now but fine to keep

#         # Robot base is yawed by pi in the world, so base = -world
#         self.box_B_xy = -box_B_world
#         self.box_A_xy = -box_A_world

#         self.box_height_B = 0.05
#         self.box_height_A = 0.2

#         # fixed tool orientation (not used in position-only IK, but kept for future)
#         self.ee_rpy = np.array([0.0, np.pi, 0.0])

#         self.timer = self.create_timer(3.0, self.run_sequence_once)
#         self.done = False

#     def run_sequence_once(self):
#         if self.done:
#             return
#         self.done = True
#         self.get_logger().info("Starting move to pickup position")

#         # 1. Define key cartesian poses around Box B
#         z_clear = 0.3  # height above box B
#         z_grasp_B = self.box_height_B / 2.0 + 0.02  # slightly above mid-height

#         # Positions (in robot base frame)
#         pos_above_B = np.array([self.box_B_xy[0], self.box_B_xy[1], z_clear])
#         pos_grasp_B = np.array([self.box_B_xy[0], self.box_B_xy[1], z_grasp_B])

#         # 2. Only go to pickup: above B -> grasp B
#         waypoints_cart = [
#             pos_above_B,
#             # pos_grasp_B,
#         ]

#         joint_waypoints = []
#         q_current = np.zeros(6)

#         for i, p in enumerate(waypoints_cart):
#             self.get_logger().info(f"Computing IK (position only) for waypoint {i}: {p}")
#             q_sol, ok = inverse_kinematics_position_only(p, q_init=q_current)
#             if not ok:
#                 self.get_logger().error(f"IK (position-only) failed at waypoint {i}")
#                 return
#             joint_waypoints.append(q_sol)
#             q_current = q_sol

#         # 3. Build full joint trajectory using trapezoidal profiles between each waypoint pair
#         full_times = []
#         full_q = []

#         t_offset = 0.0
#         q_prev = joint_waypoints[0]
#         full_times.append(t_offset)
#         full_q.append(q_prev)

#         for i in range(1, len(joint_waypoints)):
#             q_next = joint_waypoints[i]
#             t_segment, q_segment = trapezoidal_multi(q_prev, q_next, self.vmax, self.amax, self.dt)

#             # skip first sample to avoid duplicate time
#             for k in range(1, len(t_segment)):
#                 full_times.append(t_offset + t_segment[k])
#                 full_q.append(q_segment[k])

#             t_offset = full_times[-1]
#             q_prev = q_next

#         full_times = np.array(full_times)
#         full_q = np.array(full_q)

#         # 4. Publish as JointTrajectory
#         traj = JointTrajectory()
#         traj.joint_names = self.joint_names

#         for t, q in zip(full_times, full_q):
#             pt = JointTrajectoryPoint()
#             pt.positions = q.tolist()
#             pt.time_from_start.sec = int(t)
#             pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
#             traj.points.append(pt)

#         self.get_logger().info("Publishing trajectory with %d points" % len(traj.points))
#         self.pub.publish(traj)


# def main(args=None):
#     rclpy.init(args=args)
#     node = PickAndPlaceNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()

