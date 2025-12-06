import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import numpy as np

from control_msgs.action import GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

from geometry_msgs.msg import PointStamped


from gp7_control.kinematics import inverse_kinematics_position_only, forward_kinematics
from gp7_control.trajectory import trapezoidal_multi


class PickAndPlaceNode(Node):

    def __init__(self):
        super().__init__('gp7_pick_and_place')

        # Arm joint names (6 DOF)
        self.arm_joint_names = [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6',
        ]

        # Gripper joint names (2 prismatic fingers)
        self.gripper_joint_names = ['joint_7', 'joint_8']

        # Publisher for arm trajectory
        self.arm_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )
        
        # Publisher for gripper commands (direct position control)
        self.gripper_pub = self.create_publisher(
            JointTrajectory,
            '/gripper_controller/joint_trajectory',
            10
        )
        
        # Subscribe to joint states to get current position
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Publisher for end-effector position (for visualization / debugging)
        self.ee_pos_pub = self.create_publisher(
            PointStamped,
            '/ee_position',
            10
        )

        self.current_joint_positions = None
        self.dt = 0.02      # 50 Hz trajectory
        self.vmax = 0.8     # rad/s for arm
        self.amax = 1.0     # rad/s^2 for arm

        # Gripper parameters
        self.gripper_open_position = 0.04   # Open: fingers apart
        self.gripper_close_position = -0.01  # Closed: fingers together (to grip 0.05m box)

        # World coordinates (as in SDF)
        box_B_world = np.array([0.7, -0.4])  # Small red box
        box_A_world = np.array([0.3, 0.4])   # Large blue box

        # Robot base is yawed by pi in the world, so base = -world
        self.box_B_xy = -box_B_world
        self.box_A_xy = -box_A_world

        self.box_height_B = 0.05  # Small box height
        self.box_height_A = 0.2   # Large box height

        # Fixed tool orientation (for future full pose IK)
        self.ee_rpy = np.array([0.0, np.pi, 0.0])

        self.timer = self.create_timer(5.0, self.run_full_pick_and_place)
        self.done = False


    def publish_ee_position(self, q, note: str = ""):
        """
        Compute forward kinematics for joint positions q and 
        publish/log the end-effector position.
        """
        try:
            p = forward_kinematics(q)  # whatever your FK returns
        except Exception as e:
            self.get_logger().error(f"FK failed: {e}")
            return

        # Make sure p is a simple 1D array of floats: [x, y, z]
        p = np.array(p, dtype=float).reshape(-1)

        if p.size < 3:
            self.get_logger().error(f"FK returned unexpected shape: {p.shape}, value: {p}")
            return

        x, y, z = float(p[0]), float(p[1]), float(p[2])

        # Log to console
        if note:
            self.get_logger().info(
                f"{note} EE position: x={x:.3f}, y={y:.3f}, z={z:.3f}"
            )
        else:
            self.get_logger().info(
                f"EE position: x={x:.3f}, y={y:.3f}, z={z:.3f}"
            )

        # Publish as PointStamped
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'  # or your base frame

        msg.point.x = x
        msg.point.y = y
        msg.point.z = z

        self.ee_pos_pub.publish(msg)



    def joint_state_callback(self, msg):
        """Store current joint positions"""
        try:
            # Extract positions for our controlled joints
            positions = []
            for name in self.arm_joint_names:
                idx = msg.name.index(name)
                positions.append(msg.position[idx])
            self.current_joint_positions = np.array(positions)
            # self.publish_ee_position(self.current_joint_positions,
            #                          note="[JOINT STATE]")

        except (ValueError, IndexError) as e:
            self.get_logger().warn(f"Could not find joint in state: {e}")

    def create_oneshot_timer(self, delay, callback):
        """Create a timer that fires once and then destroys itself"""
        def oneshot_wrapper():
            callback()
            timer.cancel()
            self.destroy_timer(timer)
        
        timer = self.create_timer(delay, oneshot_wrapper)
        return timer

    def send_arm_trajectory(self, joint_waypoints):
        """
        Build and send arm trajectory from joint waypoints.
        Returns the total trajectory time.
        """
        # Build full joint trajectory using trapezoidal profiles
        full_times = []
        full_q = []

        t_offset = 0.0
        q_prev = joint_waypoints[0]
        full_times.append(t_offset)
        full_q.append(q_prev)

        for i in range(1, len(joint_waypoints)):
            q_next = joint_waypoints[i]
            t_segment, q_segment = trapezoidal_multi(q_prev, q_next, self.vmax, self.amax, self.dt)

            # Skip first sample to avoid duplicate time
            for k in range(1, len(t_segment)):
                full_times.append(t_offset + t_segment[k])
                full_q.append(q_segment[k])

            t_offset = full_times[-1]
            q_prev = q_next

        full_times = np.array(full_times)
        full_q = np.array(full_q)

        # Publish as JointTrajectory
        traj = JointTrajectory()
        traj.joint_names = self.arm_joint_names

        for t, q in zip(full_times, full_q):
            pt = JointTrajectoryPoint()
            pt.positions = q.tolist()
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
            traj.points.append(pt)

        self.get_logger().info(f"Publishing arm trajectory with {len(traj.points)} points over {full_times[-1]:.2f}s")
        self.arm_pub.publish(traj)
        
        return full_times[-1] if len(full_times) > 0 else 0.0

    def send_gripper_command(self, position, duration=1.0):
        """
        Send gripper command to open or close.
        position: target position for both gripper joints
        duration: time to complete the motion (seconds)
        """
        traj = JointTrajectory()
        traj.joint_names = self.gripper_joint_names

        pt = JointTrajectoryPoint()
        # Both gripper joints move symmetrically
        pt.positions = [position, -position]  # joint_7 and joint_8 move opposite
        pt.time_from_start.sec = int(duration)
        pt.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        traj.points.append(pt)

        self.get_logger().info(f"Sending gripper command: position={position:.3f}")
        self.gripper_pub.publish(traj)

    def compute_cartesian_to_joint_waypoints(self, cartesian_waypoints):
        """
        Convert list of cartesian positions to joint configurations using IK.
        Returns list of joint configurations or None if IK fails.
        """
        joint_waypoints = []
        q_current = self.current_joint_positions.copy()

        for i, p in enumerate(cartesian_waypoints):
            self.get_logger().info(f"Computing IK for waypoint {i}: {p}")
            q_sol, ok = inverse_kinematics_position_only(p, q_init=q_current)
            if not ok:
                self.get_logger().error(f"IK failed at waypoint {i}")
                return None
            joint_waypoints.append(q_sol)
            q_current = q_sol

        return joint_waypoints
    
    


    def run_full_pick_and_place(self):
        """Execute the full pick and place sequence with gripper control"""
        if self.done:
            return
        
        # Wait for joint state
        if self.current_joint_positions is None:
            self.get_logger().info("Waiting for joint states...")
            return
            
        self.done = True
        self.get_logger().info("=" * 50)
        self.get_logger().info("Starting FULL pick-and-place sequence")
        self.get_logger().info("=" * 50)

        # Define heights
        z_clear = 0.05  # Safe height above boxes
        z_grasp_B = self.box_height_B + 0.02  # Slightly above small box
        z_place_A = self.box_height_A + self.box_height_B / 2.0 + 0.02  # Place on top of large box

        # ============================================
        # PHASE 1: Move to position above Box B
        # ============================================
        self.get_logger().info("\n[PHASE 1] Moving to position above Box B")
        pos_above_B = np.array([self.box_B_xy[0], self.box_B_xy[1], z_clear])
        
        waypoints = self.compute_cartesian_to_joint_waypoints([pos_above_B])
        if waypoints is None:
            return
        print(waypoints)
        duration = self.send_arm_trajectory(waypoints)
        
        # ============================================
        # PHASE 2: Open gripper before descending
        # ============================================
        # def phase2():
        #     self.get_logger().info("\n[PHASE 2] Opening gripper")
        #     self.send_gripper_command(self.gripper_open_position, 1.0)
        #     # Schedule phase 3
        #     self.create_oneshot_timer(2.0, lambda: self.get_logger().info("\n✓ Pick-and-place sequence COMPLETE!"))
        
        # self.create_oneshot_timer(duration + 0.5, phase2)
        
        # # ============================================
        # # PHASE 3: Descend to grasp position
        # # ============================================
        # def phase3():
        #     self.get_logger().info("\n[PHASE 3] Descending to grasp Box B")
        #     pos_grasp_B = np.array([self.box_B_xy[0], self.box_B_xy[1], z_grasp_B])
        #     waypoints = self.compute_cartesian_to_joint_waypoints([pos_grasp_B])
        #     if waypoints:
        #         duration = self.send_arm_trajectory(waypoints)
        #         # Schedule phase 4
        #         self.create_oneshot_timer(duration + 0.5, phase4)
        
        # # ============================================
        # # PHASE 4: Close gripper to grasp box
        # # ============================================
        # def phase4():
        #     self.get_logger().info("\n[PHASE 4] Closing gripper to grasp Box B")
        #     self.send_gripper_command(self.gripper_close_position, 1.0)
        #     # Schedule phase 5
        #     self.create_oneshot_timer(1.5, phase5)
        
        # # ============================================
        # # PHASE 5: Lift box to safe height
        # # ============================================
        # def phase5():
        #     self.get_logger().info("\n[PHASE 5] Lifting Box B to safe height")
        #     pos_lift_B = np.array([self.box_B_xy[0], self.box_B_xy[1], z_clear])
        #     waypoints = self.compute_cartesian_to_joint_waypoints([pos_lift_B])
        #     if waypoints:
        #         duration = self.send_arm_trajectory(waypoints)
        #         # Schedule phase 6
        #         self.create_oneshot_timer(duration + 0.5, phase6)
        
        # # ============================================
        # # PHASE 6: Move to position above Box A
        # # ============================================
        # def phase6():
        #     self.get_logger().info("\n[PHASE 6] Moving to position above Box A")
        #     pos_above_A = np.array([self.box_A_xy[0], self.box_A_xy[1], z_clear])
        #     waypoints = self.compute_cartesian_to_joint_waypoints([pos_above_A])
        #     if waypoints:
        #         duration = self.send_arm_trajectory(waypoints)
        #         # Schedule phase 7
        #         self.create_oneshot_timer(duration + 0.5, phase7)
        
        # # ============================================
        # # PHASE 7: Descend to place position
        # # ============================================
        # def phase7():
        #     self.get_logger().info("\n[PHASE 7] Descending to place Box B on Box A")
        #     pos_place_A = np.array([self.box_A_xy[0], self.box_A_xy[1], z_place_A])
        #     waypoints = self.compute_cartesian_to_joint_waypoints([pos_place_A])
        #     if waypoints:
        #         duration = self.send_arm_trajectory(waypoints)
        #         # Schedule phase 8
        #         self.create_oneshot_timer(duration + 0.5, phase8)
        
        # # ============================================
        # # PHASE 8: Open gripper to release box
        # # ============================================
        # def phase8():
        #     self.get_logger().info("\n[PHASE 8] Opening gripper to release Box B")
        #     self.send_gripper_command(self.gripper_open_position, 1.0)
        #     # Schedule phase 9
        #     self.create_oneshot_timer(1.5, phase9)
        
        # # ============================================
        # # PHASE 9: Retract to safe height
        # # ============================================
        # def phase9():
        #     self.get_logger().info("\n[PHASE 9] Retracting to safe height")
        #     pos_retract = np.array([self.box_A_xy[0], self.box_A_xy[1], z_clear])
        #     waypoints = self.compute_cartesian_to_joint_waypoints([pos_retract])
        #     if waypoints:
        #         duration = self.send_arm_trajectory(waypoints)
        #         self.create_oneshot_timer(duration + 0.5, 
        #                                 lambda: self.get_logger().info("\n✓ Pick-and-place sequence COMPLETE!"))


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()