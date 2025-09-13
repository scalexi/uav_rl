import rclpy, numpy as np, gymnasium as gym, time
from gymnasium import spaces
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry
from sensor_msgs.msg import Image
from yolo_msgs.msg import DetectionArray
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation

class DroneEnv(gym.Env, Node):
    metadata = {'render_modes': ['human']}
    
    def __init__(self):
        Node.__init__(self, 'drone_env_node')
        gym.Env.__init__(self)
        
        self.target_altitude = 15.0
        self.max_episode_steps = 1000
        self.current_step = 0
        self.goal_pos = np.array([10.0, 10.0, 20.0])
        
        self.vel_z_min = -3.0
        self.vel_z_max = 3.0
        self.pos_z_min = 10.0
        self.pos_z_max = 30.0
        self.target_zone_min = 14.7
        self.target_zone_max = 15.3
        self.min_safe_dist = 0.5
        
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(4,), dtype=np.float32)
        self.max_vx = 0.5
        self.max_vy = 0.5
        self.max_vz = 4.0
        self.max_yaw_rate = 0.1
        
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(18,), dtype=np.float32)
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.offboard_control_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        
        self.odom_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos_profile)
        self.depth_sub = self.create_subscription(Image, '/depth_image', self.depth_callback, qos_profile)
        self.yolo_sub = self.create_subscription(Detection2DArray, '/yolo/detections', self.yolo_callback, qos_profile)
        
        self.drone_pos = np.zeros(3)
        self.drone_orientation_q = np.array([1.0, 0.0, 0.0, 0.0])
        self.drone_vel = np.zeros(3)
        self.drone_yaw = 0.0
        self.is_armed = False
        self.offboard_active = False
        
        self.depth_sectors = np.full(8, 10.0)
        self.closest_object = [10.0, 0.0]
        self.bridge = CvBridge()
        
        self.timer = self.create_timer(0.02, self.timer_callback)
        
        self.print_every_n_steps = 10
        self.step_counter = 0
        
        self.get_logger().info('DroneEnv initialized.')

    def odom_callback(self, msg):
        self.drone_pos = np.array(msg.position)
        self.drone_pos[2] = -self.drone_pos[2]
        self.drone_orientation_q = np.array(msg.q)
        rot = Rotation.from_quat([msg.q[1], msg.q[2], msg.q[3], msg.q[0]])
        self.drone_yaw = rot.as_euler('zyx')[0]
        self.drone_vel = np.array(msg.velocity)
        self.drone_vel[2] = -self.drone_vel[2]

    def depth_callback(self, msg):
        depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        depth_img = np.nan_to_num(depth_img, nan=10.0, posinf=10.0, neginf=10.0)
        depth_img = np.clip(depth_img, 0.1, 10.0)
        height, width = depth_img.shape
        sector_width = width // 8
        self.depth_sectors = np.array([
            np.min(depth_img[:, i*sector_width:(i+1)*sector_width]) 
            if np.any(depth_img[:, i*sector_width:(i+1)*sector_width] > 0) else 10.0 
            for i in range(8)
        ])
        if self.current_step % 50 == 0:
            self.get_logger().info(f"Depth Sectors: {self.depth_sectors}")

    def yolo_callback(self, msg):
        min_dist = float('inf')
        closest = None
        for detection in msg.detections:
            if detection.class_id == 0:  # 'person' in COCO dataset
                center_x = detection.bbox.center.position.x
                bbox_size = detection.bbox.size.x
                fov = 1.204  # Oak-D Lite RGB HFoV ~69°
                angle = (center_x / 960 - 0.5) * fov  # 1920/2 = 960
                dist = 0.2 / np.tan(bbox_size / 1920 * fov / 2)  # Person width ~0.2m
                if dist < min_dist:
                    min_dist = dist
                    closest = [dist, angle]
        self.closest_object = closest if closest else [10.0, 0.0]
        
    def timer_callback(self):
        if self.offboard_active:
            offboard_msg = OffboardControlMode()
            offboard_msg.position = False
            offboard_msg.velocity = True
            offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.offboard_control_pub.publish(offboard_msg)
    
    def publish_vehicle_command(self, command, **kwargs):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(kwargs.get("param1", 0.0))
        msg.param2 = float(kwargs.get("param2", 0.0))
        msg.param7 = float(kwargs.get("param7", 0.0))
        msg.target_system = 1
        msg.target_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)

    def print_drone_state(self, action_taken=None, reward=None, forced_action=False):
        pos_z = self.drone_pos[2]
        vel_z = self.drone_vel[2]
        goal_dist = np.linalg.norm(self.drone_pos - self.goal_pos)
        
        print(f"\n--- STEP {self.current_step} ---")
        print(f"Pos:     [{self.drone_pos[0]:+7.2f}, {self.drone_pos[1]:+7.2f}, {pos_z:+7.2f}] m")
        print(f"Vel:     [{self.drone_vel[0]:+7.2f}, {self.drone_vel[1]:+7.2f}, {vel_z:+7.2f}] m/s")
        print(f"Yaw:     {self.drone_yaw:+7.2f} rad")
        print(f"Goal Dist: {goal_dist:.2f} m")
        if action_taken is not None:
            print(f"Action:  [{action_taken[0]:+7.2f}, {action_taken[1]:+7.2f}, {action_taken[2]:+7.2f}, {action_taken[3]:+7.2f}]")
        if reward is not None:
            print(f"Reward:  {reward:+7.2f}")
        print(f"Min Depth: {np.min(self.depth_sectors):.2f} m")
        print(f"YOLO Dist: {self.closest_object[0]:.2f} m")
        if self.target_zone_min < pos_z < self.target_zone_max:
            print("🎯 IN TARGET ALTITUDE ZONE! 🎯")
        violations = []
        if vel_z > self.vel_z_max or vel_z < self.vel_z_min:
            violations.append(f"Vel_z out ({vel_z:.2f})")
        if pos_z < self.pos_z_min or pos_z > self.pos_z_max:
            violations.append(f"Pos_z out ({pos_z:.2f})")
        if violations:
            print(f"⚠️  Violations: {', '.join(violations)}")
        print("-" * 40)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.current_step = 0
        self.step_counter = 0
        
        if seed is not None:
            np.random.seed(seed)
        self.goal_pos = np.array([np.random.uniform(5, 15), np.random.uniform(5, 15), self.target_altitude])
        
        self.get_logger().info(f"--- New Episode: Goal at {self.goal_pos} ---")
        
        self.is_armed = False
        self.offboard_active = False
        self.use_position_control = True
        time.sleep(0.2)
        rclpy.spin_once(self, timeout_sec=0.1)
        initial_position = self.drone_pos.copy()
        
        self.get_logger().info("Starting offboard heartbeat...")
        self.offboard_active = True
        
        for i in range(25):
            traj_msg = TrajectorySetpoint()
            traj_msg.position = [initial_position[0], initial_position[1], -initial_position[2]]
            traj_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.trajectory_pub.publish(traj_msg)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)
        
        self.get_logger().info("Setting to Offboard mode...")
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0, param2=6.0
        )
        
        for i in range(15):
            traj_msg = TrajectorySetpoint()
            traj_msg.position = [initial_position[0], initial_position[1], -initial_position[2]]
            traj_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.trajectory_pub.publish(traj_msg)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)
        
        self.get_logger().info("Force arming vehicle...")
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0, param2=21196.0
        )
        
        for i in range(15):
            traj_msg = TrajectorySetpoint()
            traj_msg.position = [initial_position[0], initial_position[1], -initial_position[2]]
            traj_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.trajectory_pub.publish(traj_msg)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)
        
        self.is_armed = True
        self.get_logger().info("--- Arming Complete, Starting Takeoff ---")
        
        self.get_logger().info(f"Taking off to {self.target_altitude}m...")
        takeoff_start_time = time.time()
        takeoff_timeout = 30.0
        target_pos_ned = [initial_position[0], initial_position[1], -self.target_altitude]
        
        climb_steps = 50
        for step in range(climb_steps + 1):
            if time.time() - takeoff_start_time > takeoff_timeout:
                break
            progress = step / climb_steps
            current_target_alt = initial_position[2] + progress * (self.target_altitude - initial_position[2])
            current_pos_ned = [initial_position[0], initial_position[1], -current_target_alt]
            for _ in range(25):
                traj_msg = TrajectorySetpoint()
                traj_msg.position = current_pos_ned
                traj_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                self.trajectory_pub.publish(traj_msg)
                rclpy.spin_once(self, timeout_sec=0.01)
                time.sleep(0.02)
            if step % 10 == 0:
                self.get_logger().info(f"Takeoff progress: {self.drone_pos[2]:.1f}m / {self.target_altitude:.1f}m")
            if abs(self.drone_pos[2] - self.target_altitude) < 1.0:
                break
        
        self.get_logger().info("Stabilizing at target altitude...")
        for _ in range(50):
            traj_msg = TrajectorySetpoint()
            traj_msg.position = target_pos_ned
            traj_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.trajectory_pub.publish(traj_msg)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)
        
        self.get_logger().info("Switching to velocity control...")
        self.use_position_control = False
        for _ in range(25):
            traj_msg = TrajectorySetpoint()
            traj_msg.velocity = [0.0, 0.0, 0.0]
            traj_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.trajectory_pub.publish(traj_msg)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)
        
        rclpy.spin_once(self, timeout_sec=0.1)
        final_alt = self.drone_pos[2]
        if abs(final_alt - self.target_altitude) < 3.0:
            self.get_logger().info(f"Takeoff successful! Final altitude: {final_alt:.2f}m")
        else:
            self.get_logger().warning(f"Takeoff incomplete. Altitude: {final_alt:.2f}m")
        
        print(f"\n🚁 NEW EPISODE STARTED 🚁")
        self.print_drone_state()
        
        return self._get_observation(), {}

    def step(self, action):
        self.current_step += 1
        self.step_counter += 1
        
        if action[3] > 0.9:
            self.get_logger().info("Agent requested restart")
            self.print_drone_state(action_taken=action)
            return self._get_observation(), 0.0, True, False, {"restart_requested": True}
        
        vx = action[0] * self.max_vx
        vy = action[1] * self.max_vy
        vz = action[2] * self.max_vz
        yaw_rate = action[3] * self.max_yaw_rate
        
        forced_action = False
        if self.drone_vel[2] > self.vel_z_max:
            vz = -self.max_vz
            forced_action = True
        elif self.drone_vel[2] < self.vel_z_min:
            vz = self.max_vz
            forced_action = True
        
        setpoint_msg = TrajectorySetpoint()
        setpoint_msg.velocity = [vx, vy, -vz]
        setpoint_msg.yaw = yaw_rate
        setpoint_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(setpoint_msg)
        
        rclpy.spin_once(self, timeout_sec=0.1)
        
        observation = self._get_observation()
        reward = self._calculate_reward()
        
        if self.step_counter % self.print_every_n_steps == 0:
            self.print_drone_state(action_taken=action, reward=reward, forced_action=forced_action)
        
        terminated = self._check_termination()
        truncated = self.current_step >= self.max_episode_steps
        
        if terminated or truncated:
            print(f"\n🏁 EPISODE ENDED (Step {self.current_step}) 🏁")
            self.print_drone_state(action_taken=action, reward=reward, forced_action=forced_action)
            if terminated:
                print("Reason: Termination condition met")
            if truncated:
                print("Reason: Max episode steps reached")
            print("=" * 50)
        
        return observation, reward, terminated, truncated, {}

    def _calculate_reward(self):
        pos = self.drone_pos
        goal_dist = np.linalg.norm(pos - self.goal_pos)
        min_depth = np.min(self.depth_sectors)
        yolo_dist = self.closest_object[0]
        
        alt_reward = 1.0 if self.target_zone_min < pos[2] < self.target_zone_max else -0.2
        
        prev_goal_dist = getattr(self, '_prev_goal_dist', goal_dist)
        goal_reward = 0.3 * (prev_goal_dist - goal_dist)
        if goal_dist < 1.0:
            goal_reward += 100.0
        self._prev_goal_dist = goal_dist
        
        obs_reward = -0.5 * np.exp(-min_depth / 0.5) - 0.3 * np.exp(-yolo_dist / 0.5)
        
        collision_penalty = -300.0 if min_depth < self.min_safe_dist or yolo_dist < self.min_safe_dist else 0.0
        
        step_penalty = -0.01
        
        goal_vec = self.goal_pos - pos
        goal_angle = np.arctan2(goal_vec[1], goal_vec[0])
        yaw_error = np.abs((self.drone_yaw - goal_angle + np.pi) % (2 * np.pi) - np.pi)
        yaw_reward = -0.1 * yaw_error
        
        return alt_reward + goal_reward + obs_reward + collision_penalty + step_penalty + yaw_reward

    def _check_termination(self):
        if self.current_step < 50:
            return False
        pos_z = self.drone_pos[2]
        min_depth = np.min(self.depth_sectors)
        yolo_dist = self.closest_object[0]
        if pos_z < self.pos_z_min or pos_z > self.pos_z_max or pos_z < 0.5:
            self.get_logger().warning(f"Termination: Pos_z={pos_z:.2f} out of range")
            return True
        if min_depth < self.min_safe_dist or yolo_dist < self.min_safe_dist:
            self.get_logger().warning("Termination: Collision detected")
            return True
        if np.linalg.norm(self.drone_pos - self.goal_pos) < 1.0:
            self.get_logger().info("Termination: Goal reached")
            return True
        return False

    def _get_observation(self):
        pos = self.drone_pos
        vel = self.drone_vel
        vel[2] = np.clip(vel[2], self.vel_z_min, self.vel_z_max)
        goal_dist = np.linalg.norm(pos - self.goal_pos)
        goal_vec = self.goal_pos - pos
        goal_angle = np.arctan2(goal_vec[1], goal_vec[0])
        return np.concatenate([pos, vel, [self.drone_yaw], self.depth_sectors, self.closest_object, [goal_dist, goal_angle]], dtype=np.float32)

    def close(self):
        self.get_logger().info('Closing DroneEnv.')
        self.offboard_active = False
        if hasattr(self, 'timer'):
            self.timer.destroy()
        self.destroy_node()
