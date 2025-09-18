import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy

import numpy as np
import math
from collections import deque

# ROS2 Messages
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from px4_msgs.msg import TrajectorySetpoint, VehicleCommand, VehicleAttitude, VehicleStatus

class SafeDroneEnvironment(Node):
    def __init__(self):
        super().__init__('safe_drone_environment')
        
        # Set sim time parameter
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time',
            rclpy.Parameter.Type.BOOL,
            True
        )])
        
        # Environment parameters
        self.max_episode_steps = 1000
        self.current_step = 0
        self.target_altitude = 2.5  # Fixed Z altitude in ENU
        
        # State variables
        self.current_pose = None
        self.current_velocity = None
        self.current_angular_velocity = None  # New: From IMU/odometry
        self.current_attitude = None
        self.laser_data = None
        self.goal_position = np.array([11.0, 12.0])  # Updated to match reset
        self.previous_distance = None
        self.collision_threshold = 1.0  # Relaxed for RL (PX4 handles actual collision)
        
        # Action limits (velocity commands in m/s)
        self.max_velocity = 2.0
        self.max_position_step = 0.5  # Reduced for stability
        
        # Safety parameters
        self.max_tilt_angle = 30.0  # Maximum allowed tilt in degrees
        self.max_angular_rate = 1.0  # rad/s threshold for penalty
        self.emergency_land_altitude = 0.5  # Emergency landing altitude
        
        # Collision prevention parameters
        self.obstacle_threshold = 2.0
        self.repulsion_scale = 0.1
        
        # Current target position for smooth control (in ENU)
        self.target_position = None
        
        # Reward parameters - adjusted for position control with collision avoidance
        self.collision_penalty = -50.0  # Reduced since PX4 handles collision
        self.goal_reward = 100.0
        self.distance_reward_scale = 2.0  # Increased for better goal seeking
        self.velocity_penalty_scale = 0.005  # Reduced velocity penalty
        self.obstacle_avoidance_reward_scale = 0.2  # Reduced since PX4 handles this
        self.attitude_penalty_scale = 5.0  # Increased to discourage tilts
        self.angular_rate_penalty_scale = 0.5  # New: Penalize high rates
        
        # History for smoothing
        self.velocity_history = deque(maxlen=5)
        self.position_history = deque(maxlen=10)
        self.attitude_history = deque(maxlen=5)
        
        # Publishers
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_profile
        )
        
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            qos_profile
        )
        
        # Subscribers
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            qos_profile_sensor_data
        )
        
        self.vehicle_attitude_subscription = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            qos_profile_sensor_data
        )
        
        # Vehicle status subscriber for mode monitoring
        self.vehicle_status_subscription = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            qos_profile_sensor_data
        )
        
        self.current_mode = None  # To store current nav_state
        
        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_timer_callback)
        
        # State flags
        self.data_ready = False
        self.episode_done = False
        
        self.get_logger().info(f"Safe Drone Environment initialized with goal at ({self.goal_position[0]}, {self.goal_position[1]})")
    
    def vehicle_status_callback(self, msg: VehicleStatus):
        self.current_mode = msg.nav_state
        mode_str = {
            0: "Manual",
            1: "Altitude",
            2: "Position",
            3: "Mission",
            4: "Loiter",
            5: "RTL",
            10: "Acro",
            12: "Descend",
            13: "Termination",
            14: "Offboard",
            15: "Stabilized",
            17: "Takeoff",
            18: "Land",
            19: "Follow Target",
            20: "Precision Land",
            21: "Orbit",
            22: "VTOL Takeoff"
        }.get(msg.nav_state, "Unknown")
        self.get_logger().info(f"Current flight mode: {mode_str} (nav_state: {msg.nav_state})")
    
    def reset(self, goal_position=None):
        """Reset environment for new episode"""
        self.current_step = 0
        self.episode_done = False
        self.previous_distance = None
        self.velocity_history.clear()
        self.position_history.clear()
        self.attitude_history.clear()
        
        if goal_position is not None:
            self.goal_position = np.array(goal_position)
        else:
            self.goal_position = np.array([11.0, 12.0])
        
        # Reset target position to takeoff position (ENU)
        self.target_position = np.array([0.0, 0.0, self.target_altitude])
        
        # Wait for initial data
        start_time = self.get_clock().now()
        while not self.data_ready and (self.get_clock().now() - start_time).nanoseconds / 1e9 < 10.0:  # Timeout 10s
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not self.data_ready:
            self.get_logger().error("Reset failed: Data not ready")
            return None
        
        # Publish initial setpoints continuously BEFORE mode switch
        for _ in range(20):  # Publish at ~10Hz for 2s
            self.publish_position_command()
            rclpy.spin_once(self, timeout_sec=0.05)
        
        # Arm the drone
        self.send_arm_command()
        
        # Wait for armed state and initial mode update
        start_time = self.get_clock().now()
        while self.current_mode is None:
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > 5.0:
                self.get_logger().error("Failed to receive vehicle status")
                return None
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Set Offboard mode
        self.enable_offboard_mode()
        
        # Wait for mode switch to Offboard
        start_time = self.get_clock().now()
        while self.current_mode != 14:
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > 5.0:
                self.get_logger().error("Failed to switch to offboard mode")
                return None
            self.enable_offboard_mode()  # Resend
            self.publish_position_command()  # Keep publishing
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Wait for takeoff to complete (z >= 2.0)
        start_time = self.get_clock().now()
        while self.current_pose is None or self.current_pose[2] < 2.0:
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > 20.0:  # Timeout 20s
                self.get_logger().error("Takeoff timeout - episode reset failed")
                self.emergency_land()
                return None
            self.publish_position_command()  # Keep publishing during wait
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Wait for stable attitude after takeoff (tilt < 10 deg)
        start_time = self.get_clock().now()
        while not self.check_attitude_safety() or max(abs(math.degrees(a)) for a in self.current_attitude[:2]) > 10.0:
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > 5.0:
                self.get_logger().error("Attitude stabilization timeout")
                self.emergency_land()
                return None
            self.publish_position_command()
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().info(f"Takeoff complete - altitude: {self.current_pose[2]:.2f}m, Goal: ({self.goal_position[0]}, {self.goal_position[1]})")
        
        # Reset target to current pose for start
        self.target_position = self.current_pose.copy()
        
        return self.get_state()
    
    def odom_callback(self, msg: Odometry):
        """Callback for odometry data"""
        self.current_pose = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
        self.current_velocity = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])
        
        self.current_angular_velocity = np.array([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ])  # New: IMU-derived angular rates
        
        # Initialize target position if not set
        if self.target_position is None:
            self.target_position = self.current_pose.copy()
        
        # Store velocity history for smoothing
        self.velocity_history.append(np.linalg.norm(self.current_velocity[:2]))
        self.position_history.append(self.current_pose[:2].copy())
        
        self._check_data_ready()
    
    def laser_callback(self, msg: LaserScan):
        """Callback for laser scan data"""
        # Convert laser scan to numpy array, handle inf values
        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges)] = msg.range_max
        ranges[np.isnan(ranges)] = msg.range_max
        
        # Store laser data
        self.laser_data = {
            'ranges': ranges,
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment,
            'range_max': msg.range_max
        }
        
        self._check_data_ready()
    
    def attitude_callback(self, msg: VehicleAttitude):
        """Callback for vehicle attitude"""
        # Extract Euler angles (roll, pitch, yaw) from quaternion
        q = msg.q  # [w, x, y, z]
        sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3])
        cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2 * (q[0] * q[2] - q[3] * q[1])
        pitch = math.asin(sinp)
        
        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.current_attitude = np.array([roll, pitch, yaw])
        
        self.attitude_history.append(self.current_attitude.copy())
        
        self._check_data_ready()
    
    def _check_data_ready(self):
        """Check if all required data is available"""
        ready = (
            self.current_pose is not None and
            self.current_velocity is not None and
            self.current_angular_velocity is not None and
            self.current_attitude is not None and
            self.laser_data is not None
        )
        
        if ready and not self.data_ready:
            self.data_ready = True
            self.get_logger().info("All sensor data ready")
    
    def get_state(self):
        """Get current state for RL agent"""
        if not self.data_ready:
            return None
        
        # Relative position to goal
        rel_pos = self.goal_position - self.current_pose[:2]
        dist_to_goal = np.linalg.norm(rel_pos)
        
        # Normalized direction to goal
        if dist_to_goal > 0:
            dir_to_goal = rel_pos / dist_to_goal
        else:
            dir_to_goal = np.zeros(2)
        
        # Current velocity in XY
        vel_xy = self.current_velocity[:2]
        
        # Attitude (roll, pitch)
        att_rp = self.current_attitude[:2] if self.current_attitude is not None else np.zeros(2)
        
        # Process laser data into features (e.g., min distances in sectors)
        laser_features = self.process_laser_data()
        
        # Concatenate state
        state = np.concatenate([
            rel_pos,          # 2
            [dist_to_goal],   # 1
            dir_to_goal,      # 2
            vel_xy,           # 2
            att_rp,           # 2
            laser_features    # e.g., 8 sectors
        ])
        
        return state.astype(np.float32)
    
    def process_laser_data(self):
        """Process laser ranges into features (min distances in 8 sectors)"""
        if self.laser_data is None:
            return np.full(8, self.collision_threshold * 2)
        
        ranges = self.laser_data['ranges']
        num_sectors = 8
        sector_size = len(ranges) // num_sectors
        features = np.zeros(num_sectors)
        
        for i in range(num_sectors):
            sector = ranges[i * sector_size : (i + 1) * sector_size]
            features[i] = np.min(sector) if len(sector) > 0 else self.laser_data['range_max']
        
        return features
    
    def adjust_target_for_obstacles(self):
        """Adjust target position to avoid obstacles using laser data"""
        if self.laser_data is None or self.current_attitude is None:
            return
        
        ranges = self.laser_data['ranges']
        angle_min = self.laser_data['angle_min']
        angle_inc = self.laser_data['angle_increment']
        repulsion = np.zeros(2)
        
        for i, r in enumerate(ranges):
            if r < self.obstacle_threshold:
                angle_body = angle_min + i * angle_inc
                angle_world = angle_body + self.current_attitude[2]
                obs_dir_x = math.cos(angle_world)
                obs_dir_y = math.sin(angle_world)
                force = (self.obstacle_threshold - r) / r
                repulsion[0] += -obs_dir_x * force
                repulsion[1] += -obs_dir_y * force
        
        self.target_position[:2] += repulsion * self.repulsion_scale
    
    def step(self, action):
        """Execute action and get next state, reward, done"""
        if not self.data_ready:
            return None, 0.0, False, {}
        
        self.current_step += 1
        
        # Scale and clip action (dx, dy in ENU)
        action = np.clip(action, -1.0, 1.0) * self.max_position_step
        self.target_position[:2] += action
        
        # Limit target position step for stability
        pos_diff = self.target_position[:2] - self.current_pose[:2]
        pos_diff_mag = np.linalg.norm(pos_diff)
        if pos_diff_mag > self.max_position_step:
            self.target_position[:2] = self.current_pose[:2] + (pos_diff / pos_diff_mag) * self.max_position_step
        
        # Adjust for obstacles
        self.adjust_target_for_obstacles()
        
        # Publish new setpoint
        self.publish_position_command()
        
        # Get reward
        reward = self.calculate_reward(action)
        
        # Check done
        done = self.is_episode_done()
        
        # Get next state
        next_state = self.get_state()
        
        # Info dict
        info = {
            'distance_to_goal': np.linalg.norm(self.current_pose[:2] - self.goal_position),
            'attitude_safe': self.check_attitude_safety()
        }
        
        return next_state, reward, done, info
    
    def calculate_reward(self, action):
        """Calculate reward based on state and action"""
        reward = 0.0
        
        # Distance to goal reward
        current_distance = np.linalg.norm(self.current_pose[:2] - self.goal_position)
        
        if self.previous_distance is not None:
            # Reward for getting closer to goal
            distance_improvement = self.previous_distance - current_distance
            reward += distance_improvement * self.distance_reward_scale
        
        self.previous_distance = current_distance
        
        # Goal reached reward
        if current_distance < 1.0:  # Within 1 meter of goal
            reward += self.goal_reward
        
        # Altitude bonus/penalty
        alt_error = abs(self.current_pose[2] - self.target_altitude)
        if alt_error < 0.2:
            reward += 0.1
        else:
            reward -= 0.1 * alt_error
        
        # Attitude penalty - penalize excessive tilt
        if self.current_attitude is not None:
            roll_deg = abs(math.degrees(self.current_attitude[0]))
            pitch_deg = abs(math.degrees(self.current_attitude[1]))
            max_tilt = max(roll_deg, pitch_deg)
            
            if max_tilt > self.max_tilt_angle:
                tilt_penalty = (max_tilt - self.max_tilt_angle) / self.max_tilt_angle
                reward -= tilt_penalty * self.attitude_penalty_scale
        
        # Angular rate penalty - penalize high rates from IMU
        if self.current_angular_velocity is not None:
            angular_magnitude = np.linalg.norm(self.current_angular_velocity)
            if angular_magnitude > self.max_angular_rate:
                rate_penalty = (angular_magnitude - self.max_angular_rate) / self.max_angular_rate
                reward -= rate_penalty * self.angular_rate_penalty_scale
        
        # Obstacle avoidance reward (reduced since PX4 handles collision)
        min_obstacle_distance = np.min(self.process_laser_data())
        if min_obstacle_distance > 0:
            # Small reward for maintaining reasonable distance
            if min_obstacle_distance < 2.0:  # Close to obstacle
                safety_reward = (min_obstacle_distance / 2.0) * self.obstacle_avoidance_reward_scale
                reward += safety_reward
        
        # Collision penalty (if we somehow get too close despite PX4 protection)
        if min_obstacle_distance < 0.5:
            reward += self.collision_penalty
        
        # Small velocity penalty for efficiency
        velocity_magnitude = np.linalg.norm(action)
        reward -= velocity_magnitude * self.velocity_penalty_scale
        
        # Efficiency penalty
        reward -= 0.01
        
        return reward
    
    def check_attitude_safety(self):
        """Check if drone attitude is safe (not tilted too much)"""
        if self.current_attitude is None:
            return True
        
        roll_deg = abs(math.degrees(self.current_attitude[0]))
        pitch_deg = abs(math.degrees(self.current_attitude[1]))
        max_tilt = max(roll_deg, pitch_deg)
        
        return max_tilt < self.max_tilt_angle
    
    def is_episode_done(self):
        """Check if episode should end"""
        if not self.data_ready:
            return False
        
        # Episode ends if:
        # 1. Maximum steps reached
        if self.current_step >= self.max_episode_steps:
            return True
        
        # 2. Goal reached
        distance_to_goal = np.linalg.norm(self.current_pose[:2] - self.goal_position)
        if distance_to_goal < 1.0:
            return True
        
        # 3. Drone too far from goal (safety)
        if distance_to_goal > 50.0:
            return True
        
        # 4. Unsafe attitude (excessive tilt)
        if not self.check_attitude_safety():
            self.get_logger().warn("Episode ended due to unsafe attitude")
            return True
        
        # 5. Altitude too low (potential crash)
        if self.current_pose[2] < self.emergency_land_altitude:
            self.get_logger().warn("Episode ended due to low altitude")
            return True
        
        return False
    
    def publish_position_command(self):
        """Publish position command to PX4 with collision avoidance"""
        if not self.data_ready or self.target_position is None:
            return
        
        # Adjust for obstacles
        self.adjust_target_for_obstacles()
        
        # Send position setpoint in NED
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.position[0] = float(self.target_position[1])  # north
        trajectory_msg.position[1] = float(self.target_position[0])  # east
        trajectory_msg.position[2] = float(-self.target_altitude)  # down, fixed altitude
        
        # Optional: Set velocity feedforward for smoother motion
        if len(self.position_history) >= 2:
            dt = 0.1  # Control timestep
            vel_x_enu = (self.position_history[-1][0] - self.position_history[-2][0]) / dt  # east
            vel_y_enu = (self.position_history[-1][1] - self.position_history[-2][1]) / dt  # north
            trajectory_msg.velocity[0] = np.clip(vel_y_enu, -self.max_velocity, self.max_velocity)  # north
            trajectory_msg.velocity[1] = np.clip(vel_x_enu, -self.max_velocity, self.max_velocity)  # east
            trajectory_msg.velocity[2] = 0.0
        
        # Set yaw to face movement direction
        if len(self.position_history) >= 2:
            dx = self.position_history[-1][0] - self.position_history[-2][0]
            dy = self.position_history[-1][1] - self.position_history[-2][1]
            if abs(dx) > 0.1 or abs(dy) > 0.1:
                desired_yaw = math.atan2(dx, dy)
                trajectory_msg.yaw = desired_yaw
        
        trajectory_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(trajectory_msg)
    
    def send_arm_command(self):
        """Send arm command"""
        cmd = VehicleCommand()
        cmd.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        cmd.param1 = 1.0  # 1 to arm
        cmd.param2 = 0.0
        cmd.param3 = 0.0
        cmd.param4 = 0.0
        cmd.param5 = 0.0
        cmd.param6 = 0.0
        cmd.param7 = 0.0
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(cmd)
        
        self.get_logger().info("Arm command sent")
    
    def enable_offboard_mode(self):
        """Enable PX4 Offboard mode"""
        cmd = VehicleCommand()
        cmd.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        cmd.param1 = 1.0  # Base mode (custom enabled)
        cmd.param2 = 6.0  # Custom main mode: Offboard
        cmd.param3 = 0.0
        cmd.param4 = 0.0
        cmd.param5 = 0.0
        cmd.param6 = 0.0
        cmd.param7 = 0.0
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(cmd)
        
        self.get_logger().info("Offboard mode enabled")
    
    def emergency_land(self):
        """Emergency landing procedure"""
        cmd = VehicleCommand()
        cmd.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        cmd.param1 = 0.0  # Unused or descent rate
        cmd.param2 = 0.0  # Unused
        cmd.param3 = 0.0  # Unused
        cmd.param4 = 0.0  # Unused
        cmd.param5 = 0.0  # Unused
        cmd.param6 = 0.0  # Unused
        cmd.param7 = 0.0  # Unused
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(cmd)
        
        self.get_logger().warn("Emergency landing initiated!")
    
    def control_timer_callback(self):
        """Timer callback for safety monitoring"""
        if self.data_ready and self.current_pose is not None:
            # Safety checks
            if not self.check_attitude_safety():
                self.get_logger().warn("Unsafe attitude detected - consider emergency landing")
                # Optionally trigger emergency landing
                # self.emergency_land()
            
            # Check if drone is responding to commands
            if (self.target_position is not None and 
                np.linalg.norm(self.current_pose[:2] - self.target_position[:2]) > 5.0):
                self.get_logger().warn("Drone far from target position - possible control issue")
            
            self.publish_position_command()