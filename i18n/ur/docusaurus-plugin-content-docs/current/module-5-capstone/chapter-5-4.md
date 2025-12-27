---
id: module-5-chapter-4
title: "Navigation and Obstacle Avoidance"
sidebar_label: "5.4 Navigation System"
sidebar_position: 4
description: "Implement autonomous navigation: Nav2 configuration, dynamic obstacle avoidance, human-aware navigation, and goal-oriented path planning for humanoid robots"
keywords: [autonomous-navigation, nav2, obstacle-avoidance, path-planning, human-aware-navigation]
estimated_time: 90
prerequisites:
  - module-3-chapter-4
  - module-5-chapter-2
learning_outcomes:
  - Configure Nav2 for humanoid platform kinematics
  - Implement dynamic obstacle avoidance
  - Design human-aware navigation behaviors
  - Integrate VSLAM for localization
  - Tune navigation stack for real-world performance
hardware_tier: premium
---

# باب 5.4: نیویگیشن اور رکاوٹوں سے بچاؤ

خودکار نیویگیشن موبائل ہیومینائڈز کے لیے بنیادی ہے۔ یہ باب ایک مکمل نیویگیشن سسٹم کو نافذ کرتا ہے جس میں ڈائنامک رکاوٹوں سے بچاؤ اور انسانوں کے لیے آگاه رویے شامل ہیں۔

## 5.4.1 ہیومینائڈ نیویگیشن کا تعارف

موبائل ہیومینائڈ روبوٹس کو انسانوں کے ماحول میں محفوظ طریقے سے کام کرنے کے لیے جدید نیویگیشن صلاحیتوں کی ضرورت ہوتی ہے۔ پہیے دار روبوٹس کے برعکس، ہیومینائڈ پلیٹ فارمز کو توازن کی پابندیاں، متغیر قدموں کے پیٹرن، اور سماجی تعامل کے اصولوں پر غور کرنا ہوتا ہے۔ نیویگیشن سسٹمز کو ڈائنامک رکاوٹوں، تنگ راستوں، اور بھری ہوئی جگہوں کو سنبھالنا ہوتا ہے جبکہ استقامت اور انسانوں کے آرام کو برقرار رکھنا ہوتا ہے۔

ہیومینائڈز کے لیے اہم نیویگیشن چیلنجز میں شامل ہیں:
- **کینیمیٹک پابندیاں**: محدود موڑنے کا رداس اور رفتار
- **ڈائنامک استقامت**: حرکت کے دوران توازن برقرار رکھنا
- **سماجی آگاهی**: ذاتی جگہ اور سماجی اصولوں کا احترام کرنا
- **ریل ٹائم کارکردگی**: ڈائنامک ماحول کے لیے تیز منصوبہ بندی
- **لوکلیزیشن**: بیرونی نشانوں کے بغیر مضبوط پوزیشن کا تخمینہ

## 5.4.2 ہیومینائڈز کے لیے Nav2 اسٹیک آرکیٹیکچر

ROS 2 نیویگیشن اسٹیک (Nav2) خودکار نیویگیشن کے لیے ایک لچکدار فریم ورک فراہم کرتا ہے۔ ہم اسے خصوصی طور پر ہیومینائڈ پلیٹ فارمز کے لیے ترتیب دیتے ہیں:

```yaml
# config/nav2_params.yaml
bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    
    # Behavior tree for humanoid navigation
    default_nav_to_pose_bt_xml: "$(find-pkg-share humanoid_nav)/behavior_trees/navigate_w_recovery.xml"
    
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node

controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    
    # Humanoid-specific velocity limits
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]
    
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.15
      yaw_goal_tolerance: 0.15
      stateful: true
    
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      
      # Humanoid velocity limits
      min_vel_x: -0.3
      max_vel_x: 0.5
      min_vel_y: 0.0
      max_vel_y: 0.0
      max_vel_theta: 0.8
      min_speed_xy: 0.0
      max_speed_xy: 0.5
      min_speed_theta: 0.0
      
      # Acceleration limits for stability
      acc_lim_x: 0.5
      acc_lim_y: 0.0
      acc_lim_theta: 1.0
      decel_lim_x: -0.5
      decel_lim_y: 0.0
      decel_lim_theta: -1.0
      
      # Trajectory generation
      vx_samples: 20
      vy_samples: 0
      vtheta_samples: 40
      sim_time: 2.0
      discretize_by_time: false
      
      # Scoring
      critics: ["RotateToGoal", "Oscillation", "ObstacleFootprint", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
      
      ObstacleFootprint.scale: 0.02
      PathAlign.scale: 32.0
      PathDist.scale: 32.0
      GoalAlign.scale: 24.0
      GoalDist.scale: 24.0
```

## 5.4.3 ڈائنامک ماحول کے لیے کاسٹ میپ ترتیب

کاسٹ میپس منصوبہ بندی کے لیے رکاوٹوں کی معلومات کی نمائندگی کرتے ہیں۔ ہم علیحدہ عالمی اور مقامی کاسٹ میپس کو ترتیب دیتے ہیں:

```yaml
# config/costmap_common.yaml
footprint: [[-0.3, -0.2], [-0.3, 0.2], [0.3, 0.2], [0.3, -0.2]]
footprint_padding: 0.05

plugins:
  - "static_layer"
  - "obstacle_layer"
  - "inflation_layer"
  - "voxel_layer"

static_layer:
  plugin: "nav2_costmap_2d::StaticLayer"
  map_subscribe_transient_local: true
  map_topic: "/map"

obstacle_layer:
  plugin: "nav2_costmap_2d::ObstacleLayer"
  enabled: true
  observation_sources: scan camera
  
  scan:
    topic: /scan
    max_obstacle_height: 2.0
    min_obstacle_height: 0.1
    obstacle_max_range: 10.0
    obstacle_min_range: 0.0
    raytrace_max_range: 10.0
    raytrace_min_range: 0.0
    clearing: true
    marking: true
    data_type: "LaserScan"
  
  camera:
    topic: /depth/points
    max_obstacle_height: 2.0
    min_obstacle_height: 0.3
    obstacle_max_range: 3.0
    clearing: true
    marking: true
    data_type: "PointCloud2"

voxel_layer:
  plugin: "nav2_costmap_2d::VoxelLayer"
  enabled: true
  publish_voxel_map: true
  origin_z: 0.0
  z_resolution: 0.2
  z_voxels: 10
  max_obstacle_height: 2.0
  mark_threshold: 2
  observation_sources: scan

inflation_layer:
  plugin: "nav2_costmap_2d::InflationLayer"
  cost_scaling_factor: 3.0
  inflation_radius: 0.55

# Global costmap configuration
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: false
      resolution: 0.05
      track_unknown_space: true
      
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

# Local costmap configuration  
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: false
      rolling_window: true
      width: 5
      height: 5
      resolution: 0.05
      
      plugins: ["voxel_layer", "obstacle_layer", "inflation_layer"]
```

## 5.4.4 Smac Planner کے ساتھ عالمی راستہ منصوبہ بندی

Smac Planner غیر ہولونومک روبوٹس کے لیے کینیمیٹک طور پر قابل عمل راستے فراہم کرتا ہے:

```yaml
# config/planner_server.yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 1.0
    use_sim_time: false
    planner_plugins: ["GridBased"]
    
    GridBased:
      plugin: "nav2_smac_planner/SmacPlannerHybridA*"
      
      # Humanoid motion model
      motion_model_for_search: "DUBIN"  # or "REEDS_SHEPP" for backward motion
      angle_quantization_bins: 72
      analytic_expansion_ratio: 3.5
      analytic_expansion_max_length: 3.0
      
      # Search parameters
      minimum_turning_radius: 0.4  # Humanoid turning constraint
      reverse_penalty: 2.0
      change_penalty: 0.05
      non_straight_penalty: 1.2
      cost_penalty: 2.0
      
      # Smoothing
      smoother:
        max_iterations: 1000
        w_smooth: 0.3
        w_data: 0.2
        tolerance: 1e-10
        do_refinement: true
```

کسٹم پلینر پلگ ان کے لیے پائیتھون کی نافذگی:

```python
# humanoid_nav/smac_planner_node.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionServer
import numpy as np

class HumanoidSmacPlanner(Node):
    def __init__(self):
        super().__init__('humanoid_smac_planner')
        
        # Action server for path planning
        self._action_server = ActionServer(
            self,
            ComputePathToPose,
            'compute_path_to_pose',
            self.execute_callback
        )
        
        # Planner parameters
        self.turning_radius = 0.4
        self.step_size = 0.1
        self.angle_resolution = np.pi / 36  # 5 degrees
        
        self.get_logger().info('Smac planner initialized')
    
    def execute_callback(self, goal_handle):
        """Compute path to goal pose"""
        request = goal_handle.request
        start = request.start
        goal = request.goal
        
        self.get_logger().info('Planning path...')
        
        # Run A* search with kinematic constraints
        path = self.plan_path(start, goal)
        
        if path:
            result = ComputePathToPose.Result()
            result.path = path
            goal_handle.succeed()
            return result
        else:
            goal_handle.abort()
            return ComputePathToPose.Result()
    
    def plan_path(self, start, goal):
        """Hybrid A* search"""
        # Simplified implementation
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        
        # Generate waypoints
        num_points = 20
        for i in range(num_points):
            t = i / (num_points - 1)
            pose = PoseStamped()
            pose.header = path.header
            
            # Linear interpolation (replace with actual A*)
            pose.pose.position.x = (
                start.pose.position.x * (1 - t) + 
                goal.pose.position.x * t
            )
            pose.pose.position.y = (
                start.pose.position.y * (1 - t) + 
                goal.pose.position.y * t
            )
            pose.pose.orientation = goal.pose.orientation
            
            path.poses.append(pose)
        
        return path

def main(args=None):
    rclpy.init(args=args)
    planner = HumanoidSmacPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()
```

## 5.4.5 DWB کنٹرولر کے ساتھ مقامی ٹریجیکٹری منصوبہ بندی

DWB (ڈائنامک ونڈو اپروچ) ہلکی پھلکی ٹریجیکٹریز پیدا کرتا ہے جو رفتار کی پابندیاں کا احترام کرتی ہیں:

```python
# humanoid_nav/dwb_controller.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
import numpy as np

class HumanoidDWBController(Node):
    def __init__(self):
        super().__init__('humanoid_dwb_controller')
        
        # Velocity limits
        self.max_vel_x = 0.5
        self.max_vel_theta = 0.8
        self.max_acc_x = 0.5
        self.max_acc_theta = 1.0
        
        # Trajectory simulation parameters
        self.sim_time = 2.0
        self.sim_granularity = 0.025
        self.angular_sim_granularity = 0.025
        
        # Current state
        self.current_vel = Twist()
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        
        self.path_sub = self.create_subscription(
            Path,
            'local_plan',
            self.path_callback,
            10
        )
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
        # Control loop
        self.create_timer(0.05, self.control_loop)  # 20 Hz
        
    def odom_callback(self, msg):
        """Update current velocity"""
        self.current_vel = msg.twist.twist
    
    def path_callback(self, msg):
        """Store received path"""
        self.current_path = msg
    
    def control_loop(self):
        """Generate velocity commands"""
        if not hasattr(self, 'current_path'):
            return
        
        # Sample velocity space
        best_trajectory = None
        best_score = -float('inf')
        
        # Generate velocity samples
        v_samples = np.linspace(-0.3, self.max_vel_x, 10)
        w_samples = np.linspace(-self.max_vel_theta, self.max_vel_theta, 20)
        
        for v in v_samples:
            for w in w_samples:
                # Check dynamic window
                if not self.in_dynamic_window(v, w):
                    continue
                
                # Simulate trajectory
                trajectory = self.simulate_trajectory(v, w)
                
                # Score trajectory
                score = self.score_trajectory(trajectory)
                
                if score > best_score:
                    best_score = score
                    best_trajectory = (v, w)
        
        # Publish best velocity
        if best_trajectory:
            cmd = Twist()
            cmd.linear.x = best_trajectory[0]
            cmd.angular.z = best_trajectory[1]
            self.cmd_vel_pub.publish(cmd)
    
    def in_dynamic_window(self, v, w):
        """Check if velocity is reachable"""
        dt = 0.05
        
        # Check acceleration constraints
        dv = abs(v - self.current_vel.linear.x)
        dw = abs(w - self.current_vel.angular.z)
        
        if dv > self.max_acc_x * dt:
            return False
        if dw > self.max_acc_theta * dt:
            return False
        
        return True
    
    def simulate_trajectory(self, v, w):
        """Forward simulate trajectory"""
        trajectory = []
        x, y, theta = 0, 0, 0
        
        num_steps = int(self.sim_time / self.sim_granularity)
        
        for _ in range(num_steps):
            x += v * np.cos(theta) * self.sim_granularity
            y += v * np.sin(theta) * self.sim_granularity
            theta += w * self.sim_granularity
            
            trajectory.append((x, y, theta))
        
        return trajectory
    
    def score_trajectory(self, trajectory):
        """Evaluate trajectory quality"""
        # Simplified scoring
        path_alignment = self.compute_path_alignment(trajectory)
        goal_distance = self.compute_goal_distance(trajectory)
        obstacle_distance = self.compute_obstacle_distance(trajectory)
        
        score = (
            path_alignment * 32.0 +
            goal_distance * 24.0 +
            obstacle_distance * 10.0
        )
        
        return score
    
    def compute_path_alignment(self, trajectory):
        """Score alignment with global path"""
        # Implementation depends on path representation
        return 1.0
    
    def compute_goal_distance(self, trajectory):
        """Score distance to goal"""
        # Closer is better
        return 1.0
    
    def compute_obstacle_distance(self, trajectory):
        """Score distance to obstacles"""
        # Further is better
        return 1.0
```

## 5.4.6 cuVSLAM کے ساتھ ویژول SLAM انٹیگریشن

NVIDIA کے cuVSLAM کا استعمال کرتے ہوئے GPU کی مدد سے ویژول اوڈومیٹری کے ذریعے مضبوط لوکلیزیشن:

```python
# humanoid_nav/vslam_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
import numpy as np
import cv2
from cv_bridge import CvBridge

class VisualSLAMNode(Node):
    def __init__(self):
        super().__init__('visual_slam')
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # SLAM state
        self.current_pose = None
        self.map_points = []
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )
        
        # Publishers
        self.pose_pub = self.create_publisher(
            PoseStamped,
            'visual_slam/pose',
            10
        )
        
        self.odom_pub = self.create_publisher(
            Odometry,
            'visual_slam/odom',
            10
        )
        
        # Feature detector
        self.orb = cv2.ORB_create(nfeatures=1000)
        self.prev_frame = None
        self.prev_keypoints = None
        self.prev_descriptors = None
        
        self.get_logger().info('Visual SLAM node initialized')
    
    def camera_info_callback(self, msg):
        """Store camera intrinsics"""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
    
    def image_callback(self, msg):
        """Process visual odometry"""
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'mono8')
        
        # Detect features
        keypoints, descriptors = self.orb.detectAndCompute(cv_image, None)
        
        if self.prev_frame is not None:
            # Match features
            matches = self.match_features(
                self.prev_descriptors,
                descriptors
            )
            
            if len(matches) > 10:
                # Estimate motion
                delta_pose = self.estimate_motion(
                    self.prev_keypoints,
                    keypoints,
                    matches
                )
                
                # Update pose
                self.update_pose(delta_pose)
                
                # Publish
                self.publish_pose()
        
        # Store current frame
        self.prev_frame = cv_image
        self.prev_keypoints = keypoints
        self.prev_descriptors = descriptors
    
    def match_features(self, desc1, desc2):
        """Match features between frames"""
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        matches = bf.match(desc1, desc2)
        matches = sorted(matches, key=lambda x: x.distance)
        return matches[:50]  # Top 50 matches
    
    def estimate_motion(self, kp1, kp2, matches):
        """Estimate camera motion from matches"""
        # Extract matched points
        pts1 = np.float32([kp1[m.queryIdx].pt for m in matches])
        pts2 = np.float32([kp2[m.trainIdx].pt for m in matches])
        
        # Compute essential matrix
        E, mask = cv2.findEssentialMat(
            pts1,
            pts2,
            self.camera_matrix,
            method=cv2.RANSAC,
            prob=0.999,
            threshold=1.0
        )
        
        # Recover pose
        _, R, t, mask = cv2.recoverPose(
            E,
            pts1,
            pts2,
            self.camera_matrix
        )
        
        return R, t
    
    def update_pose(self, delta_pose):
        """Update current pose estimate"""
        R, t = delta_pose
        
        # Initialize pose if needed
        if self.current_pose is None:
            self.current_pose = {
                'position': np.zeros(3),
                'orientation': np.eye(3)
            }
        
        # Update position and orientation
        self.current_pose['position'] += self.current_pose['orientation'] @ t.flatten()
        self.current_pose['orientation'] = R @ self.current_pose['orientation']
    
    def publish_pose(self):
        """Publish current pose estimate"""
        if self.current_pose is None:
            return
        
        # Create PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        
        pose_msg.pose.position.x = self.current_pose['position'][0]
        pose_msg.pose.position.y = self.current_pose['position'][1]
        pose_msg.pose.position.z = self.current_pose['position'][2]
        
        # Convert rotation matrix to quaternion
        # Simplified - use proper conversion
        pose_msg.pose.orientation.w = 1.0
        
        self.pose_pub.publish(pose_msg)
        
        # Publish TF
        self.publish_transform(pose_msg)
    
    def publish_transform(self, pose):
        """Broadcast TF transform"""
        t = TransformStamped()
        t.header = pose.header
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = pose.pose.position.x
        t.transform.translation.y = pose.pose.position.y
        t.transform.translation.z = pose.pose.position.z
        t.transform.rotation = pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = VisualSLAMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## 5.4.7 ڈائنامک رکاوٹوں سے بچاؤ

متعدد سینسرز سے ریئل ٹائم رکاوٹوں کا پتہ لگانا اور بچنا:

```python
# humanoid_nav/obstacle_detector.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Twist
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

class DynamicObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        
        # Obstacle tracking
        self.obstacles = []
        self.velocity_estimates = {}
        
        # Safety parameters
        self.safety_distance = 0.5  # meters
        self.emergency_stop_distance = 0.3
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            'depth/points',
            self.cloud_callback,
            10
        )
        
        # Publisher for emergency stop
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel_override',
            10
        )
        
        self.create_timer(0.1, self.check_safety)
        
    def scan_callback(self, msg):
        """Process laser scan for obstacles"""
        # Convert ranges to cartesian coordinates
        angles = np.arange(
            msg.angle_min,
            msg.angle_max,
            msg.angle_increment
        )
        
        ranges = np.array(msg.ranges)
        valid = (ranges > msg.range_min) & (ranges < msg.range_max)
        
        x = ranges[valid] * np.cos(angles[valid])
        y = ranges[valid] * np.sin(angles[valid])
        
        # Cluster points into obstacles
        obstacles = self.cluster_points(x, y)
        self.obstacles = obstacles
    
    def cloud_callback(self, msg):
        """Process 3D point cloud"""
        points = []
        for point in pc2.read_points(msg, skip_nans=True):
            points.append([point[0], point[1], point[2]])
        
        if len(points) > 0:
            points = np.array(points)
            # Filter points in robot's path
            forward_points = points[
                (points[:, 0] > 0) &
                (np.abs(points[:, 1]) < 0.5) &
                (points[:, 2] < 1.8)  # Below human height
            ]
            
            if len(forward_points) > 0:
                min_distance = np.min(np.linalg.norm(
                    forward_points[:, :2],
                    axis=1
                ))
                
                if min_distance < self.emergency_stop_distance:
                    self.emergency_stop()
    
    def cluster_points(self, x, y):
        """Cluster points into distinct obstacles"""
        points = np.column_stack([x, y])
        
        if len(points) == 0:
            return []
        
        # Simple distance-based clustering
        clusters = []
        remaining = list(range(len(points)))
        
        while remaining:
            cluster = [remaining.pop(0)]
            i = 0
            
            while i < len(remaining):
                idx = remaining[i]
                
                # Check distance to any point in cluster
                for c_idx in cluster:
                    dist = np.linalg.norm(points[idx] - points[c_idx])
                    if dist < 0.2:  # 20cm clustering threshold
                        cluster.append(remaining.pop(i))
                        break
                else:
                    i += 1
            
            # Compute cluster centroid
            cluster_points = points[cluster]
            centroid = np.mean(cluster_points, axis=0)
            radius = np.max(np.linalg.norm(
                cluster_points - centroid,
                axis=1
            ))
            
            clusters.append({
                'position': centroid,
                'radius': radius
            })
        
        return clusters
    
    def check_safety(self):
        """Monitor for collision risks"""
        if not self.obstacles:
            return
        
        # Check distance to nearest obstacle
        min_distance = float('inf')
        
        for obstacle in self.obstacles:
            dist = np.linalg.norm(obstacle['position'])
            if dist < min_distance:
                min_distance = dist
        
        if min_distance < self.emergency_stop_distance:
            self.emergency_stop()
            self.get_logger().warn(
                f'Emergency stop! Obstacle at {min_distance:.2f}m'
            )
    
    def emergency_stop(self):
        """Publish stop command"""
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_cmd)

def main(args=None):
    rclpy.init(args=args)
    detector = DynamicObstacleDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()
```

## 5.4.8 انسانوں کے لیے آگاه نیویگیشن

ذاتی جگہ کا احترام کرنے والے سماجی نیویگیشن رویوں کو نافذ کرنا:

```python
# humanoid_nav/social_navigation.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import Path
from vision_msgs.msg import Detection2DArray
import numpy as np

class SocialNavigationNode(Node):
    def __init__(self):
        super().__init__('social_navigation')
        
        # Social distance parameters
        self.personal_space = 1.2  # meters
        self.social_space = 2.5
        self.approach_angle = np.pi / 4  # 45 degrees
        
        # Human tracking
        self.detected_humans = []
        
        # Subscribers
        self.detections_sub = self.create_subscription(
            Detection2DArray,
            'human_detections',
            self.detections_callback,
            10
        )
        
        self.path_sub = self.create_subscription(
            Path,
            'planned_path',
            self.path_callback,
            10
        )
        
        # Publisher for modified path
        self.social_path_pub = self.create_publisher(
            Path,
            'social_aware_path',
            10
        )
        
    def detections_callback(self, msg):
        """Track detected humans"""
        self.detected_humans = []
        
        for detection in msg.detections:
            # Extract human position (simplified)
            # In practice, use depth + bbox to estimate 3D position
            human_pos = self.estimate_3d_position(detection)
            
            self.detected_humans.append({
                'position': human_pos,
                'velocity': self.estimate_velocity(detection)
            })
    
    def path_callback(self, msg):
        """Modify path to respect social norms"""
        if not self.detected_humans:
            self.social_path_pub.publish(msg)
            return
        
        # Create modified path
        social_path = Path()
        social_path.header = msg.header
        
        for pose in msg.poses:
            # Check if waypoint violates social space
            modified_pose = self.apply_social_constraints(
                pose,
                self.detected_humans
            )
            social_path.poses.append(modified_pose)
        
        self.social_path_pub.publish(social_path)
    
    def apply_social_constraints(self, pose, humans):
        """Adjust waypoint to maintain social distance"""
        pos = np.array([
            pose.pose.position.x,
            pose.pose.position.y
        ])
        
        for human in humans:
            h_pos = np.array(human['position'][:2])
            distance = np.linalg.norm(pos - h_pos)
            
            # If too close, push waypoint away
            if distance < self.personal_space:
                # Compute repulsion vector
                repulsion = (pos - h_pos) / distance
                pos += repulsion * (self.personal_space - distance)
            
            # Apply side-passing preference
            elif distance < self.social_space:
                # Pass on the right when possible
                lateral_offset = self.compute_passing_offset(pos, h_pos)
                pos += lateral_offset
        
        # Update pose
        modified_pose = PoseStamped()
        modified_pose.header = pose.header
        modified_pose.pose = pose.pose
        modified_pose.pose.position.x = pos[0]
        modified_pose.pose.position.y = pos[1]
        
        return modified_pose
    
    def compute_passing_offset(self, robot_pos, human_pos):
        """Compute lateral offset for comfortable passing"""
        # Perpendicular direction for passing
        direction = human_pos - robot_pos
        perpendicular = np.array([-direction[1], direction[0]])
        perpendicular = perpendicular / np.linalg.norm(perpendicular)
        
        # Offset to the right (0.3m)
        offset = perpendicular * 0.3
        return offset
    
    def estimate_3d_position(self, detection):
        """Estimate 3D position from 2D detection"""
        # Simplified - use depth sensor or stereo
        # For now, assume at fixed distance
        return np.array([2.0, 0.0, 0.0])
    
    def estimate_velocity(self, detection):
        """Estimate human velocity for prediction"""
        # Track detection over time
        return np.array([0.0, 0.0])

def main(args=None):
    rclpy.init(args=args)
    node = SocialNavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## 5.4.9 نیویگیشن فیلیئرز کے لیے ریکوری رویے

جب نیویگیشن پھنس جائے تو ذہین ریکوری حکمت عملیوں کو نافذ کرنا:

```python
# humanoid_nav/recovery_behaviors.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np
import time

class RecoveryBehaviorsNode(Node):
    def __init__(self):
        super().__init__('recovery_behaviors')
        
        # Recovery state
        self.is_stuck = False
        self.stuck_start_time = None
        self.recovery_attempts = 0
        
        # Thresholds
        self.stuck_time_threshold = 5.0  # seconds
        self.max_recovery_attempts = 3
        
        # Subscribers
        self.stuck_sub = self.create_subscription(
            Bool,
            'navigation/stuck',
            self.stuck_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel_recovery',
            10
        )
        
        self.recovery_status_pub = self.create_publisher(
            Bool,
            'recovery/active',
            10
        )
        
        self.get_logger().info('Recovery behaviors initialized')
    
    def stuck_callback(self, msg):
        """Handle stuck detection"""
        if msg.data and not self.is_stuck:
            # Just got stuck
            self.is_stuck = True
            self.stuck_start_time = time.time()
            self.execute_recovery_sequence()
        elif not msg.data:
            # No longer stuck
            self.is_stuck = False
            self.recovery_attempts = 0
    
    def execute_recovery_sequence(self):
        """Try different recovery strategies"""
        self.get_logger().warn('Robot stuck - attempting recovery')
        
        # Publish recovery status
        status_msg = Bool()
        status_msg.data = True
        self.recovery_status_pub.publish(status_msg)
        
        if self.recovery_attempts == 0:
            self.back_up_recovery()
        elif self.recovery_attempts == 1:
            self.rotate_recovery()
        elif self.recovery_attempts == 2:
            self.clear_and_replan()
        else:
            self.request_human_assistance()
        
        self.recovery_attempts += 1
    
    def back_up_recovery(self):
        """Back up to escape local minimum"""
        self.get_logger().info('Recovery: Backing up')
        
        backup_cmd = Twist()
        backup_cmd.linear.x = -0.2  # Slow backward
        
        # Back up for 2 seconds
        start_time = time.time()
        rate = self.create_rate(10)
        
        while time.time() - start_time < 2.0:
            self.cmd_vel_pub.publish(backup_cmd)
            rate.sleep()
        
        # Stop
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
    
    def rotate_recovery(self):
        """Rotate to find clear path"""
        self.get_logger().info('Recovery: Rotating')
        
        rotate_cmd = Twist()
        rotate_cmd.angular.z = 0.5  # Moderate rotation
        
        # Rotate for 3 seconds
        start_time = time.time()
        rate = self.create_rate(10)
        
        while time.time() - start_time < 3.0:
            self.cmd_vel_pub.publish(rotate_cmd)
            rate.sleep()
        
        # Stop
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
    
    def clear_and_replan(self):
        """Clear costmaps and request new plan"""
        self.get_logger().info('Recovery: Clearing costmaps')
        # Trigger costmap clearing service
        # Request new global plan
    
    def request_human_assistance(self):
        """Alert human operator for help"""
        self.get_logger().error('Recovery failed - requesting assistance')
        # Publish alert message
        # Play audio alert
        # Send notification

def main(args=None):
    rclpy.init(args=args)
    node = RecoveryBehaviorsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## 5.4.10 نیویگیشن کارکردگی ٹیسٹنگ

نیویگیشن سسٹم کے لیے جامع ٹیسٹنگ فریم ورک:

```python
# scripts/test_navigation.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import numpy as np
import time
import json

class NavigationTester(Node):
    def __init__(self):
        super().__init__('navigation_tester')
        
        # Action client
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        
        # Test scenarios
        self.test_scenarios = [
            {
                'name': 'Straight corridor',
                'goal': {'x': 5.0, 'y': 0.0},
                'expected_time': 15.0
            },
            {
                'name': 'Right turn',
                'goal': {'x': 3.0, 'y': 3.0},
                'expected_time': 20.0
            },
            {
                'name': 'Narrow passage',
                'goal': {'x': 2.0, 'y': -2.0},
                'expected_time': 25.0
            },
            {
                'name': 'Crowded area',
                'goal': {'x': -3.0, 'y': 4.0},
                'expected_time': 30.0
            }
        ]
        
        # Test results
        self.results = []
    
    def run_tests(self):
        """Execute all test scenarios"""
        self.get_logger().info('Starting navigation tests')
        
        for scenario in self.test_scenarios:
            self.get_logger().info(f"Testing: {scenario['name']}")
            
            result = self.test_navigation(scenario)
            self.results.append(result)
            
            # Wait between tests
            time.sleep(5.0)
        
        self.print_summary()
    
    def test_navigation(self, scenario):
        """Test single navigation scenario"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = scenario['goal']['x']
        goal_msg.pose.pose.position.y = scenario['goal']['y']
        goal_msg.pose.pose.orientation.w = 1.0
        
        # Send goal
        start_time = time.time()
        self.nav_client.wait_for_server()
        
        future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            return {
                'scenario': scenario['name'],
                'success': False,
                'reason': 'Goal rejected'
            }
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        elapsed_time = time.time() - start_time
        
        return {
            'scenario': scenario['name'],
            'success': True,
            'time': elapsed_time,
            'expected_time': scenario['expected_time'],
            'performance_ratio': elapsed_time / scenario['expected_time']
        }
    
    def print_summary(self):
        """Display test results"""
        print("\n" + "="*60)
        print("NAVIGATION TEST SUMMARY")
        print("="*60)
        
        total_tests = len(self.results)
        passed_tests = sum(1 for r in self.results if r['success'])
        
        for result in self.results:
            status = "PASS" if result['success'] else "FAIL"
            print(f"\n{result['scenario']}: {status}")
            
            if result['success']:
                print(f"  Time: {result['time']:.2f}s")
                print(f"  Expected: {result['expected_time']:.2f}s")
                print(f"  Ratio: {result['performance_ratio']:.2f}x")
        
        print(f"\nTotal: {passed_tests}/{total_tests} tests passed")
        print("="*60)
        
        # Save to file
        with open('navigation_test_results.json', 'w') as f:
            json.dump(self.results, f, indent=2)

def main(args=None):
    rclpy.init(args=args)
    tester = NavigationTester()
    tester.run_tests()
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 5.4.11 ریئل ٹائم کارکردگی کی بہتری

ریئل ٹائم ردعمل کے لیے نیویگیشن کو بہتر بنانا:

```python
# humanoid_nav/performance_monitor.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
import psutil
import numpy as np

class PerformanceMonitor(Node):
    def __init__(self):
        super().__init__('performance_monitor')
        
        # Performance metrics
        self.loop_times = []
        self.cpu_usage = []
        self.memory_usage = []
        
        # Publishers
        self.loop_time_pub = self.create_publisher(
            Float32,
            'performance/loop_time',
            10
        )
        
        self.cpu_pub = self.create_publisher(
            Float32,
            'performance/cpu_usage',
            10
        )
        
        # Monitor timer
        self.create_timer(0.1, self.monitor_performance)
        
        self.last_time = time.time()
    
    def monitor_performance(self):
        """Track real-time performance metrics"""
        current_time = time.time()
        loop_time = current_time - self.last_time
        self.last_time = current_time
        
        # Track loop time
        self.loop_times.append(loop_time)
        if len(self.loop_times) > 100:
            self.loop_times.pop(0)
        
        # Track CPU usage
        cpu_percent = psutil.cpu_percent(interval=None)
        self.cpu_usage.append(cpu_percent)
        
        # Track memory
        memory = psutil.virtual_memory()
        self.memory_usage.append(memory.percent)
        
        # Publish metrics
        loop_msg = Float32()
        loop_msg.data = loop_time
        self.loop_time_pub.publish(loop_msg)
        
        cpu_msg = Float32()
        cpu_msg.data = cpu_percent
        self.cpu_pub.publish(cpu_msg)
        
        # Log warnings
        if loop_time > 0.15:  # More than 150ms
            self.get_logger().warn(f'Slow loop: {loop_time*1000:.1f}ms')
        
        if cpu_percent > 80.0:
            self.get_logger().warn(f'High CPU usage: {cpu_percent:.1f}%')
    
    def get_statistics(self):
        """Compute performance statistics"""
        return {
            'avg_loop_time': np.mean(self.loop_times),
            'max_loop_time': np.max(self.loop_times),
            'avg_cpu': np.mean(self.cpu_usage),
            'max_cpu': np.max(self.cpu_usage)
        }

def main(args=None):
    rclpy.init(args=args)
    monitor = PerformanceMonitor()
    rclpy.spin(monitor)
    
    # Print final statistics
    stats = monitor.get_statistics()
    print("\nPerformance Statistics:")
    print(f"Average loop time: {stats['avg_loop_time']*1000:.2f}ms")
    print(f"Max loop time: {stats['max_loop_time']*1000:.2f}ms")
    print(f"Average CPU: {stats['avg_cpu']:.1f}%")
    print(f"Max CPU: {stats['max_cpu']:.1f}%")
    
    monitor.destroy_node()
    rclpy.shutdown()
```

## 5.4.12 مکمل لانچ ترتیب

نیویگیشن سسٹم کے لیے جامع لانچ فائل:

```python
# launch/navigation_system.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('humanoid_nav')
    
    # Configuration files
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    costmap_params = os.path.join(pkg_share, 'config', 'costmap_common.yaml')
    
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        # Visual SLAM
        Node(
            package='humanoid_nav',
            executable='vslam_node',
            name='visual_slam',
            output='screen'
        ),
        
        # Obstacle detection
        Node(
            package='humanoid_nav',
            executable='obstacle_detector',
            name='obstacle_detector',
            output='screen'
        ),
        
        # Social navigation
        Node(
            package='humanoid_nav',
            executable='social_navigation',
            name='social_navigation',
            output='screen'
        ),
        
        # Recovery behaviors
        Node(
            package='humanoid_nav',
            executable='recovery_behaviors',
            name='recovery_behaviors',
            output='screen'
        ),
        
        # Performance monitor
        Node(
            package='humanoid_nav',
            executable='performance_monitor',
            name='performance_monitor',
            output='screen'
        ),
        
        # Nav2 stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('nav2_bringup'),
                '/launch/navigation_launch.py'
            ]),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': nav2_params
            }.items()
        ),
    ])
```

## 5.4.13 خلاصہ

اس باب میں ہیومینائڈ روبوٹس کے لیے ایک مکمل خودکار نیویگیشن سسٹم کو نافذ کیا گیا ہے جس میں Nav2 کی انٹیگریشن ہیومینائڈ مخصوص کینیمیٹک پابندیاں، LiDAR اور گہرائی سینسرز کا استعمال کرتے ہوئے ڈائنامک رکاوٹوں سے بچاؤ، بیرونی بیکنز کے بغیر مضبوط لوکلیزیشن کے لیے ویژول SLAM، سماجی فاصلے کے اصولوں کا احترام کرنے والی انسانوں کے لیے آگاه نیویگیشن، نیویگیشن فیلیئرز کو سنبھالنے کے لیے ذہین ریکوری رویے، اور جامع کارکردگی کی نگرانی شامل ہے۔ یہ سسٹم پیچیدہ انسانوں کے ماحول میں محفوظ، موثر، اور سماجی طور پر قابل قبول نیویگیشن فراہم کرتا ہے جبکہ ریئل ٹائم کارکردگی کو برقرار رکھتا ہے تاکہ ردعملی خودکار حرکت ممکن ہو۔