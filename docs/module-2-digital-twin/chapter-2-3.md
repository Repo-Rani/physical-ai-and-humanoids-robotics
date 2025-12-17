---
id: module-2-chapter-3
title: "Sensor Simulation in Gazebo"
sidebar_label: "2.3 Sensor Simulation"
sidebar_position: 3
description: "Simulate cameras, LiDAR, IMU, and depth sensors in Gazebo with realistic noise models, calibration, and ROS 2 integration for perception development"
keywords: [sensors, camera-simulation, lidar, imu, depth-camera, noise-models, gazebo-plugins]
estimated_time: 75
prerequisites:
  - module-2-chapter-2
  - basic-sensor-concepts
learning_outcomes:
  - Configure camera plugins for RGB and depth sensing
  - Simulate LiDAR sensors with realistic point clouds
  - Add IMU sensors with noise and bias models
  - Integrate simulated sensors with ROS 2 perception pipelines
  - Generate synthetic datasets for machine learning
hardware_tier: miniature
---

# Chapter 2.3: Sensor Simulation in Gazebo

Accurate sensor simulation is critical for developing robust perception algorithms. This chapter covers how to simulate cameras, LiDAR, IMUs, and other sensors in Gazebo with realistic noise characteristics.

## Why Sensor Simulation Matters

Accurate sensor simulation enables:

- **Perception Algorithm Development**: Test vision and point cloud processing
- **Synthetic Data Generation**: Create labeled datasets for machine learning
- **Sensor Placement Optimization**: Find optimal mounting locations
- **Edge Case Testing**: Simulate rare scenarios (fog, glare, darkness)
- **Hardware Selection**: Evaluate sensors before purchasing

## Camera Simulation

### RGB Camera Plugin

Configure a basic camera in Gazebo SDF:

```xml
<sensor name="camera" type="camera">
  <pose>0.1 0 0.2 0 0 0</pose>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.007</stddev>
    </noise>
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
  <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
    <ros>
      <namespace>/robot</namespace>
      <remapping>image_raw:=camera/image_raw</remapping>
      <remapping>camera_info:=camera/camera_info</remapping>
    </ros>
    <camera_name>front_camera</camera_name>
    <frame_name>camera_link</frame_name>
  </plugin>
</sensor>
```

### Depth Camera (RGB-D)

Simulate depth sensors like Intel RealSense:

```xml
<sensor name="depth_camera" type="depth_camera">
  <pose>0.1 0 0.2 0 0 0</pose>
  <camera>
    <horizontal_fov>1.5708</horizontal_fov>  <!-- 90 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.3</near>
      <far>10.0</far>
    </clip>
    <depth_camera>
      <clip>
        <near>0.3</near>
        <far>10.0</far>
      </clip>
    </depth_camera>
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <plugin name="depth_camera_controller" filename="libgazebo_ros_camera.so">
    <ros>
      <namespace>/robot</namespace>
      <remapping>depth/image_raw:=depth_camera/depth/image_raw</remapping>
      <remapping>depth/camera_info:=depth_camera/depth/camera_info</remapping>
      <remapping>points:=depth_camera/points</remapping>
    </ros>
    <camera_name>depth_camera</camera_name>
    <frame_name>depth_camera_link</frame_name>
  </plugin>
</sensor>
```

### Camera Intrinsic Parameters

Camera intrinsics define the projection model:

```python
import numpy as np

class CameraIntrinsics:
    def __init__(self, width, height, hfov):
        self.width = width
        self.height = height
        self.hfov = hfov  # horizontal field of view (radians)

        # Calculate focal length in pixels
        self.fx = (width / 2.0) / np.tan(hfov / 2.0)
        self.fy = self.fx  # assume square pixels

        # Principal point (center of image)
        self.cx = width / 2.0
        self.cy = height / 2.0

        # Camera matrix K
        self.K = np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ])

    def project_point(self, point_3d):
        """Project 3D point to 2D image coordinates"""
        # point_3d: [x, y, z] in camera frame
        if point_3d[2] <= 0:
            return None  # behind camera

        u = self.fx * point_3d[0] / point_3d[2] + self.cx
        v = self.fy * point_3d[1] / point_3d[2] + self.cy

        return (u, v)

# Example: 640x480 camera with 60° FOV
camera = CameraIntrinsics(640, 480, np.deg2rad(60))
print(f"Focal length: {camera.fx:.1f} pixels")
print(f"Camera matrix:\n{camera.K}")
```

### Camera Noise Models

Add realistic noise to simulated images:

```python
import cv2
import numpy as np

class CameraNoiseSimulator:
    def add_gaussian_noise(self, image, mean=0, stddev=10):
        """Add Gaussian noise (sensor readout noise)"""
        noise = np.random.normal(mean, stddev, image.shape).astype(np.uint8)
        noisy = cv2.add(image, noise)
        return noisy

    def add_salt_pepper(self, image, prob=0.01):
        """Add salt-and-pepper noise (dead pixels)"""
        noisy = image.copy()
        # Salt (white pixels)
        salt_mask = np.random.random(image.shape[:2]) < (prob / 2)
        noisy[salt_mask] = 255
        # Pepper (black pixels)
        pepper_mask = np.random.random(image.shape[:2]) < (prob / 2)
        noisy[pepper_mask] = 0
        return noisy

    def add_motion_blur(self, image, kernel_size=5):
        """Add motion blur"""
        kernel = np.zeros((kernel_size, kernel_size))
        kernel[kernel_size//2, :] = np.ones(kernel_size)
        kernel /= kernel_size
        blurred = cv2.filter2D(image, -1, kernel)
        return blurred

    def simulate_realistic(self, image):
        """Apply combination of noise effects"""
        # Gaussian noise (readout)
        noisy = self.add_gaussian_noise(image, stddev=5)
        # Salt-and-pepper (dead pixels)
        noisy = self.add_salt_pepper(noisy, prob=0.001)
        # Slight motion blur
        noisy = self.add_motion_blur(noisy, kernel_size=3)
        return noisy
```

## LiDAR Simulation

### 2D LiDAR (Laser Scanner)

Simulate a planar laser scanner:

```xml
<sensor name="lidar" type="gpu_lidar">
  <pose>0 0 0.15 0 0 0</pose>
  <lidar>
    <scan>
      <horizontal>
        <samples>720</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.12</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>
    </noise>
  </lidar>
  <always_on>1</always_on>
  <update_rate>10</update_rate>
  <visualize>true</visualize>
  <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
    <ros>
      <namespace>/robot</namespace>
      <remapping>~/out:=scan</remapping>
    </ros>
    <output_type>sensor_msgs/LaserScan</output_type>
    <frame_name>lidar_link</frame_name>
  </plugin>
</sensor>
```

### 3D LiDAR

Simulate spinning 3D LiDAR (e.g., Velodyne):

```xml
<sensor name="velodyne" type="gpu_lidar">
  <pose>0 0 0.4 0 0 0</pose>
  <lidar>
    <scan>
      <horizontal>
        <samples>1024</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>16</samples>
        <resolution>1</resolution>
        <min_angle>-0.261799</min_angle>  <!-- -15 degrees -->
        <max_angle>0.261799</max_angle>   <!-- +15 degrees -->
      </vertical>
    </scan>
    <range>
      <min>0.9</min>
      <max>100.0</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.02</stddev>
    </noise>
  </lidar>
  <always_on>1</always_on>
  <update_rate>10</update_rate>
  <plugin name="velodyne_controller" filename="libgazebo_ros_ray_sensor.so">
    <ros>
      <namespace>/robot</namespace>
      <remapping>~/out:=velodyne_points</remapping>
    </ros>
    <output_type>sensor_msgs/PointCloud2</output_type>
    <frame_name>velodyne_link</frame_name>
  </plugin>
</sensor>
```

### LiDAR Ray Casting

Understand how LiDAR simulation works:

```python
import numpy as np

class LiDARSimulator:
    def __init__(self, min_range=0.1, max_range=30, num_beams=720):
        self.min_range = min_range
        self.max_range = max_range
        self.num_beams = num_beams
        self.angles = np.linspace(-np.pi, np.pi, num_beams)

    def cast_ray(self, origin, direction, obstacles):
        """Ray casting for single beam"""
        min_distance = self.max_range

        for obstacle in obstacles:
            # Simple ray-plane intersection
            distance = self.ray_plane_intersection(origin, direction, obstacle)
            if distance and distance < min_distance:
                min_distance = distance

        return min_distance if min_distance < self.max_range else np.inf

    def simulate_scan(self, robot_pose, obstacles):
        """Generate full 2D scan"""
        ranges = []

        for angle in self.angles:
            # Compute ray direction in world frame
            world_angle = robot_pose[2] + angle  # robot yaw + beam angle
            direction = np.array([np.cos(world_angle), np.sin(world_angle)])

            # Cast ray
            distance = self.cast_ray(robot_pose[:2], direction, obstacles)

            # Add noise
            if distance < self.max_range:
                distance += np.random.normal(0, 0.01)

            ranges.append(max(self.min_range, distance))

        return np.array(ranges)

    def ray_plane_intersection(self, origin, direction, plane):
        """Calculate distance to plane intersection"""
        # Simplified implementation
        pass
```

## IMU Simulation

### IMU Sensor Plugin

Configure Inertial Measurement Unit:

```xml
<sensor name="imu" type="imu">
  <pose>0 0 0.1 0 0 0</pose>
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.009</stddev>
          <bias_mean>0.00075</bias_mean>
          <bias_stddev>0.0000008</bias_stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.009</stddev>
          <bias_mean>0.00075</bias_mean>
          <bias_stddev>0.0000008</bias_stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.009</stddev>
          <bias_mean>0.00075</bias_mean>
          <bias_stddev>0.0000008</bias_stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
          <bias_mean>0.1</bias_mean>
          <bias_stddev>0.001</bias_stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
          <bias_mean>0.1</bias_mean>
          <bias_stddev>0.001</bias_stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
          <bias_mean>0.1</bias_mean>
          <bias_stddev>0.001</bias_stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
  <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
    <ros>
      <namespace>/robot</namespace>
      <remapping>~/out:=imu/data</remapping>
    </ros>
    <frame_name>imu_link</frame_name>
  </plugin>
</sensor>
```

### IMU Data Processing

Process simulated IMU data:

```python
import numpy as np
from scipy.spatial.transform import Rotation

class IMUProcessor:
    def __init__(self, dt=0.01):
        self.dt = dt  # sampling period
        self.velocity = np.zeros(3)
        self.position = np.zeros(3)
        self.orientation = Rotation.identity()

    def integrate_gyroscope(self, angular_velocity):
        """Update orientation from gyroscope"""
        # Angular velocity to rotation
        angle = np.linalg.norm(angular_velocity) * self.dt
        if angle > 1e-6:
            axis = angular_velocity / np.linalg.norm(angular_velocity)
            delta_rotation = Rotation.from_rotvec(axis * angle)
            self.orientation = self.orientation * delta_rotation

        return self.orientation.as_quat()

    def integrate_accelerometer(self, linear_acceleration, gravity=[0, 0, -9.81]):
        """Update velocity and position from accelerometer"""
        # Remove gravity (if orientation known)
        gravity_world = np.array(gravity)
        gravity_body = self.orientation.inv().apply(gravity_world)
        accel_body = linear_acceleration - gravity_body

        # Integrate acceleration
        accel_world = self.orientation.apply(accel_body)
        self.velocity += accel_world * self.dt
        self.position += self.velocity * self.dt

        return self.position, self.velocity
```

## GPS Simulation

### GPS Sensor Plugin

```xml
<sensor name="gps" type="gps">
  <pose>0 0 0.3 0 0 0</pose>
  <always_on>true</always_on>
  <update_rate>10</update_rate>
  <gps>
    <position_sensing>
      <horizontal>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2.0</stddev>  <!-- 2m standard deviation -->
        </noise>
      </horizontal>
      <vertical>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>4.0</stddev>  <!-- 4m standard deviation -->
        </noise>
      </vertical>
    </position_sensing>
    <velocity_sensing>
      <horizontal>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.2</stddev>
        </noise>
      </horizontal>
      <vertical>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.4</stddev>
        </noise>
      </vertical>
    </velocity_sensing>
  </gps>
  <plugin name="gps_plugin" filename="libgazebo_ros_gps_sensor.so">
    <ros>
      <namespace>/robot</namespace>
      <remapping>~/out:=gps/fix</remapping>
    </ros>
    <frame_name>gps_link</frame_name>
  </plugin>
</sensor>
```

## Multi-Sensor Fusion

Combine multiple sensors for robust perception:

```python
import numpy as np

class SensorFusion:
    def __init__(self):
        # Extended Kalman Filter state: [x, y, z, vx, vy, vz]
        self.state = np.zeros(6)
        self.P = np.eye(6) * 10  # covariance

    def predict(self, imu_accel, dt):
        """Prediction step using IMU"""
        # State transition
        F = np.array([
            [1, 0, 0, dt, 0, 0],
            [0, 1, 0, 0, dt, 0],
            [0, 0, 1, 0, 0, dt],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])

        # Control input (acceleration)
        B = np.array([
            [0.5*dt**2, 0, 0],
            [0, 0.5*dt**2, 0],
            [0, 0, 0.5*dt**2],
            [dt, 0, 0],
            [0, dt, 0],
            [0, 0, dt]
        ])

        self.state = F @ self.state + B @ imu_accel
        self.P = F @ self.P @ F.T + self.Q  # process noise

    def update_gps(self, gps_position):
        """Update step using GPS"""
        # Measurement matrix (observe position only)
        H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0]
        ])

        # Innovation
        y = gps_position - H @ self.state

        # Kalman gain
        S = H @ self.P @ H.T + self.R_gps
        K = self.P @ H.T @ np.linalg.inv(S)

        # Update state and covariance
        self.state = self.state + K @ y
        self.P = (np.eye(6) - K @ H) @ self.P

        return self.state
```

## Practical Exercise: Multi-Sensor Robot

Create a robot with multiple sensors:

1. **Add Sensors**: Configure camera, LiDAR, IMU, GPS in URDF/SDF
2. **Launch Gazebo**: Spawn robot in simulation world
3. **Visualize Data**: Use RViz to display sensor outputs
4. **Test Noise**: Drive robot and observe realistic sensor imperfections
5. **Implement Fusion**: Combine IMU and GPS for position estimation

**Validation**:
- [ ] Camera publishes images at 30Hz
- [ ] LiDAR produces point cloud with realistic range noise
- [ ] IMU data shows drift when integrating
- [ ] GPS position has ~2m accuracy
- [ ] Fused estimate outperforms individual sensors

## Synthetic Data Generation

Use simulated sensors for ML training:

```python
import rospy
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import cv2

class DatasetGenerator:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.save_counter = 0

    def image_callback(self, msg):
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Save with auto-incrementing filename
        filename = f"dataset/image_{self.save_counter:06d}.png"
        cv2.imwrite(filename, cv_image)
        self.save_counter += 1

        # Also save ground truth labels, depth, etc.
```

## Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| No sensor data published | Plugin not loaded | Check `ros2 topic list`, verify plugin in SDF |
| Camera shows black image | Lighting missing | Add light sources to world file |
| LiDAR returns all max_range | No obstacles detected | Check sensor pose, add objects |
| IMU orientation drifts | Realistic gyroscope bias | Expected behavior, use sensor fusion |
| Slow simulation | Too many sensors | Reduce update rates, use GPU plugins |

## Next Chapter

In [Chapter 2.4: Multi-Robot Simulation (if exists)](./chapter-2-4.md) or [Chapter 2.5: Unity Integration](./chapter-2-5.md), you'll learn to create interactive visualization interfaces.
