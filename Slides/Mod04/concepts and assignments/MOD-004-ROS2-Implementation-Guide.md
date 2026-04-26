# MOD-004 Final Integration Challenge
# StreetBot — ROS 2 Implementation Guide

**Course:** Mobile Robots — Sensor Modeling, Measurement & Fusion  
**ROS 2 Distribution:** Humble Hawksbill (LTS)  
**Simulation:** Gazebo Fortress  
**Author:** Dr. Ahmad Abbadi  

---

## Table of Contents

1. [Overview & Learning Objectives](#1-overview--learning-objectives)
2. [Prerequisites & Environment Setup](#2-prerequisites--environment-setup)
3. [Workspace Structure](#3-workspace-structure)
4. [ROS 2 Core Concepts You Need](#4-ros-2-core-concepts-you-need)
5. [TF2 Frame Tree — The Backbone](#5-tf2-frame-tree--the-backbone)
6. [Sensor Nodes & Topics — Full Map](#6-sensor-nodes--topics--full-map)
7. [Phase A: Robot Description (URDF/Xacro)](#7-phase-a-robot-description-urdfxacro)
8. [Phase B: Sensor Simulation in Gazebo](#8-phase-b-sensor-simulation-in-gazebo)
9. [Phase C: Sensor Model Nodes (Student Code)](#9-phase-c-sensor-model-nodes-student-code)
10. [Phase D: Sensor Fusion with robot_localization](#10-phase-d-sensor-fusion-with-robot_localization)
11. [Phase E: Sensor Health Monitoring Node](#11-phase-e-sensor-health-monitoring-node)
12. [Phase F: Visualization & Debugging](#12-phase-f-visualization--debugging)
13. [Launch Files — Putting It Together](#13-launch-files--putting-it-together)
14. [Student Challenges](#14-student-challenges)
15. [Grading Rubric](#15-grading-rubric)
16. [References & Resources](#16-references--resources)

---

## 1. Overview & Learning Objectives

### The Mission

You are the software team for **StreetBot** — a last-mile delivery robot. Your task is to implement the complete sensor pipeline in ROS 2:

1. **Simulate** 6 sensors (encoders, ultrasonic, LiDAR, IMU, camera, radar) in Gazebo
2. **Write** sensor model nodes that add realistic noise (from MOD-004 equations)
3. **Configure** sensor fusion using `robot_localization` (EKF)
4. **Implement** sensor health monitoring with Mahalanobis gating
5. **Test** in outdoor scenarios: rain, glass storefronts, uneven terrain

### What You Will Practice

| MOD-004 Concept | ROS 2 Implementation |
|----------------|---------------------|
| z = h(x) + b + v | Noise injection nodes publishing to sensor topics |
| Covariance propagation | Populating `covariance` fields in `Odometry` and `Imu` messages |
| Beam model (LiDAR) | Custom `LaserScan` post-processing node |
| Allan Variance parameters | IMU noise config in Gazebo `<imu>` plugin |
| Heteroscedastic noise (camera) | Depth-dependent noise in `PointCloud2` processing |
| Sensor fusion (EKF) | `robot_localization` `ekf_node` configuration |
| Mahalanobis gating | Custom health monitor node |
| Graceful degradation | Dynamic covariance scaling via ROS 2 parameters |

---

## 2. Prerequisites & Environment Setup

### System Requirements

```bash
# OS: Ubuntu 22.04 LTS
# ROS 2: Humble Hawksbill
# Gazebo: Fortress (ros-humble-ros-gz)
```

### Install Required Packages

```bash
# Core ROS 2 packages
sudo apt update
sudo apt install -y \
  ros-humble-robot-localization \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-xacro \
  ros-humble-rviz2 \
  ros-humble-tf2-ros \
  ros-humble-tf2-tools \
  ros-humble-tf2-geometry-msgs

# Sensor message types
sudo apt install -y \
  ros-humble-sensor-msgs \
  ros-humble-nav-msgs \
  ros-humble-geometry-msgs \
  ros-humble-diagnostic-msgs

# Gazebo simulation
sudo apt install -y \
  ros-humble-ros-gz \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-plugins

# Navigation (optional, for later integration)
sudo apt install -y \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox

# Python dependencies
pip3 install numpy scipy
```

### Source ROS 2 in Every Terminal

```bash
# Add to ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/streetbot_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 3. Workspace Structure

```
~/streetbot_ws/
├── src/
│   ├── streetbot_description/        # URDF/Xacro + meshes
│   │   ├── urdf/
│   │   │   ├── streetbot.urdf.xacro        # Main robot description
│   │   │   ├── sensors/
│   │   │   │   ├── lidar.xacro             # LiDAR mount + plugin
│   │   │   │   ├── imu.xacro              # IMU mount + plugin
│   │   │   │   ├── camera.xacro           # RGB-D camera + plugin
│   │   │   │   ├── ultrasonic.xacro       # Ultrasonic array + plugin
│   │   │   │   └── radar.xacro            # Radar mount + plugin
│   │   │   └── materials.xacro
│   │   ├── meshes/
│   │   ├── launch/
│   │   │   └── display.launch.py           # View URDF in RViz
│   │   ├── config/
│   │   │   └── rviz_config.rviz
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   │
│   ├── streetbot_gazebo/              # Simulation worlds + launch
│   │   ├── worlds/
│   │   │   ├── sidewalk_delivery.world     # Main test world
│   │   │   └── glass_storefront.world      # Glass failure scenario
│   │   ├── launch/
│   │   │   └── simulation.launch.py        # Spawn robot + world
│   │   ├── models/                         # Custom Gazebo models
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   │
│   ├── streetbot_sensor_models/       # ← STUDENT CODE HERE
│   │   ├── streetbot_sensor_models/        # Python package
│   │   │   ├── __init__.py
│   │   │   ├── encoder_noise_node.py       # Challenge 1
│   │   │   ├── ultrasonic_model_node.py    # Challenge 2
│   │   │   ├── lidar_beam_model_node.py    # Challenge 3
│   │   │   ├── imu_noise_node.py           # Challenge 4
│   │   │   ├── camera_depth_noise_node.py  # Challenge 5
│   │   │   ├── radar_model_node.py         # Challenge 6
│   │   │   └── utils/
│   │   │       ├── noise_models.py         # Shared noise functions
│   │   │       └── covariance_utils.py     # Covariance matrix helpers
│   │   ├── test/                           # Unit tests
│   │   │   ├── test_gaussian_noise.py
│   │   │   ├── test_covariance_propagation.py
│   │   │   └── test_mahalanobis.py
│   │   ├── launch/
│   │   │   └── sensor_models.launch.py
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── setup.cfg
│   │
│   ├── streetbot_fusion/              # ← STUDENT CODE HERE
│   │   ├── config/
│   │   │   ├── ekf_local.yaml              # Challenge 7: EKF config
│   │   │   └── ekf_global.yaml             # Optional: dual EKF
│   │   ├── streetbot_fusion/
│   │   │   ├── __init__.py
│   │   │   └── sensor_health_monitor.py    # Challenge 8
│   │   ├── launch/
│   │   │   └── fusion.launch.py
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── setup.cfg
│   │
│   └── streetbot_bringup/             # Top-level launch
│       ├── launch/
│       │   ├── streetbot_full.launch.py    # Everything together
│       │   └── streetbot_test.launch.py    # Test scenarios
│       ├── config/
│       │   └── rviz_full.rviz
│       ├── package.xml
│       └── CMakeLists.txt
│
├── build/          # (auto-generated by colcon)
├── install/        # (auto-generated by colcon)
└── log/            # (auto-generated by colcon)
```

### Create the Workspace

```bash
mkdir -p ~/streetbot_ws/src
cd ~/streetbot_ws/src

# Create packages
ros2 pkg create --build-type ament_cmake streetbot_description
ros2 pkg create --build-type ament_cmake streetbot_gazebo
ros2 pkg create --build-type ament_python streetbot_sensor_models
ros2 pkg create --build-type ament_python streetbot_fusion
ros2 pkg create --build-type ament_cmake streetbot_bringup

# Build
cd ~/streetbot_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 4. ROS 2 Core Concepts You Need

### Nodes

A **node** is a single process that does one job. In our system:

| Node | Job | Package |
|------|-----|---------|
| `robot_state_publisher` | Publishes robot URDF transforms | `robot_state_publisher` |
| `gazebo` | Simulates physics + sensors | `gazebo_ros` |
| `encoder_noise_node` | Adds noise to odometry | `streetbot_sensor_models` |
| `imu_noise_node` | Adds noise + bias to IMU | `streetbot_sensor_models` |
| `lidar_beam_model_node` | Applies beam model to LaserScan | `streetbot_sensor_models` |
| `camera_depth_noise_node` | Adds depth-dependent noise | `streetbot_sensor_models` |
| `ekf_filter_node` | EKF sensor fusion | `robot_localization` |
| `sensor_health_monitor` | Mahalanobis gating | `streetbot_fusion` |

### Topics

A **topic** is a named channel for messages. Publishers write to it, subscribers read from it.

**Naming convention:**

```
/streetbot/sensors/<sensor_name>/raw       ← Gazebo output (clean)
/streetbot/sensors/<sensor_name>/noisy     ← After noise node (realistic)
/streetbot/odom/filtered                   ← EKF output (fused)
```

### Messages

Key ROS 2 message types you will use:

```python
from sensor_msgs.msg import Imu, LaserScan, PointCloud2, Range, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseWithCovariance, TwistWithCovariance
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
```

### Quality of Service (QoS)

Sensors typically use **best-effort** reliability for speed:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
```

---

## 5. TF2 Frame Tree — The Backbone

### Frame Hierarchy

```
map                          ← Global frame (from SLAM or AMCL)
 └── odom                    ← Odometry frame (from EKF, continuous, drifts)
      └── base_footprint     ← Robot center on the ground plane
           └── base_link     ← Robot body center
                ├── lidar_link         ← LiDAR sensor frame
                ├── imu_link           ← IMU sensor frame
                ├── camera_link        ← RGB-D camera frame
                │    └── camera_depth_optical_frame
                ├── ultrasonic_front_link
                ├── ultrasonic_rear_link
                ├── radar_link         ← Radar sensor frame
                ├── left_wheel_link
                └── right_wheel_link
```

### Who Publishes What

| Transform | Published by | Notes |
|-----------|-------------|-------|
| `map → odom` | `robot_localization` (global EKF) or AMCL | Corrects drift |
| `odom → base_footprint` | `robot_localization` (local EKF) | Fused odometry |
| `base_footprint → base_link` | `robot_state_publisher` | Static (from URDF) |
| `base_link → <sensor>_link` | `robot_state_publisher` | Static (from URDF) |

### Critical Rule

**Only ONE node may publish each transform.** If both the Gazebo diff_drive plugin AND `robot_localization` try to publish `odom → base_link`, the robot will oscillate wildly.

**Solution:** Disable Gazebo's odometry TF broadcast:

```xml
<!-- In your Gazebo diff_drive plugin -->
<publish_odom_tf>false</publish_odom_tf>
```

Let `robot_localization` handle `odom → base_footprint`.

### Debugging TF

```bash
# View the full transform tree
ros2 run tf2_tools view_frames
# Opens a PDF showing all frames and publishers

# Check a specific transform
ros2 run tf2_ros tf2_echo odom base_link

# Monitor TF health
ros2 run tf2_ros tf2_monitor
```

---

## 6. Sensor Nodes & Topics — Full Map

```
┌──────────────────────────────────────────────────────────────────────┐
│                        GAZEBO SIMULATION                             │
│                                                                      │
│  diff_drive_plugin ──→ /streetbot/odom/raw        (nav_msgs/Odometry)│
│  imu_plugin        ──→ /streetbot/imu/raw         (sensor_msgs/Imu)  │
│  lidar_plugin      ──→ /streetbot/lidar/raw       (sensor_msgs/LaserScan) │
│  camera_plugin     ──→ /streetbot/camera/color/raw (sensor_msgs/Image)│
│                    ──→ /streetbot/camera/depth/raw (sensor_msgs/Image)│
│  sonar_plugin(×4)  ──→ /streetbot/ultrasonic/front/raw (sensor_msgs/Range)│
│                    ──→ /streetbot/ultrasonic/rear/raw  ...            │
│  (custom node)     ──→ /streetbot/radar/raw       (sensor_msgs/PointCloud2)│
└──────────┬───────────────────────────────────────────────────────────┘
           │
           ▼
┌──────────────────────────────────────────────────────────────────────┐
│                   SENSOR MODEL NODES (Student Code)                   │
│                   Package: streetbot_sensor_models                    │
│                                                                      │
│  encoder_noise_node:                                                 │
│    /streetbot/odom/raw ──→ add noise + covariance ──→ /streetbot/odom/noisy │
│                                                                      │
│  imu_noise_node:                                                     │
│    /streetbot/imu/raw ──→ add bias + noise ──→ /streetbot/imu/noisy  │
│                                                                      │
│  lidar_beam_model_node:                                              │
│    /streetbot/lidar/raw ──→ apply beam model ──→ /streetbot/lidar/noisy │
│                                                                      │
│  camera_depth_noise_node:                                            │
│    /streetbot/camera/depth/raw ──→ add Z²-noise ──→ /camera/depth/noisy │
│                                                                      │
│  ultrasonic_model_node:                                              │
│    /streetbot/ultrasonic/*/raw ──→ add temp bias ──→ /ultrasonic/*/noisy │
│                                                                      │
│  radar_model_node:                                                   │
│    /streetbot/radar/raw ──→ add clutter + angular blur ──→ /radar/noisy │
└──────────┬───────────────────────────────────────────────────────────┘
           │
           ▼
┌──────────────────────────────────────────────────────────────────────┐
│                   SENSOR FUSION (robot_localization)                  │
│                   Package: streetbot_fusion                           │
│                                                                      │
│  ekf_filter_node:                                                    │
│    Inputs:                                                           │
│      odom0: /streetbot/odom/noisy                                    │
│      imu0:  /streetbot/imu/noisy                                     │
│    Output:                                                           │
│      /streetbot/odom/filtered    (nav_msgs/Odometry)                 │
│      TF: odom → base_footprint                                      │
│                                                                      │
│  sensor_health_monitor:                                              │
│    Subscribes to all /noisy topics + /filtered                       │
│    Publishes: /streetbot/diagnostics (diagnostic_msgs/DiagnosticArray)│
│    Action: dynamically scales covariance when sensor degrades        │
└──────────┬───────────────────────────────────────────────────────────┘
           │
           ▼
┌──────────────────────────────────────────────────────────────────────┐
│                     VISUALIZATION (RViz2)                             │
│                                                                      │
│  Displays: TF tree, LaserScan, PointCloud2, Odometry arrows,        │
│            Covariance ellipses, Camera image, Diagnostics            │
└──────────────────────────────────────────────────────────────────────┘
```

---

## 7. Phase A: Robot Description (URDF/Xacro)

### Main URDF File Structure

File: `streetbot_description/urdf/streetbot.urdf.xacro`

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="streetbot">

  <!-- Include material definitions -->
  <xacro:include filename="$(find streetbot_description)/urdf/materials.xacro"/>

  <!-- Include sensor macros -->
  <xacro:include filename="$(find streetbot_description)/urdf/sensors/lidar.xacro"/>
  <xacro:include filename="$(find streetbot_description)/urdf/sensors/imu.xacro"/>
  <xacro:include filename="$(find streetbot_description)/urdf/sensors/camera.xacro"/>
  <xacro:include filename="$(find streetbot_description)/urdf/sensors/ultrasonic.xacro"/>
  <xacro:include filename="$(find streetbot_description)/urdf/sensors/radar.xacro"/>

  <!-- ==================== BASE ==================== -->
  <link name="base_footprint"/>

  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </joint>

  <link name="base_link">
    <visual>
      <geometry><box size="0.6 0.4 0.25"/></geometry>
    </visual>
    <collision>
      <geometry><box size="0.6 0.4 0.25"/></geometry>
    </collision>
    <inertial>
      <mass value="20.0"/>
      <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.8" iyz="0" izz="0.6"/>
    </inertial>
  </link>

  <!-- ==================== WHEELS ==================== -->
  <!-- Left and right drive wheels + caster(s) — differential drive -->
  <!-- (Students: define wheel links, joints, and Gazebo diff_drive plugin) -->

  <!-- ==================== SENSORS ==================== -->
  <!-- Each macro places the sensor link, joint, and Gazebo plugin -->
  <xacro:lidar_sensor parent="base_link" xyz="0.0 0.0 0.20" rpy="0 0 0"/>
  <xacro:imu_sensor parent="base_link" xyz="0.0 0.0 0.05" rpy="0 0 0"/>
  <xacro:rgbd_camera parent="base_link" xyz="0.25 0.0 0.15" rpy="0 0 0"/>
  <xacro:ultrasonic_array parent="base_link"/>
  <xacro:radar_sensor parent="base_link" xyz="-0.25 0.0 0.10" rpy="0 0 3.14159"/>

</robot>
```

### Sensor Xacro Example: IMU

File: `streetbot_description/urdf/sensors/imu.xacro`

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:macro name="imu_sensor" params="parent xyz rpy">
    <link name="imu_link">
      <visual>
        <geometry><box size="0.02 0.02 0.01"/></geometry>
      </visual>
      <inertial>
        <mass value="0.01"/>
        <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
      </inertial>
    </link>

    <joint name="imu_joint" type="fixed">
      <parent link="${parent}"/>
      <child link="imu_link"/>
      <origin xyz="${xyz}" rpy="${rpy}"/>
    </joint>

    <gazebo reference="imu_link">
      <sensor name="imu_sensor" type="imu">
        <always_on>true</always_on>
        <update_rate>200</update_rate>
        <topic>/streetbot/imu/raw</topic>
        <imu>
          <angular_velocity>
            <x><noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.0003</stddev>   <!-- rad/s — from Allan Variance -->
              <bias_mean>0.00087</bias_mean>  <!-- 0.05 deg/s bias -->
              <bias_stddev>0.00001</bias_stddev>
            </noise></x>
            <!-- Repeat for y, z -->
          </angular_velocity>
          <linear_acceleration>
            <x><noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.005</stddev>   <!-- m/s^2 -->
              <bias_mean>0.0049</bias_mean>  <!-- 0.5 mg bias -->
              <bias_stddev>0.0001</bias_stddev>
            </noise></x>
            <!-- Repeat for y, z -->
          </linear_acceleration>
        </imu>
        <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
          <ros>
            <remapping>~/out:=/streetbot/imu/raw</remapping>
          </ros>
          <frame_name>imu_link</frame_name>
        </plugin>
      </sensor>
    </gazebo>
  </xacro:macro>

</robot>
```

---

## 8. Phase B: Sensor Simulation in Gazebo

### Gazebo Sensor Plugins — Quick Reference

| Sensor | Gazebo Plugin | Output Topic | Message Type |
|--------|-------------|-------------|-------------|
| Wheel encoders | `libgazebo_ros_diff_drive.so` | `/streetbot/odom/raw` | `nav_msgs/Odometry` |
| IMU | `libgazebo_ros_imu_sensor.so` | `/streetbot/imu/raw` | `sensor_msgs/Imu` |
| 2D LiDAR | `libgazebo_ros_ray_sensor.so` | `/streetbot/lidar/raw` | `sensor_msgs/LaserScan` |
| RGB-D Camera | `libgazebo_ros_camera.so` | `/streetbot/camera/color/raw` | `sensor_msgs/Image` |
| Depth Camera | `libgazebo_ros_depth_camera.so` | `/streetbot/camera/depth/raw` | `sensor_msgs/Image` |
| Ultrasonic | `libgazebo_ros_ray_sensor.so` | `/streetbot/ultrasonic/*/raw` | `sensor_msgs/Range` |
| Radar | Custom node (no built-in Gazebo plugin) | `/streetbot/radar/raw` | `sensor_msgs/PointCloud2` |

### Important: Disable Gazebo's Odometry TF

In the diff_drive plugin config:

```xml
<plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
  <ros><remapping>odom:=/streetbot/odom/raw</remapping></ros>
  <left_joint>left_wheel_joint</left_joint>
  <right_joint>right_wheel_joint</right_joint>
  <wheel_separation>0.4</wheel_separation>
  <wheel_diameter>0.15</wheel_diameter>
  <max_wheel_torque>10</max_wheel_torque>
  <publish_odom>true</publish_odom>
  <publish_odom_tf>false</publish_odom_tf>   <!-- CRITICAL: let EKF handle TF -->
  <publish_wheel_tf>true</publish_wheel_tf>
  <odometry_frame>odom</odometry_frame>
  <robot_base_frame>base_footprint</robot_base_frame>
</plugin>
```

---

## 9. Phase C: Sensor Model Nodes (Student Code)

This is where you implement the MOD-004 math. Each node:

1. **Subscribes** to a `/raw` topic (clean Gazebo data)
2. **Applies** the sensor model (noise, bias, failures)
3. **Publishes** to a `/noisy` topic (realistic data)
4. **Populates** the `covariance` fields in the message

### Node Template

File: `streetbot_sensor_models/streetbot_sensor_models/node_template.py`

```python
#!/usr/bin/env python3
"""
Template for a sensor model node.
Students: copy this and implement the add_noise() method.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np

# Import the appropriate message type
# from sensor_msgs.msg import Imu
# from nav_msgs.msg import Odometry

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

class SensorModelNode(Node):
    def __init__(self):
        super().__init__('sensor_model_node')

        # Declare parameters (tunable at runtime)
        self.declare_parameter('noise_stddev', 0.01)
        self.declare_parameter('bias_mean', 0.0)
        self.declare_parameter('enabled', True)

        # Subscriber: clean data from Gazebo
        self.sub = self.create_subscription(
            Imu,                              # ← Change message type
            '/streetbot/imu/raw',             # ← Change topic
            self.callback,
            SENSOR_QOS
        )

        # Publisher: noisy data for fusion
        self.pub = self.create_publisher(
            Imu,                              # ← Same message type
            '/streetbot/imu/noisy',           # ← Change topic
            SENSOR_QOS
        )

        self.get_logger().info('Sensor model node started.')

    def callback(self, msg):
        """Receive clean data, add noise, publish noisy data."""
        if not self.get_parameter('enabled').value:
            self.pub.publish(msg)  # Pass through if disabled
            return

        noisy_msg = self.add_noise(msg)
        self.pub.publish(noisy_msg)

    def add_noise(self, msg):
        """
        ============================================================
        STUDENT: IMPLEMENT THIS METHOD
        Apply the sensor model from MOD-004 to the message.
        Return the modified message with realistic noise.
        ============================================================
        """
        raise NotImplementedError("Students must implement add_noise()")


def main(args=None):
    rclpy.init(args=args)
    node = SensorModelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Covariance in ROS 2 Messages

ROS 2 `Odometry` and `Imu` messages contain `covariance` arrays. You MUST fill these correctly for `robot_localization` to work.

**`nav_msgs/Odometry` covariance** — 6×6 matrix, row-major, 36 elements:

```python
# pose.covariance = [σ²_x, 0, 0, 0, 0, 0,     ← row 1 (x)
#                    0, σ²_y, 0, 0, 0, 0,       ← row 2 (y)
#                    0, 0, σ²_z, 0, 0, 0,       ← row 3 (z)
#                    0, 0, 0, σ²_roll, 0, 0,    ← row 4 (roll)
#                    0, 0, 0, 0, σ²_pitch, 0,   ← row 5 (pitch)
#                    0, 0, 0, 0, 0, σ²_yaw]     ← row 6 (yaw)

# For a 2D robot (x, y, yaw only), set unused axes to a large value:
msg.pose.covariance[0]  = sigma_x ** 2       # x variance
msg.pose.covariance[7]  = sigma_y ** 2       # y variance
msg.pose.covariance[14] = 1e6               # z (unused — set large)
msg.pose.covariance[21] = 1e6               # roll (unused)
msg.pose.covariance[28] = 1e6               # pitch (unused)
msg.pose.covariance[35] = sigma_yaw ** 2     # yaw variance
```

**`sensor_msgs/Imu` covariance** — three 3×3 matrices (9 elements each):

```python
# orientation_covariance — [σ²_roll, ..., σ²_yaw]  (3x3 row-major)
# angular_velocity_covariance — [σ²_wx, ..., σ²_wz]
# linear_acceleration_covariance — [σ²_ax, ..., σ²_az]

msg.orientation_covariance[0] = sigma_roll ** 2
msg.orientation_covariance[4] = sigma_pitch ** 2
msg.orientation_covariance[8] = sigma_yaw ** 2

msg.angular_velocity_covariance[0] = sigma_gyro ** 2  # Same for all axes
msg.angular_velocity_covariance[4] = sigma_gyro ** 2
msg.angular_velocity_covariance[8] = sigma_gyro ** 2
```

**Important:** Setting a covariance value to `-1` tells `robot_localization` to IGNORE that measurement. Setting it to `0` is invalid and will crash the filter.

---

## 10. Phase D: Sensor Fusion with robot_localization

### EKF Configuration

File: `streetbot_fusion/config/ekf_local.yaml`

```yaml
# =============================================================
# EKF Node Configuration for StreetBot
# This is the LOCAL odometry EKF (odom frame)
# Students: fill in the covariance values based on your sensor models
# =============================================================

ekf_filter_node:
  ros__parameters:
    # How often to publish the fused estimate
    frequency: 50.0            # Hz — should be >= fastest sensor
    sensor_timeout: 0.1        # seconds — reject data older than this

    # Frames
    odom_frame: odom
    base_link_frame: base_footprint
    world_frame: odom          # For local EKF, world = odom
    map_frame: map

    # Publish the odom → base_footprint transform
    publish_tf: true

    # ─── ODOMETRY INPUT (from wheel encoders + noise node) ───
    odom0: /streetbot/odom/noisy
    odom0_config: [true,  true,  false,   # x, y, z
                   false, false, true,    # roll, pitch, yaw
                   false, false, false,   # vx, vy, vz
                   false, false, false,   # vroll, vpitch, vyaw
                   false, false, false]   # ax, ay, az
    odom0_differential: false
    odom0_relative: false
    odom0_queue_size: 10

    # ─── IMU INPUT (from IMU + noise node) ───
    imu0: /streetbot/imu/noisy
    imu0_config: [false, false, false,   # x, y, z        (IMU has no position)
                  true,  true,  true,    # roll, pitch, yaw
                  false, false, false,   # vx, vy, vz
                  true,  true,  true,    # vroll, vpitch, vyaw
                  true,  true,  true]    # ax, ay, az
    imu0_differential: false
    imu0_relative: false
    imu0_queue_size: 10
    imu0_remove_gravitational_acceleration: true

    # ─── PROCESS NOISE COVARIANCE (Q matrix) ───
    # These represent how much the STATE drifts per second
    # Larger values = trust measurements more, model less
    # STUDENT: tune these based on your noise analysis
    process_noise_covariance:
      [0.05,  0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,
       0.0,   0.05,  0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,
       0.0,   0.0,   0.06,  0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,
       0.0,   0.0,   0.0,   0.03,  0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,
       0.0,   0.0,   0.0,   0.0,   0.03,  0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,
       0.0,   0.0,   0.0,   0.0,   0.0,   0.06,  0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,
       0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.025, 0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,
       0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.025, 0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,
       0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.04,  0.0,   0.0,   0.0,   0.0,   0.0,   0.0,
       0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.01,  0.0,   0.0,   0.0,   0.0,   0.0,
       0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.01,  0.0,   0.0,   0.0,   0.0,
       0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.02,  0.0,   0.0,   0.0,
       0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.01,  0.0,   0.0,
       0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.01,  0.0,
       0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.015]
```

### How the EKF State Vector Maps to the Config Matrix

The `robot_localization` EKF maintains a **15-dimensional state**:

```
State = [x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]
Index:   0  1  2   3     4      5   6   7   8    9      10      11   12  13  14
```

Each `odomN_config` and `imuN_config` is a 15-element boolean array specifying which state variables that sensor contributes to. Setting `true` means "use this measurement," `false` means "ignore."

---

## 11. Phase E: Sensor Health Monitoring Node

File: `streetbot_fusion/streetbot_fusion/sensor_health_monitor.py`

```python
#!/usr/bin/env python3
"""
Sensor Health Monitor — Mahalanobis Gating
Monitors the innovation (prediction error) of each sensor.
If d_M > threshold, flags the sensor as unhealthy.

============================================================
STUDENT: Complete the compute_mahalanobis() and
         degrade_sensor() methods.
============================================================
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import numpy as np


class SensorHealthMonitor(Node):
    def __init__(self):
        super().__init__('sensor_health_monitor')

        # Parameters
        self.declare_parameter('mahalanobis_threshold', 3.0)
        self.declare_parameter('window_size', 20)

        # Subscribe to filtered (fused) output
        self.sub_filtered = self.create_subscription(
            Odometry, '/streetbot/odom/filtered',
            self.filtered_callback, 10)

        # Subscribe to individual noisy sensors
        self.sub_odom = self.create_subscription(
            Odometry, '/streetbot/odom/noisy',
            self.odom_callback, 10)

        self.sub_imu = self.create_subscription(
            Imu, '/streetbot/imu/noisy',
            self.imu_callback, 10)

        # Diagnostics publisher
        self.diag_pub = self.create_publisher(
            DiagnosticArray, '/streetbot/diagnostics', 10)

        # State storage
        self.latest_filtered = None
        self.innovation_history = {
            'odom': [],
            'imu': [],
        }

        # Timer for periodic health check
        self.create_timer(0.5, self.health_check)  # 2 Hz

    def filtered_callback(self, msg):
        self.latest_filtered = msg

    def odom_callback(self, msg):
        if self.latest_filtered is None:
            return
        # STUDENT: compute innovation and store in history
        # innovation = z_measured - z_predicted (from filtered state)
        pass

    def imu_callback(self, msg):
        if self.latest_filtered is None:
            return
        # STUDENT: compute innovation for IMU data
        pass

    def compute_mahalanobis(self, innovation, covariance):
        """
        ============================================================
        STUDENT CHALLENGE 8: Implement Mahalanobis distance
        
        d_M = sqrt( innovation^T @ S^{-1} @ innovation )
        
        where S = covariance matrix of the innovation
        
        Inputs:
            innovation: np.array — difference (z - z_hat)
            covariance: np.array — innovation covariance S
        Returns:
            float — Mahalanobis distance
        ============================================================
        """
        raise NotImplementedError("Implement Mahalanobis distance")

    def health_check(self):
        """Periodic check of all sensor health."""
        threshold = self.get_parameter('mahalanobis_threshold').value
        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = self.get_clock().now().to_msg()

        for sensor_name, history in self.innovation_history.items():
            status = DiagnosticStatus()
            status.name = f"streetbot/sensor/{sensor_name}"

            if len(history) == 0:
                status.level = DiagnosticStatus.STALE
                status.message = "No data received"
            else:
                avg_d_m = np.mean(history[-20:])  # Last 20 samples
                status.values.append(
                    KeyValue(key='avg_mahalanobis', value=f'{avg_d_m:.3f}'))

                if avg_d_m < threshold:
                    status.level = DiagnosticStatus.OK
                    status.message = f"Healthy (d_M={avg_d_m:.2f})"
                else:
                    status.level = DiagnosticStatus.WARN
                    status.message = f"DEGRADED (d_M={avg_d_m:.2f} > {threshold})"
                    # STUDENT: call degrade_sensor() to increase covariance

            diag_msg.status.append(status)

        self.diag_pub.publish(diag_msg)

    def degrade_sensor(self, sensor_name, scale_factor=10.0):
        """
        ============================================================
        STUDENT CHALLENGE 8b: Implement graceful degradation
        
        When a sensor is flagged, increase its covariance by
        scale_factor, effectively reducing its weight in the EKF.
        
        Approach: publish a modified message with scaled covariance,
        OR use ROS 2 parameter server to dynamically reconfigure
        the EKF's process noise for that sensor.
        ============================================================
        """
        raise NotImplementedError("Implement graceful degradation")


def main(args=None):
    rclpy.init(args=args)
    node = SensorHealthMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

---

## 12. Phase F: Visualization & Debugging

### RViz2 Displays to Add

| Display Type | Topic | What It Shows |
|-------------|-------|--------------|
| `TF` | (automatic) | Full frame tree — verify all connections |
| `LaserScan` | `/streetbot/lidar/noisy` | LiDAR point cloud around robot |
| `Odometry` | `/streetbot/odom/filtered` | Fused pose arrow + covariance ellipse |
| `Odometry` | `/streetbot/odom/raw` | Raw odom (compare drift vs. fused) |
| `Image` | `/streetbot/camera/color/raw` | Camera feed |
| `Range` | `/streetbot/ultrasonic/front/noisy` | Ultrasonic cone visualization |
| `DiagnosticDisplay` | `/streetbot/diagnostics` | Sensor health status |

### Useful Debugging Commands

```bash
# List all active topics
ros2 topic list | grep streetbot

# Check message rate (should match expected Hz)
ros2 topic hz /streetbot/imu/noisy

# Echo a single message (inspect covariance fields)
ros2 topic echo /streetbot/odom/noisy --once

# Check that TF tree is complete
ros2 run tf2_tools view_frames

# Monitor EKF health
ros2 topic echo /streetbot/odom/filtered --field pose.covariance

# Record data for offline analysis
ros2 bag record -a -o streetbot_test_run
```

---

## 13. Launch Files — Putting It Together

File: `streetbot_bringup/launch/streetbot_full.launch.py`

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    desc_dir = get_package_share_directory('streetbot_description')
    gazebo_dir = get_package_share_directory('streetbot_gazebo')
    fusion_dir = get_package_share_directory('streetbot_fusion')

    return LaunchDescription([

        # 1. Robot State Publisher (URDF → TF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': open(
                    os.path.join(desc_dir, 'urdf', 'streetbot.urdf.xacro')
                ).read()
                # NOTE: In practice, use xacro processing here
            }]
        ),

        # 2. Gazebo Simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_dir, 'launch', 'simulation.launch.py')
            )
        ),

        # 3. Sensor Model Nodes (Student Code)
        Node(package='streetbot_sensor_models',
             executable='encoder_noise_node', name='encoder_noise'),
        Node(package='streetbot_sensor_models',
             executable='imu_noise_node', name='imu_noise'),
        Node(package='streetbot_sensor_models',
             executable='lidar_beam_model_node', name='lidar_noise'),
        Node(package='streetbot_sensor_models',
             executable='camera_depth_noise_node', name='camera_noise'),
        Node(package='streetbot_sensor_models',
             executable='ultrasonic_model_node', name='ultrasonic_noise'),
        Node(package='streetbot_sensor_models',
             executable='radar_model_node', name='radar_noise'),

        # 4. EKF Sensor Fusion
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[os.path.join(fusion_dir, 'config', 'ekf_local.yaml')],
            remappings=[('odometry/filtered', '/streetbot/odom/filtered')]
        ),

        # 5. Sensor Health Monitor
        Node(package='streetbot_fusion',
             executable='sensor_health_monitor',
             name='health_monitor'),

        # 6. RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('streetbot_bringup'),
                'config', 'rviz_full.rviz')]
        ),
    ])
```

### Build and Run

```bash
cd ~/streetbot_ws
colcon build --symlink-install
source install/setup.bash

# Launch everything
ros2 launch streetbot_bringup streetbot_full.launch.py
```

---

## 14. Student Challenges

Each challenge maps directly to a MOD-004 concept. Complete them in order — each builds on the previous.

---

### Challenge 1: Encoder Noise Node
**File:** `streetbot_sensor_models/encoder_noise_node.py`  
**MOD-004 Block:** 1 (Wheel Encoders)

**Task:**
- Subscribe to `/streetbot/odom/raw` (`nav_msgs/Odometry`)
- Add Gaussian noise to `pose.pose.position.x`, `.y` and `pose.pose.orientation` (yaw)
- **Implement covariance propagation:** compute P from the Jacobian F and noise Q
- Populate `pose.covariance` (6×6 row-major) correctly
- Publish to `/streetbot/odom/noisy`

**Key equations:**
- P_{k+1} = F · P_k · Fᵀ + G · Q · Gᵀ
- F = [1 0 −Δs·sin(θ+Δθ/2); 0 1 Δs·cos(θ+Δθ/2); 0 0 1]

**Validation:** Drive StreetBot 50m in a straight line. Plot raw vs. noisy odometry. The noisy path should drift realistically (2–5m after 50m with turns).

---

### Challenge 2: Ultrasonic Model Node
**File:** `streetbot_sensor_models/ultrasonic_model_node.py`  
**MOD-004 Block:** 2 (Ultrasonic)

**Task:**
- Subscribe to `/streetbot/ultrasonic/front/raw` (`sensor_msgs/Range`)
- Apply temperature-dependent bias: c(T) = 331.3 + 0.606·T
- Add Gaussian noise: σ ≈ 5–15 mm
- Model specular reflection failure: if surface angle > 5° (from simulation), return `max_range`
- Publish to `/streetbot/ultrasonic/front/noisy`

**Key equation:** z = (c_assumed / c_true) · d_true + v,  v ~ N(0, σ²)

**Validation:** Place StreetBot facing a glass wall. Verify ultrasonic detects it while LiDAR returns max_range.

---

### Challenge 3: LiDAR Beam Model Node
**File:** `streetbot_sensor_models/lidar_beam_model_node.py`  
**MOD-004 Block:** 3 (LiDAR)

**Task:**
- Subscribe to `/streetbot/lidar/raw` (`sensor_msgs/LaserScan`)
- Implement the 4-component beam model:
  - p_hit: add Gaussian noise N(0, σ²_r) to each beam
  - p_short: with probability α_short, replace with shorter reading (exponential)
  - p_max: with probability α_max, replace with `range_max`
  - p_rand: with probability α_rand, replace with uniform random in [0, range_max]
- Publish to `/streetbot/lidar/noisy`

**Key equation:** p(z|x,m) = 0.85·p_hit + 0.05·p_short + 0.05·p_max + 0.05·p_rand

**Validation:** Visualize raw vs. noisy scans in RViz. The noisy scan should show occasional short readings, max-range dropouts, and random speckle.

---

### Challenge 4: IMU Noise Node
**File:** `streetbot_sensor_models/imu_noise_node.py`  
**MOD-004 Block:** 4 (IMU)

**Task:**
- Subscribe to `/streetbot/imu/raw` (`sensor_msgs/Imu`)
- Add gyroscope bias drift: b_g(k+1) = b_g(k) + w_b, w_b ~ N(0, σ²_bg)
- Add white noise to angular velocity: n_g ~ N(0, σ²_g)
- Add accelerometer bias + noise similarly
- Populate all three covariance arrays correctly
- Publish to `/streetbot/imu/noisy`

**Key equations:**
- ω_meas = ω_true + b_g + n_g
- f_meas = R^T(a − g) + b_a + n_a
- Position error ∝ T² from accelerometer integration

**Validation:** Keep StreetBot stationary for 60 seconds. Plot gyroscope bias drift over time. It should show random walk behavior.

---

### Challenge 5: Camera Depth Noise Node
**File:** `streetbot_sensor_models/camera_depth_noise_node.py`  
**MOD-004 Block:** 5 (Camera)

**Task:**
- Subscribe to `/streetbot/camera/depth/raw` (`sensor_msgs/Image`)
- For each pixel, add depth-dependent noise: σ_Z = (Z²/fB) · σ_d
- This is **heteroscedastic** — noise grows with depth²
- Model failure modes: set depth to NaN for pixels with Z > 8m or in sunlit regions
- Publish to `/streetbot/camera/depth/noisy`

**Key equation:** σ_Z = (Z² / (f·B)) · σ_d — quadratic growth

**Validation:** View raw vs. noisy depth images. Close objects should look clean, far objects should be noisy, and very far objects should show NaN gaps.

---

### Challenge 6: Radar Model Node
**File:** `streetbot_sensor_models/radar_model_node.py`  
**MOD-004 Block:** 6 (Radar)

**Task:**
- Since Gazebo has no built-in radar plugin, create a node that:
  - Subscribes to `/streetbot/lidar/raw` (use LiDAR as proxy for range-bearing)
  - Adds Doppler velocity field (compute from consecutive scans or ground truth twist)
  - Degrades angular resolution: merge nearby points within Δθ ≈ 5°
  - Adds clutter points (random detections from multipath)
  - Publishes as `/streetbot/radar/noisy` (`sensor_msgs/PointCloud2`)

**Key equation:** v_r = λ · f_Doppler / 2, Δr = c / (2·BW)

**Validation:** Visualize radar point cloud. It should be sparse (compared to LiDAR) but each point should carry a velocity field.

---

### Challenge 7: EKF Configuration Tuning
**File:** `streetbot_fusion/config/ekf_local.yaml`  
**MOD-004 Blocks:** 7–8 (Fusion)

**Task:**
- Configure the `robot_localization` EKF to fuse odometry + IMU
- Set the correct `odom0_config` and `imu0_config` boolean arrays
- Tune `process_noise_covariance` based on your sensor noise parameters
- Compare: odometry-only vs. IMU-only vs. fused. Record trajectories.
- Generate a plot showing covariance ellipse over time for each case.

**Deliverable:** A tuned `ekf_local.yaml` with comments justifying each parameter choice.

**Validation:** Drive StreetBot in a square (10m × 10m). Measure the return-to-origin error for: (a) raw odom, (b) EKF fused. The fused error should be significantly smaller.

---

### Challenge 8: Sensor Health Monitor
**File:** `streetbot_fusion/sensor_health_monitor.py`  
**MOD-004 Block:** 8 (Fusion Architecture)

**Task:**
- Implement `compute_mahalanobis()` using the innovation and covariance
- Implement `degrade_sensor()` to scale covariance when d_M > threshold
- Test with a scenario: at t=30s, simulate camera failure (set all depth to NaN)
- Verify that the health monitor detects the failure and the EKF continues with remaining sensors

**Deliverable:** A `/streetbot/diagnostics` topic showing real-time health of each sensor, and a log showing the degradation/recovery cycle.

---

### Challenge 9: Full Integration Test
**MOD-004:** All Blocks

**Task:** Run the complete StreetBot pipeline through 3 test scenarios:

| Scenario | What Fails | Expected Behavior |
|----------|-----------|-------------------|
| A: Normal sidewalk | Nothing | All sensors healthy, low covariance |
| B: Glass storefront | LiDAR (pass-through), camera depth (IR through glass) | Ultrasonic + radar compensate, d_M spikes on LiDAR |
| C: Rain + night | Camera (dark, wet lens), ultrasonic (rain noise) | Radar + IMU + LiDAR carry the load |

**Deliverable:** For each scenario, record a rosbag and produce:
1. Plot of x,y trajectory: ground truth vs. fused estimate
2. Plot of covariance trace over time
3. Sensor health diagnostics log
4. Written analysis: which sensors failed? Did the system degrade gracefully?

---

### Challenge 10: Comparison Report — MediBot vs. StreetBot
**MOD-004:** Transfer Analysis

**Task:** Write a 2-page technical report answering:
1. Which sensor parameters needed to change for outdoor use? Why?
2. Which fusion architecture changes were needed?
3. What NEW failure modes appeared outdoors that didn't exist in the hospital?
4. If you could add ONE more sensor to StreetBot, what would it be and why?

---

## 15. Grading Rubric

| Challenge | Weight | Criteria |
|-----------|--------|----------|
| 1. Encoder noise + covariance | 10% | Correct Jacobian, P grows realistically |
| 2. Ultrasonic model | 8% | Temperature bias, specular failure modeled |
| 3. LiDAR beam model | 10% | All 4 components implemented, visual verification |
| 4. IMU noise + bias drift | 10% | Bias random walk, covariance arrays correct |
| 5. Camera depth noise | 8% | Heteroscedastic (Z²), failure modes |
| 6. Radar model | 8% | Doppler, angular degradation, clutter |
| 7. EKF tuning | 12% | Justified parameters, comparison plots |
| 8. Health monitor | 12% | Mahalanobis, graceful degradation working |
| 9. Integration test | 15% | 3 scenarios, rosbags, trajectory plots, analysis |
| 10. Comparison report | 7% | Clear reasoning, correct physics |

**Bonus (up to +10%):** Implement a dual-EKF setup (local + global), add GPS simulation for outdoor, or implement particle filter as alternative to EKF.

---

## 16. References & Resources

### ROS 2 Documentation
- ROS 2 Humble docs: https://docs.ros.org/en/humble/
- robot_localization wiki: http://docs.ros.org/en/noetic/api/robot_localization/html/
- REP-103 (Standard Units): https://www.ros.org/reps/rep-0103.html
- REP-105 (Coordinate Frames): https://www.ros.org/reps/rep-0105.html

### Sensor Fusion Tutorials
- methylDragon's ROS sensor fusion tutorial: https://github.com/methylDragon/ros-sensor-fusion-tutorial
- Automatic Addison — Sensor Fusion with robot_localization: https://automaticaddison.com/sensor-fusion-using-the-robot-localization-package-ros-2/

### Textbooks (same as MOD-004)
1. Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic Robotics.* MIT Press.
2. Siegwart, R., Nourbakhsh, I.R., & Scaramuzza, D. (2011). *Introduction to Autonomous Mobile Robots.* MIT Press.
3. Titterton, D.H. & Weston, J.L. (2004). *Strapdown Inertial Navigation Technology.* IET.

### Helpful ROS 2 Commands Cheat Sheet

```bash
# Build one package
colcon build --packages-select streetbot_sensor_models --symlink-install

# Run a single node for testing
ros2 run streetbot_sensor_models encoder_noise_node

# Set a parameter at runtime
ros2 param set /encoder_noise noise_stddev 0.05

# Record specific topics
ros2 bag record /streetbot/odom/raw /streetbot/odom/noisy /streetbot/odom/filtered -o test_run

# Play back a recording
ros2 bag play test_run

# View TF tree as PDF
ros2 run tf2_tools view_frames && xdg-open frames_*.pdf
```

---

*MOD-004 ROS 2 Implementation Guide v1.0 — Course Architect Engine*  
*Mobile Robots Course — Sensor Modeling, Measurement & Fusion*
