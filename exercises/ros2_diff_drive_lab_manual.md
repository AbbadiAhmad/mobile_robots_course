# ROS 2 Differential Drive Robot
## Complete Student Lab Manual

**Platform:** Ubuntu 24.04 LTS · ROS 2 Jazzy Jalisco · Gazebo Harmonic  
**Level:** Undergraduate Engineering — Third Year  
**Estimated Time:** 8–10 hours (across multiple lab sessions)

---

> **How to use this manual:**  
> Each section begins with a short concept introduction (what it is and why it matters), followed by step-by-step implementation commands. Every command is complete and copy-pasteable. Results are verified at the end of each section before moving on.  
> Do **not** skip verification steps — they catch errors early and save time later.

---

## Table of Contents

1. [Environment Setup](#1-environment-setup)
2. [ROS 2 Workspace and Package](#2-ros-2-workspace-and-package)
3. [URDF — Robot Description](#3-urdf--robot-description)
4. [Visualizing the Robot in RViz2](#4-visualizing-the-robot-in-rviz2)
5. [TF — Transform Frames](#5-tf--transform-frames)
6. [Publisher and Subscriber Nodes](#6-publisher-and-subscriber-nodes)
7. [Odometry Node — Full Implementation](#7-odometry-node--full-implementation)
8. [Launch Files](#8-launch-files)
9. [Gazebo Harmonic Integration](#9-gazebo-harmonic-integration)
10. [Keyboard Control and cmd_vel](#10-keyboard-control-and-cmd_vel)
11. [TF Tree Diagram](#11-tf-tree-diagram)
12. [Troubleshooting Reference](#12-troubleshooting-reference)

---

## 1. Environment Setup

### Concept
ROS 2 (Robot Operating System 2) is a middleware framework — a communication layer — that connects independent programs called **nodes**. These nodes talk to each other by publishing and subscribing to **topics**, or by calling **services**. Ubuntu 24.04 with ROS 2 Jazzy is the current Long-Term Support (LTS) combination recommended for new projects. Setting up the environment correctly is the foundation for everything that follows.

---

### 1.1 Install ROS 2 Jazzy

Open a terminal (`Ctrl+Alt+T`) and run these commands one by one:

```bash
# Set locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Jazzy desktop (includes RViz2, demo nodes, etc.)
sudo apt update
sudo apt install -y ros-jazzy-desktop

# Install build tools
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool
```

### 1.2 Install Gazebo Harmonic

```bash
# Add Gazebo repository
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
  http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt update
sudo apt install -y gz-harmonic

# ROS-Gazebo bridge package
sudo apt install -y ros-jazzy-ros-gz
```

### 1.3 Install Additional ROS Packages

```bash
sudo apt install -y \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-xacro \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-nav-msgs \
  ros-jazzy-tf2-tools \
  ros-jazzy-tf2-ros \
  ros-jazzy-rqt \
  ros-jazzy-rqt-common-plugins
```

### 1.4 Configure Your Shell (Permanent)

Instead of sourcing ROS manually every session, add it to your shell startup file:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 1.5 Verify Installation

```bash
# Should print ROS environment variables
printenv | grep ROS

# Should show jazzy
echo $ROS_DISTRO

# Run a quick test (open two terminals)
# Terminal 1:
ros2 run demo_nodes_cpp talker

# Terminal 2:
ros2 run demo_nodes_cpp listener
```

**Expected result:** Terminal 2 should print messages like `[INFO] [listener]: I heard: [Hello World: 1]`. Press `Ctrl+C` in both terminals to stop.

---

## 2. ROS 2 Workspace and Package

### Concept
A **workspace** is a directory that holds all your robot's code. Inside it, code is organized into **packages** — each package is a self-contained unit with nodes, launch files, configuration, and dependencies. The build tool `colcon` compiles all packages in the workspace and generates the files needed to run them. You must **source** the workspace after building so ROS can find your packages.

---

### 2.1 Create the Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source ~/ros2_ws/install/setup.bash
```

Add workspace sourcing permanently:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2.2 Create the Robot Package

```bash
cd ~/ros2_ws/src

ros2 pkg create \
  --build-type ament_python \
  --license MIT \
  --dependencies rclpy std_msgs geometry_msgs nav_msgs sensor_msgs tf2_ros \
  diff_drive_robot
```

### 2.3 Create Required Directories

```bash
mkdir -p ~/ros2_ws/src/diff_drive_robot/urdf
mkdir -p ~/ros2_ws/src/diff_drive_robot/launch
mkdir -p ~/ros2_ws/src/diff_drive_robot/config
mkdir -p ~/ros2_ws/src/diff_drive_robot/worlds
mkdir -p ~/ros2_ws/src/diff_drive_robot/diff_drive_robot
```

### 2.4 Update package.xml

Open the file with any text editor:

```bash
nano ~/ros2_ws/src/diff_drive_robot/package.xml
```

Replace the contents with:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>diff_drive_robot</name>
  <version>1.0.0</version>
  <description>Differential drive robot for ROS 2 Jazzy lab</description>
  <maintainer email="student@university.edu">Student</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>robot_state_publisher</depend>
  <depend>joint_state_publisher</depend>
  <depend>joint_state_publisher_gui</depend>
  <depend>xacro</depend>

  <buildtool_depend>ament_python</buildtool_depend>
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

Save: `Ctrl+O`, `Enter`, `Ctrl+X`.

### 2.5 Update setup.py

```bash
nano ~/ros2_ws/src/diff_drive_robot/setup.py
```

Replace with:

```python
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'diff_drive_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@university.edu',
    description='Differential drive robot for ROS 2 Jazzy lab',
    license='MIT',
    entry_points={
        'console_scripts': [
            'odom_node = diff_drive_robot.odom_node:main',
            'talker_node = diff_drive_robot.talker_node:main',
            'listener_node = diff_drive_robot.listener_node:main',
        ],
    },
)
```

### 2.6 Build and Verify

```bash
cd ~/ros2_ws
colcon build --packages-select diff_drive_robot
source ~/.bashrc
```

**Expected result:** `Summary: 1 packages finished`

---

## 3. URDF — Robot Description

### Concept
**URDF (Unified Robot Description Format)** is an XML file that describes the physical structure of your robot. A URDF contains **links** (rigid bodies like the chassis and wheels) and **joints** (connections between links that can be fixed, revolute, or continuous). The robot_state_publisher node reads this file and broadcasts the positions of all links as TF frames, which other nodes (like RViz and Gazebo) use to render and simulate the robot.

---

### 3.1 Create the URDF File

```bash
nano ~/ros2_ws/src/diff_drive_robot/urdf/diff_robot.urdf
```

Paste the following complete URDF:

```xml
<?xml version="1.0"?>
<robot name="diff_robot">

  <!-- ===================== MATERIALS ===================== -->
  <material name="blue">
    <color rgba="0.2 0.4 0.8 1.0"/>
  </material>
  <material name="black">
    <color rgba="0.1 0.1 0.1 1.0"/>
  </material>
  <material name="grey">
    <color rgba="0.5 0.5 0.5 1.0"/>
  </material>

  <!-- ===================== BASE LINK ===================== -->
  <!-- The main chassis of the robot (40cm x 30cm x 10cm box) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.4 0.3 0.1"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.4 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0"
               iyy="0.02" iyz="0.0" izz="0.04"/>
    </inertial>
  </link>

  <!-- ===================== LEFT WHEEL ===================== -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.08" length="0.04"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.08" length="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0"
               iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint: base_link → left_wheel -->
  <!-- xyz: 0 (no forward/back), +0.17 (left side), -0.05 (below chassis) -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0.0 0.17 -0.05" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
    <dynamics damping="0.1"/>
  </joint>

  <!-- ===================== RIGHT WHEEL ===================== -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.08" length="0.04"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.08" length="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0"
               iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint: base_link → right_wheel -->
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0.0 -0.17 -0.05" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
    <dynamics damping="0.1"/>
  </joint>

  <!-- ===================== CASTER WHEEL (front) ===================== -->
  <!-- A fixed sphere at the front-bottom acts as a passive support caster -->
  <link name="caster_wheel">
    <visual>
      <geometry>
        <sphere radius="0.04"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0"
               iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel"/>
    <origin xyz="0.15 0.0 -0.09" rpy="0 0 0"/>
  </joint>

  <!-- ===================== base_footprint ===================== -->
  <!-- Virtual frame at ground level — required by navigation stack -->
  <link name="base_footprint"/>

  <joint name="base_footprint_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0.0 0.0 0.09" rpy="0 0 0"/>
  </joint>

</robot>
```

Save: `Ctrl+O`, `Enter`, `Ctrl+X`.

### 3.2 Validate the URDF

```bash
# Install urdf parser tools
sudo apt install -y ros-jazzy-urdf

# Check URDF for errors
check_urdf ~/ros2_ws/src/diff_drive_robot/urdf/diff_robot.urdf
```

**Expected result:**
```
robot name is: diff_robot
---------- Successfully Parsed XML ---------------
root Link: base_footprint has 1 child(ren)
    child(1):  base_link
        child(1):  left_wheel
        child(2):  right_wheel
        child(3):  caster_wheel
```

### 3.3 Understand the Robot Structure

The robot has these components:

| Link | Shape | Purpose |
|---|---|---|
| `base_footprint` | None (virtual) | Ground-level reference frame |
| `base_link` | Box 40×30×10 cm | Main chassis |
| `left_wheel` | Cylinder r=8 cm | Drive wheel |
| `right_wheel` | Cylinder r=8 cm | Drive wheel |
| `caster_wheel` | Sphere r=4 cm | Passive front support |

| Joint | Type | Connects |
|---|---|---|
| `base_footprint_joint` | fixed | base_footprint → base_link |
| `left_wheel_joint` | continuous | base_link → left_wheel |
| `right_wheel_joint` | continuous | base_link → right_wheel |
| `caster_joint` | fixed | base_link → caster_wheel |

---

## 4. Visualizing the Robot in RViz2

### Concept
**RViz2** is the standard ROS 2 visualization tool. It reads TF frames and topic data and renders them graphically. The `robot_state_publisher` node is the bridge between your URDF file and the visualization — it reads the URDF and publishes TF transforms for each link. The `joint_state_publisher_gui` provides a GUI slider to manually move the joints. Together, these let you inspect the robot model before connecting any simulation or hardware.

---

### 4.1 Run robot_state_publisher

Open **Terminal 1**:

```bash
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(cat ~/ros2_ws/src/diff_drive_robot/urdf/diff_robot.urdf)"
```

**What this does:** Reads the URDF, parses all links and joints, and starts publishing `/tf` and `/tf_static` messages.

### 4.2 Run joint_state_publisher_gui

Open **Terminal 2**:

```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

A small window with sliders will appear for `left_wheel_joint` and `right_wheel_joint`.

### 4.3 Open RViz2

Open **Terminal 3**:

```bash
rviz2
```

Inside RViz2:

1. Set **Fixed Frame** (top-left panel) to `base_footprint`
2. Click **Add** (bottom-left)
3. Choose **RobotModel** → Click **OK**
4. Click **Add** again → Choose **TF** → Click **OK**

**Expected result:** The blue box chassis with two black wheels and one grey caster wheel should appear. Moving the sliders in the joint_state_publisher_gui window rotates the wheels in RViz2.

> **Troubleshooting:**
> - *Robot not visible:* Check that `robot_state_publisher` is running. In RobotModel panel, expand "Status" — it will show the exact error.
> - *Fixed Frame error (red text):* Make sure Fixed Frame is set to `base_footprint`, not `map` or `odom`.
> - *No TF axes:* Confirm `joint_state_publisher_gui` is running. Check `ros2 topic echo /joint_states`.

---

## 5. TF — Transform Frames

### Concept
**TF2** (Transform Library) tracks the position and orientation of every named frame in the robot. Every link in your URDF becomes a TF frame. TF allows any node to ask: "Where is `left_wheel` relative to `base_link` right now?" This is fundamental for perception, planning, and control. There are two types: **static transforms** (fixed, like `caster_joint`) published once on `/tf_static`, and **dynamic transforms** (moving joints) published continuously on `/tf`.

---

### 5.1 Inspect Running TF Frames

With `robot_state_publisher` and `joint_state_publisher_gui` still running (from Section 4):

```bash
# List all active TF frames
ros2 run tf2_ros tf2_echo base_footprint base_link
```

**Expected output:**
```
At time 0.0
- Translation: [0.000, 0.000, 0.090]
- Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
```

This says `base_link` is 9 cm above `base_footprint` — matching our URDF `origin xyz="0.0 0.0 0.09"`.

```bash
# Check a dynamic transform (move sliders to see it change)
ros2 run tf2_ros tf2_echo base_link left_wheel
```

### 5.2 Publish a Static Transform Manually

Sometimes you need to define a frame not in the URDF — for example, a camera sensor mounted on the robot:

```bash
# Terminal: publish a static frame "camera_link" 10cm in front of base_link
ros2 run tf2_ros static_transform_publisher \
  0.20 0.0 0.05 0.0 0.0 0.0 \
  base_link camera_link
```

Format: `x y z roll pitch yaw parent_frame child_frame`

Verify in RViz2: the TF display will show a new `camera_link` axis 20 cm in front of the chassis.

### 5.3 Listen to /tf and /tf_static Topics

```bash
# See raw TF messages
ros2 topic echo /tf

# See static transforms (published once at startup)
ros2 topic echo /tf_static
```

> **When TF fails:**
> - *"Lookup would require extrapolation into the past"* — your clock is wrong or a node crashed. Restart robot_state_publisher.
> - *"Frame does not exist"* — check the frame name spelling exactly. ROS frame names are case-sensitive.
> - Fixed joints are published to `/tf_static`, not `/tf`. If you only see dynamic joints in `/tf`, this is correct.

---

## 6. Publisher and Subscriber Nodes

### Concept
In ROS 2, nodes communicate through **topics**. A **publisher** node sends data to a topic, and a **subscriber** node receives data from it. Topics are typed — both publisher and subscriber must use the same message type (e.g., `std_msgs/msg/String`). This decoupled design means publishers and subscribers don't need to know about each other — they just use the topic name as the meeting point.

---

### 6.1 Create the Talker (Publisher) Node

```bash
nano ~/ros2_ws/src/diff_drive_robot/diff_drive_robot/talker_node.py
```

```python
#!/usr/bin/env python3
"""
talker_node.py
Publishes a counter message to /chatter every 0.5 seconds.
Demonstrates: publisher, timer, node lifecycle.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TalkerNode(Node):
    """Simple publisher node that sends incrementing messages."""

    def __init__(self):
        # Initialize the node with the name 'talker'
        super().__init__('talker')

        # Create a publisher on topic '/chatter' with message type String
        # queue_size=10 means ROS buffers up to 10 undelivered messages
        self.publisher_ = self.create_publisher(String, '/chatter', 10)

        # Counter to track how many messages have been sent
        self.count = 0

        # Create a timer that calls self.timer_callback every 0.5 seconds
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('Talker node started. Publishing to /chatter')

    def timer_callback(self):
        """Called every 0.5 seconds. Builds and publishes a message."""
        msg = String()
        msg.data = f'Hello from diff_drive_robot! Count: {self.count}'

        self.publisher_.publish(msg)

        # Log to terminal every message
        self.get_logger().info(f'Publishing: "{msg.data}"')

        self.count += 1


def main(args=None):
    # Initialize ROS 2 communication
    rclpy.init(args=args)

    # Create node instance
    node = TalkerNode()

    # Keep the node alive, processing callbacks
    rclpy.spin(node)

    # Clean up after Ctrl+C
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 6.2 Create the Listener (Subscriber) Node

```bash
nano ~/ros2_ws/src/diff_drive_robot/diff_drive_robot/listener_node.py
```

```python
#!/usr/bin/env python3
"""
listener_node.py
Subscribes to /chatter and prints received messages.
Demonstrates: subscriber, callback function.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ListenerNode(Node):
    """Simple subscriber node that listens on /chatter."""

    def __init__(self):
        super().__init__('listener')

        # Create a subscription on topic '/chatter' with message type String
        # self.listener_callback is called whenever a new message arrives
        self.subscription = self.create_subscription(
            String,           # Message type
            '/chatter',       # Topic name
            self.listener_callback,  # Function to call on new message
            10                # Queue size
        )

        self.get_logger().info('Listener node started. Waiting for /chatter...')

    def listener_callback(self, msg):
        """Called automatically when a new message arrives on /chatter."""
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = ListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 6.3 Build and Run

```bash
cd ~/ros2_ws
colcon build --packages-select diff_drive_robot
source ~/.bashrc
```

Open **Terminal 1** — run the talker:

```bash
ros2 run diff_drive_robot talker_node
```

Open **Terminal 2** — run the listener:

```bash
ros2 run diff_drive_robot listener_node
```

**Expected result:** Terminal 2 prints: `I heard: "Hello from diff_drive_robot! Count: 0"` etc.

### 6.4 Inspect Topics from the Command Line

```bash
# List all active topics
ros2 topic list

# Show message type for a topic
ros2 topic info /chatter

# Print live messages to terminal
ros2 topic echo /chatter

# Check publishing rate (Hz)
ros2 topic hz /chatter

# Show active nodes
ros2 node list

# Show what a node publishes and subscribes to
ros2 node info /talker
```

> **Key understanding:** If you stop the talker, the listener doesn't crash — it just stops receiving messages. Topics are fully decoupled. This is one of ROS 2's most important design properties.

---

## 7. Odometry Node — Full Implementation

### Concept
**Odometry** is dead-reckoning navigation: estimating a robot's position and orientation by integrating wheel velocities over time. Given left and right wheel velocities, we calculate linear velocity `v` and angular velocity `ω`, then integrate to update `(x, y, θ)`. The result is published on the `/odom` topic as a `nav_msgs/Odometry` message, and also broadcast as a TF transform from `odom` → `base_footprint`. Odometry accumulates error over time and is always combined with sensor data (GPS, lidar) in real systems.

---

### 7.1 Odometry Mathematics

For a differential drive robot:

```
Wheel separation: L (distance between left and right wheels)
Wheel radius: r

Linear velocity:   v = r * (ω_right + ω_left) / 2
Angular velocity:  ω = r * (ω_right - ω_left) / L

Position update (Euler integration, timestep dt):
  x    += v * cos(θ) * dt
  y    += v * sin(θ) * dt
  θ    += ω * dt
```

### 7.2 Create the Odometry Node

```bash
nano ~/ros2_ws/src/diff_drive_robot/diff_drive_robot/odom_node.py
```

```python
#!/usr/bin/env python3
"""
odom_node.py
Full odometry node for a differential drive robot.

Subscribes to:  /joint_states  (sensor_msgs/JointState)
Publishes to:   /odom          (nav_msgs/Odometry)
Broadcasts TF:  odom → base_footprint

Robot parameters:
  Wheel radius:     0.08 m
  Wheel separation: 0.34 m  (center-to-center, 2 × 0.17)
"""

import rclpy
from rclpy.node import Node
import math

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformBroadcaster
from builtin_interfaces.msg import Time


class OdometryNode(Node):
    """
    Computes and publishes odometry from wheel joint states.
    """

    # ------------------------------------------------------------------ #
    # Robot physical parameters — change these if you change the URDF     #
    # ------------------------------------------------------------------ #
    WHEEL_RADIUS = 0.08       # meters — matches URDF cylinder radius
    WHEEL_SEPARATION = 0.34   # meters — 2 × 0.17 (joint origin y-offset)

    def __init__(self):
        super().__init__('odom_node')

        # ---- Robot pose state ---------------------------------------- #
        self.x = 0.0          # meters
        self.y = 0.0          # meters
        self.theta = 0.0      # radians

        # ---- Wheel angle tracking ------------------------------------ #
        self.prev_left_angle = None   # previous left wheel joint angle
        self.prev_right_angle = None  # previous right wheel joint angle

        # ---- Publisher: /odom ---------------------------------------- #
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

        # ---- TF Broadcaster ------------------------------------------ #
        # Publishes the transform odom → base_footprint
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---- Subscriber: /joint_states ------------------------------- #
        # joint_state_publisher or Gazebo will publish here
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.get_logger().info(
            'OdometryNode started.\n'
            f'  Wheel radius:     {self.WHEEL_RADIUS} m\n'
            f'  Wheel separation: {self.WHEEL_SEPARATION} m\n'
            'Subscribing to /joint_states, publishing /odom'
        )

    # ------------------------------------------------------------------ #
    def joint_state_callback(self, msg: JointState):
        """
        Called every time a new JointState message arrives.
        Computes odometry from wheel angle changes.
        """

        # --- Extract wheel angles from joint state message ------------ #
        # The message contains all joints; find left and right by name
        left_angle = None
        right_angle = None

        for i, name in enumerate(msg.name):
            if name == 'left_wheel_joint':
                left_angle = msg.position[i]
            elif name == 'right_wheel_joint':
                right_angle = msg.position[i]

        # Safety check: both wheels must be present
        if left_angle is None or right_angle is None:
            self.get_logger().warn(
                'Could not find left_wheel_joint or right_wheel_joint in /joint_states'
            )
            return

        # --- First message: initialize previous angles, skip update --- #
        if self.prev_left_angle is None:
            self.prev_left_angle = left_angle
            self.prev_right_angle = right_angle
            return

        # --- Compute angle deltas ------------------------------------- #
        d_left_angle = left_angle - self.prev_left_angle
        d_right_angle = right_angle - self.prev_right_angle

        # Save current angles for next iteration
        self.prev_left_angle = left_angle
        self.prev_right_angle = right_angle

        # --- Convert angle change → arc length traveled by each wheel - #
        d_left = self.WHEEL_RADIUS * d_left_angle
        d_right = self.WHEEL_RADIUS * d_right_angle

        # --- Differential drive kinematics ---------------------------- #
        d_center = (d_right + d_left) / 2.0          # linear displacement
        d_theta = (d_right - d_left) / self.WHEEL_SEPARATION  # rotation

        # --- Update pose (Euler integration) -------------------------- #
        self.x += d_center * math.cos(self.theta + d_theta / 2.0)
        self.y += d_center * math.sin(self.theta + d_theta / 2.0)
        self.theta += d_theta

        # Keep theta in [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # --- Get current time ----------------------------------------- #
        now = self.get_clock().now().to_msg()

        # --- Publish /odom -------------------------------------------- #
        self._publish_odometry(now)

        # --- Broadcast TF: odom → base_footprint ---------------------- #
        self._broadcast_tf(now)

    # ------------------------------------------------------------------ #
    def _publish_odometry(self, now):
        """Build and publish nav_msgs/Odometry message."""

        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Orientation as quaternion (rotation around Z axis)
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        # Velocity — not available from joint angles alone (set to 0)
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.angular.z = 0.0

        self.odom_publisher.publish(odom_msg)

    # ------------------------------------------------------------------ #
    def _broadcast_tf(self, now):
        """Broadcast the odom → base_footprint TF transform."""

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)

        self.tf_broadcaster.sendTransform(t)


# ---------------------------------------------------------------------- #
def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 7.3 Build and Run the Odometry Node

```bash
cd ~/ros2_ws
colcon build --packages-select diff_drive_robot
source ~/.bashrc
```

**Terminal 1** — robot_state_publisher:

```bash
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(cat ~/ros2_ws/src/diff_drive_robot/urdf/diff_robot.urdf)"
```

**Terminal 2** — joint_state_publisher_gui:

```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

**Terminal 3** — odometry node:

```bash
ros2 run diff_drive_robot odom_node
```

**Terminal 4** — watch odometry output:

```bash
ros2 topic echo /odom
```

Move the joint sliders → the `pose.pose.position` values in `/odom` should update.

### 7.4 Visualize Odometry in RViz2

**Terminal 5:**

```bash
rviz2
```

In RViz2:
1. **Fixed Frame** → `odom`
2. **Add** → `RobotModel`
3. **Add** → `TF`
4. **Add** → `Odometry` → set **Topic** to `/odom`

You will see the robot and a green arrow showing its estimated pose relative to the `odom` frame origin.

> **Odometry accuracy notes:**
> - Odometry is perfect when there is no wheel slip.
> - Error accumulates over time — the longer the robot drives, the more the estimated pose drifts from reality.
> - This is why real robots combine odometry with laser scan matching (SLAM) or GPS.
> - If the robot teleports in RViz: check that `odom_node` and `robot_state_publisher` are both running and that `/joint_states` is being published.

---

## 8. Launch Files

### Concept
**Launch files** let you start multiple nodes with a single command, passing parameters and remapping topic names. In ROS 2, launch files are Python scripts. This is far more practical than opening ten terminal windows — and it's the standard way to start any real robot. A well-written launch file also documents the intended startup sequence of a system.

---

### 8.1 Create the Display Launch File (RViz only)

```bash
nano ~/ros2_ws/src/diff_drive_robot/launch/display.launch.py
```

```python
#!/usr/bin/env python3
"""
display.launch.py
Starts: robot_state_publisher + joint_state_publisher_gui + rviz2
Use this launch file for URDF/TF visualization without Gazebo.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # ------------------------------------------------------------------ #
    # Read the URDF file content                                          #
    # ------------------------------------------------------------------ #
    urdf_file = os.path.expanduser(
        '~/ros2_ws/src/diff_drive_robot/urdf/diff_robot.urdf'
    )
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # ------------------------------------------------------------------ #
    # Node: robot_state_publisher                                         #
    # Reads URDF, publishes TF for all links                              #
    # ------------------------------------------------------------------ #
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': False}]
    )

    # ------------------------------------------------------------------ #
    # Node: joint_state_publisher_gui                                     #
    # Provides sliders to manually drive joint positions                  #
    # ------------------------------------------------------------------ #
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # ------------------------------------------------------------------ #
    # Node: rviz2                                                         #
    # ------------------------------------------------------------------ #
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.expanduser(
            '~/ros2_ws/src/diff_drive_robot/config/display.rviz'
        )]
        if os.path.exists(os.path.expanduser(
            '~/ros2_ws/src/diff_drive_robot/config/display.rviz'
        )) else []
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
```

### 8.2 Create the Full Launch File (with Odometry)

```bash
nano ~/ros2_ws/src/diff_drive_robot/launch/robot_with_odom.launch.py
```

```python
#!/usr/bin/env python3
"""
robot_with_odom.launch.py
Starts: robot_state_publisher + joint_state_publisher_gui + odom_node + rviz2
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    urdf_file = os.path.expanduser(
        '~/ros2_ws/src/diff_drive_robot/urdf/diff_robot.urdf'
    )
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': False}]
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),

        Node(
            package='diff_drive_robot',
            executable='odom_node',
            name='odom_node',
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'
        ),
    ])
```

### 8.3 Build and Run

```bash
cd ~/ros2_ws
colcon build --packages-select diff_drive_robot
source ~/.bashrc

# Run the display launch file
ros2 launch diff_drive_robot display.launch.py

# Or run the full launch with odometry
ros2 launch diff_drive_robot robot_with_odom.launch.py
```

**Expected result:** All configured nodes start in a single terminal. Ctrl+C stops all of them at once.

---

## 9. Gazebo Harmonic Integration

### Concept
**Gazebo Harmonic** is a physics simulator. It simulates sensors, actuators, and real-world physics (gravity, friction, collisions). Connecting ROS 2 to Gazebo requires two things: (1) adding Gazebo-specific plugins to the URDF so Gazebo knows how to simulate the robot, and (2) running the `ros_gz_bridge` to convert between Gazebo internal messages and ROS 2 topics. After this setup, your robot can be driven in simulation using the same ROS 2 topics you would use on real hardware.

---

### 9.1 Create the Gazebo-Compatible URDF (SDF plugins)

We need to add a differential drive plugin and IMU/sensor plugins to our URDF. Create a new file:

```bash
nano ~/ros2_ws/src/diff_drive_robot/urdf/diff_robot_gazebo.urdf
```

```xml
<?xml version="1.0"?>
<robot name="diff_robot"
  xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- ===================== MATERIALS ===================== -->
  <material name="blue">
    <color rgba="0.2 0.4 0.8 1.0"/>
  </material>
  <material name="black">
    <color rgba="0.1 0.1 0.1 1.0"/>
  </material>
  <material name="grey">
    <color rgba="0.5 0.5 0.5 1.0"/>
  </material>

  <!-- ===================== BASE LINK ===================== -->
  <link name="base_link">
    <visual>
      <geometry><box size="0.4 0.3 0.1"/></geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry><box size="0.4 0.3 0.1"/></geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0"
               iyy="0.02" iyz="0.0" izz="0.04"/>
    </inertial>
  </link>

  <!-- ===================== LEFT WHEEL ===================== -->
  <link name="left_wheel">
    <visual>
      <geometry><cylinder radius="0.08" length="0.04"/></geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry><cylinder radius="0.08" length="0.04"/></geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0"
               iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0.0 0.17 -0.05" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
    <dynamics damping="0.1" friction="0.1"/>
  </joint>

  <!-- ===================== RIGHT WHEEL ===================== -->
  <link name="right_wheel">
    <visual>
      <geometry><cylinder radius="0.08" length="0.04"/></geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry><cylinder radius="0.08" length="0.04"/></geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0"
               iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0.0 -0.17 -0.05" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
    <dynamics damping="0.1" friction="0.1"/>
  </joint>

  <!-- ===================== CASTER WHEEL ===================== -->
  <link name="caster_wheel">
    <visual>
      <geometry><sphere radius="0.04"/></geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <geometry><sphere radius="0.04"/></geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0"
               iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel"/>
    <origin xyz="0.15 0.0 -0.09" rpy="0 0 0"/>
  </joint>

  <!-- ===================== base_footprint ===================== -->
  <link name="base_footprint"/>
  <joint name="base_footprint_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0.0 0.0 0.09" rpy="0 0 0"/>
  </joint>

  <!-- =====================================================================
       GAZEBO PLUGINS
       These tell Gazebo how to simulate the robot's actuators and sensors.
       ===================================================================== -->

  <!--
    Plugin: DiffDrive
    Listens to /cmd_vel (geometry_msgs/Twist) and drives the two wheels.
    Also publishes /odom and broadcasts odom → base_footprint TF.
  -->
  <gazebo>
    <plugin filename="gz-sim-diff-drive-system"
            name="gz::sim::systems::DiffDrive">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.34</wheel_separation>
      <wheel_radius>0.08</wheel_radius>
      <odom_publish_frequency>20</odom_publish_frequency>
      <topic>cmd_vel</topic>
      <odom_topic>odom</odom_topic>
      <tf_topic>tf</tf_topic>
      <frame_id>odom</frame_id>
      <child_frame_id>base_footprint</child_frame_id>
      <max_linear_velocity>1.0</max_linear_velocity>
      <max_angular_velocity>2.0</max_angular_velocity>
    </plugin>
  </gazebo>

  <!--
    Plugin: JointStatePublisher
    Publishes /world/*/model/*/joint_state for all joints.
  -->
  <gazebo>
    <plugin filename="gz-sim-joint-state-publisher-system"
            name="gz::sim::systems::JointStatePublisher">
      <topic>joint_states</topic>
    </plugin>
  </gazebo>

</robot>
```

### 9.2 Create a Gazebo World File

```bash
nano ~/ros2_ws/src/diff_drive_robot/worlds/empty_world.sdf
```

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="empty_world">

    <!-- Physics settings -->
    <physics name="default_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Plugins needed in every Gazebo world -->
    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <!-- Lighting -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal><size>100 100</size></plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane><normal>0 0 1</normal><size>100 100</size></plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

### 9.3 Create the Gazebo Launch File

```bash
nano ~/ros2_ws/src/diff_drive_robot/launch/gazebo.launch.py
```

```python
#!/usr/bin/env python3
"""
gazebo.launch.py
Starts: Gazebo Harmonic + robot spawn + ROS-Gazebo bridge + robot_state_publisher + rviz2

ROS ↔ Gazebo communication flow:
  /cmd_vel (ROS)  →  [bridge]  →  Gazebo DiffDrive plugin  →  wheels spin
  Gazebo joints   →  [bridge]  →  /joint_states (ROS)      →  robot_state_publisher → TF
  Gazebo odom     →  [bridge]  →  /odom (ROS)              →  navigation stack
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():

    # ------------------------------------------------------------------ #
    # File paths                                                          #
    # ------------------------------------------------------------------ #
    urdf_file = os.path.expanduser(
        '~/ros2_ws/src/diff_drive_robot/urdf/diff_robot_gazebo.urdf'
    )
    world_file = os.path.expanduser(
        '~/ros2_ws/src/diff_drive_robot/worlds/empty_world.sdf'
    )

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # ------------------------------------------------------------------ #
    # 1. Start Gazebo Harmonic with our world                             #
    # ------------------------------------------------------------------ #
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', world_file, '-r'],
        output='screen'
    )

    # ------------------------------------------------------------------ #
    # 2. Spawn the robot into Gazebo after a short delay                  #
    #    (wait 3 seconds for Gazebo to fully start)                       #
    # ------------------------------------------------------------------ #
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gz', 'service',
                    '-s', '/world/empty_world/create',
                    '--reqtype', 'gz.msgs.EntityFactory',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '5000',
                    '--req',
                    f'sdf: "{robot_description.replace(chr(10), " ").replace(chr(34), chr(92)+chr(34))}"'
                    ' name: "diff_robot"'
                    ' pose { position { x: 0 y: 0 z: 0.1 } }'
                ],
                output='screen'
            )
        ]
    )

    # ------------------------------------------------------------------ #
    # 3. robot_state_publisher                                            #
    #    use_sim_time=True: syncs timestamps with Gazebo clock            #
    # ------------------------------------------------------------------ #
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # ------------------------------------------------------------------ #
    # 4. ROS ↔ Gazebo Bridge                                              #
    #    Maps Gazebo topics to ROS 2 topics and vice versa                #
    #                                                                     #
    #    Format per bridge:                                               #
    #    /topic@ros_msg_type[gz_msg_type  (Gazebo → ROS)                  #
    #    /topic@ros_msg_type]gz_msg_type  (ROS → Gazebo)                  #
    #    /topic@ros_msg_type@gz_msg_type  (bidirectional)                 #
    # ------------------------------------------------------------------ #
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            # Clock: Gazebo sim time → ROS /clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

            # cmd_vel: ROS teleop → Gazebo DiffDrive plugin
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',

            # Odometry: Gazebo → ROS
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',

            # TF: Gazebo → ROS (odom → base_footprint from DiffDrive plugin)
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',

            # Joint states: Gazebo → ROS (for robot_state_publisher)
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # ------------------------------------------------------------------ #
    # 5. RViz2                                                            #
    # ------------------------------------------------------------------ #
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,
        bridge_node,
        spawn_robot,
        rviz_node,
    ])
```

### 9.4 Build and Launch with Gazebo

```bash
cd ~/ros2_ws
colcon build --packages-select diff_drive_robot
source ~/.bashrc

ros2 launch diff_drive_robot gazebo.launch.py
```

**Expected result:**
- Gazebo Harmonic window opens with a flat ground plane
- The blue differential drive robot appears in the center
- RViz2 opens alongside Gazebo
- The `/cmd_vel`, `/odom`, and `/joint_states` topics are active

Verify the bridge is working:

```bash
# Check all active topics
ros2 topic list

# You should see:
# /clock
# /cmd_vel
# /joint_states
# /odom
# /tf
# /tf_static
# /robot_description
```

### 9.5 Understanding the ROS ↔ Gazebo Bridge

The bridge translates between two different message systems:

| ROS 2 Topic | ROS 2 Message Type | Gazebo Message | Direction |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | `gz.msgs.Clock` | Gazebo → ROS |
| `/cmd_vel` | `geometry_msgs/Twist` | `gz.msgs.Twist` | ROS → Gazebo |
| `/odom` | `nav_msgs/Odometry` | `gz.msgs.Odometry` | Gazebo → ROS |
| `/tf` | `tf2_msgs/TFMessage` | `gz.msgs.Pose_V` | Gazebo → ROS |
| `/joint_states` | `sensor_msgs/JointState` | `gz.msgs.Model` | Gazebo → ROS |

> **Troubleshooting Gazebo integration:**
> - *Gazebo opens but no robot appears:* Check the spawn command output for errors. Verify the URDF has no syntax errors with `check_urdf`.
> - *`/cmd_vel` is published but robot does not move:* Confirm the DiffDrive plugin topic matches — it must be `cmd_vel` (without leading slash) in the URDF plugin, and the bridge maps `/cmd_vel`.
> - *RViz2 shows "No transform from [base_link] to [odom]":* The bridge for `/tf` is not running or the DiffDrive plugin is not receiving commands. Check `ros2 topic hz /tf`.
> - *Robot falls through the ground:* The collision geometry in the URDF may be misaligned. Check inertial properties — very low or zero inertia values cause physics instability.

---

## 10. Keyboard Control and cmd_vel

### Concept
Robot motion commands in ROS 2 are sent on the `/cmd_vel` topic using `geometry_msgs/Twist` messages. A `Twist` message contains **linear velocity** (forward/back) and **angular velocity** (rotation). The `teleop_twist_keyboard` package reads your keyboard and publishes these messages. This is the standard way to manually drive any ROS robot — real or simulated — because any node that subscribes to `/cmd_vel` will respond identically.

---

### 10.1 Run Keyboard Control

With Gazebo running (from Section 9), open a new terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap /cmd_vel:=/cmd_vel
```

**Keyboard controls:**

| Key | Action |
|---|---|
| `i` | Move forward |
| `,` | Move backward |
| `j` | Rotate left |
| `l` | Rotate right |
| `u` | Forward + left arc |
| `o` | Forward + right arc |
| `k` | Stop |
| `q` / `z` | Increase / decrease speed |

### 10.2 Understand the /cmd_vel Message

Inspect the message while driving:

```bash
ros2 topic echo /cmd_vel
```

You will see output like:

```
linear:
  x: 0.5      # forward speed in m/s
  y: 0.0      # not used for diff-drive
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.3      # rotation in rad/s (positive = counterclockwise)
```

### 10.3 Send a Manual Command (No Keyboard)

You can publish a single velocity command directly from the terminal:

```bash
# Drive forward at 0.3 m/s for demonstration (publishes once)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Rotate in place (angular only)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"

# Stop the robot
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 10.4 Node, Topic, and Service Summary

By now you have used three core ROS 2 communication mechanisms:

**Nodes** are independent programs. Inspect them:

```bash
ros2 node list
ros2 node info /robot_state_publisher
```

**Topics** are asynchronous, one-to-many data streams:

```bash
ros2 topic list
ros2 topic info /cmd_vel
ros2 topic echo /odom
ros2 topic hz /joint_states
ros2 topic bw /tf          # bandwidth usage
```

**Services** are synchronous request/response calls (used less in driving, but fundamental in ROS):

```bash
# List available services
ros2 service list

# Call a service (example: get robot state)
ros2 service type /robot_state_publisher/get_type_hash_
```

**Parameters** are node configuration values:

```bash
# List parameters of a node
ros2 param list /robot_state_publisher

# Get a specific parameter
ros2 param get /robot_state_publisher robot_description
```

---

## 11. TF Tree Diagram

### Concept
The **TF tree** is a diagram showing every coordinate frame in the system and how they are connected. It tells you the parent-child relationships between all frames. Generating this diagram is a standard debugging step when something is spatially wrong — misaligned sensors, wrong robot pose, or navigation failures often trace back to a broken or wrong TF tree.

---

### 11.1 Generate the TF Tree as a PDF

With `robot_state_publisher` running (any configuration from above):

```bash
# Install pdf viewer if not available
sudo apt install -y evince

# Generate TF tree diagram (saves as /tmp/frames.pdf)
ros2 run tf2_tools view_frames
```

This runs for 5 seconds, collects all published TF frames, then creates:
- `/tmp/frames.pdf` — visual diagram of the tree
- `/tmp/frames.gv` — source file in Graphviz format

```bash
# Open the PDF
evince /tmp/frames.pdf
```

**Expected output for our robot:**

```
base_footprint
  └── base_link  [base_footprint_joint — fixed]
        ├── left_wheel   [left_wheel_joint — continuous]
        ├── right_wheel  [right_wheel_joint — continuous]
        └── caster_wheel [caster_joint — fixed]

(When odometry is running:)
odom
  └── base_footprint
        └── base_link
              └── ...
```

### 11.2 Live TF Monitoring

```bash
# Print current transform between two frames every 1 second
ros2 run tf2_ros tf2_echo odom base_link

# Watch transform at full rate
ros2 run tf2_ros tf2_echo base_link left_wheel

# Check if a transform is available (useful in scripts)
ros2 run tf2_ros tf2_echo base_footprint base_link 2>&1 | head -20
```

### 11.3 Check TF using rqt_tf_tree (GUI)

```bash
# Open rqt and navigate to: Plugins → Visualization → TF Tree
rqt
```

Or directly:

```bash
ros2 run rqt_tf_tree rqt_tf_tree
```

This shows a live, interactive version of the TF tree that updates as transforms change.

> **Common TF problems and solutions:**
>
> | Symptom | Cause | Fix |
> |---|---|---|
> | Frame `X` does not exist | Node publishing that frame crashed | Restart the responsible node |
> | Transform timeout | Publishing rate too low | Increase publish frequency or reduce `transform_tolerance` |
> | Tree disconnected | Missing joint in URDF or wrong parent/child | Check URDF with `check_urdf` |
> | `odom` frame missing | `odom_node` not running | Start `odom_node` or Gazebo DiffDrive plugin |

---

## 12. Troubleshooting Reference

This section collects the most common errors you will encounter and how to fix them.

---

### Build Errors

**Problem:** `colcon build` fails with `ModuleNotFoundError`

```bash
# Fix: install missing Python dependencies
sudo apt install -y python3-pip
pip3 install setuptools
```

**Problem:** `SetuptoolsDeprecationWarning` during build

```bash
# Fix: use specific version
pip3 install setuptools==58.2.0
```

---

### URDF Errors

**Problem:** Robot appears as white/grey flat box with no structure

This means the URDF has a parsing error but robot_state_publisher started anyway.

```bash
# Validate the URDF first
check_urdf ~/ros2_ws/src/diff_drive_robot/urdf/diff_robot.urdf

# Check robot_state_publisher logs
ros2 node info /robot_state_publisher
```

**Problem:** Wheels are in wrong position or orientation

Check `rpy` in joint `origin`. For cylinders aligned with Y-axis, use `rpy="-1.5708 0 0"` (−π/2 rotation around X).

---

### TF Errors

**Problem:** `Fixed Frame [map] does not exist`

Change Fixed Frame in RViz2 to `base_footprint` or `odom`.

**Problem:** `For frame [X]: No transform to fixed frame [odom]`

The `odom_node` or Gazebo DiffDrive plugin is not running. Check:

```bash
ros2 topic hz /tf
ros2 topic echo /tf
```

---

### Gazebo Errors

**Problem:** Gazebo opens but immediately closes

```bash
# Run Gazebo directly to see the error
gz sim ~/ros2_ws/src/diff_drive_robot/worlds/empty_world.sdf
```

**Problem:** Robot spawns but slides/vibrates abnormally

The inertia values in the URDF are too low. For Gazebo stability, no inertia value should be below ~1e-6. Use `ixx=iyy=izz=0.001` as a safe minimum.

**Problem:** Bridge node not connecting topics

```bash
# Verify bridge is running
ros2 node list | grep bridge

# Check bridge output for errors
ros2 node info /ros_gz_bridge
```

---

### Quick Diagnostics Checklist

When something does not work, run these commands in order:

```bash
# 1. Are the expected nodes running?
ros2 node list

# 2. Are the expected topics published?
ros2 topic list

# 3. Is data actually flowing on a topic?
ros2 topic hz /joint_states
ros2 topic hz /tf

# 4. Is the TF tree complete?
ros2 run tf2_tools view_frames && evince /tmp/frames.pdf

# 5. What is the robot_description parameter?
ros2 param get /robot_state_publisher robot_description

# 6. Are there any error messages in node logs?
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(cat ~/ros2_ws/src/diff_drive_robot/urdf/diff_robot.urdf)" \
  --log-level debug
```

---

## Summary — Complete File Structure

After completing this manual, your workspace has this structure:

```
~/ros2_ws/
└── src/
    └── diff_drive_robot/
        ├── package.xml
        ├── setup.py
        ├── diff_drive_robot/          ← Python nodes
        │   ├── __init__.py
        │   ├── talker_node.py
        │   ├── listener_node.py
        │   └── odom_node.py
        ├── urdf/
        │   ├── diff_robot.urdf        ← URDF for RViz
        │   └── diff_robot_gazebo.urdf ← URDF with Gazebo plugins
        ├── launch/
        │   ├── display.launch.py
        │   ├── robot_with_odom.launch.py
        │   └── gazebo.launch.py
        ├── config/
        └── worlds/
            └── empty_world.sdf
```

---

## Quick Reference Card

```bash
# ── Build ──────────────────────────────────────────────────────────────────
cd ~/ros2_ws && colcon build --packages-select diff_drive_robot && source ~/.bashrc

# ── URDF Check ────────────────────────────────────────────────────────────
check_urdf ~/ros2_ws/src/diff_drive_robot/urdf/diff_robot.urdf

# ── Launch Files ──────────────────────────────────────────────────────────
ros2 launch diff_drive_robot display.launch.py
ros2 launch diff_drive_robot robot_with_odom.launch.py
ros2 launch diff_drive_robot gazebo.launch.py

# ── Nodes ─────────────────────────────────────────────────────────────────
ros2 run diff_drive_robot talker_node
ros2 run diff_drive_robot listener_node
ros2 run diff_drive_robot odom_node
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# ── Topic Inspection ──────────────────────────────────────────────────────
ros2 topic list
ros2 topic echo /odom
ros2 topic echo /cmd_vel
ros2 topic hz /tf

# ── TF ────────────────────────────────────────────────────────────────────
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_tools view_frames && evince /tmp/frames.pdf
ros2 run rqt_tf_tree rqt_tf_tree

# ── Manual Drive Command ──────────────────────────────────────────────────
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}, angular: {z: 0.0}}"
```

---

*End of Lab Manual — ROS 2 Jazzy · Gazebo Harmonic · Ubuntu 24.04*
