# ROS Rover Workspace

A modular ROS 2 workspace designed for robotic rover and arm control systems. This repository contains packages for motor control, computer vision, odometry, and custom message definitions.

## 📦 Project Structure

This workspace is organized into the following ROS 2 packages:

- **`rover_msgs`**: Custom message definitions (`ControllerMsg`, `EncoderMsg`, `RobotKolMsg`) used across the system.
- **`motor_kontrol`**: Core logic for robot movement and kinematics. Supports Bluetooth, wired keyboard, and remote control interfaces. Includes inverse kinematics calculations for robotic arms.
- **`cv`**: Computer vision packages including camera publishing and ArUco marker detection.
- **`odometry`**: Logic for calculating the robot's position and orientation (odom). Includes RViz configuration for visualization.
- **`costmap`**: Python-based costmap management for navigation.
- **`costmap_converter`**: C++ node for converting grid maps to costmaps.
- **`kumanda_kontrol`**: Remote control logic.

## 🛠 Prerequisites

Before setting up the workspace, ensure you have the following installed:

- **OS**: Ubuntu 20.04 / 22.04 (Recommended)
- **ROS 2**: Humble, Iron, or Jazzy Desktop Full
- **Build Tool**: `colcon`
- **Dependencies**: Python 3, C++ Compiler, `rosdep`, `vcs`

## 🚀 Installation & Setup

### 1. Create a ROS 2 Workspace

```bash
mkdir -p ~/rover_ws/src
cd ~/rover_ws/src
```

### 2. Clone the Repository

```bash
git clone https://github.com/HU-Rover/Manuel.git
```

### 3. Install Dependencies

Navigate to the workspace root and install dependencies using `rosdep`:

```bash
cd ~/rover_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Build the Workspace

Use `colcon` to build all packages:

```bash
colcon build
```

### 5. Source the Environment

Always source the setup script in your terminal before running nodes:

```bash
source install/setup.bash
```

## ▶️ Usage

### Launching the System

You can launch specific modules using the provided launch files:

**Launch Motor Control:**
```bash
ros2 launch motor_kontrol motor_kontrol_launch.py
```

**Launch Odometry & RViz:**
```bash
ros2 launch odometry odom_launch.py
```

### Running Specific Nodes

Run individual Python nodes directly:

```bash
# Start the camera publisher
ros2 run cv camera_pub

# Start the ArUco detector
ros2 run cv aruco

# Start the robot arm kinematics node
ros2 run motor_kontrol kinematiksiz_robot_kol
```

### RViz Visualization

An RViz configuration file (`rviz_config.rviz`) is included in the `odometry` package. When launching `odom_launch.py`, RViz should open automatically with the configured view for visualizing odometry and sensor 
data.

## 📝 Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.
