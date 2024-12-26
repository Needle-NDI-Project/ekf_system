# **EKF System Package**

## Table of Contents

1. [Overview](#overview)
2. [Key Features](#key-features)
3. [Message and Service Definitions](#message-and-service-definitions)
4. [Dependencies](#dependencies)
5. [Installation](#installation)
6. [Usage Examples](#usage-examples)
    - [Publishing Messages](#publishing-messages)
    - [Subscribing to Messages](#subscribing-to-messages)
    - [Using Services](#using-services)
7. [Best Practices](#best-practices)
8. [Development Guide](#development-guide)
9. [Troubleshooting](#troubleshooting)

## Overview

The **EKF System** package implements an **Extended Kalman Filter (EKF)** for real-time **state estimation** of a robotic needle within the Needle-NDI-Project. By fusing data from multiple sources, including the NDI tracking system and various control commands, the EKF provides accurate and reliable estimations of the needle’s **pose** (position and orientation) and **velocity**. These estimations are crucial for precise control and feedback mechanisms in medical robotic procedures.

### Key Features

- **Robust State Estimation**: Utilizes an Extended Kalman Filter to fuse sensor and command data for accurate pose and velocity estimation.
- **Real-Time Performance**: Operates at high frequencies to support dynamic control and planning tasks.
- **Seamless ROS2 Integration**: Communicates effortlessly with other subsystems (`ndi_sys`, `needle_controllers`, `needle_planning`, `zaber_system`) using standardized ROS2 topics and services.
- **Modular Design**: Structured for easy maintenance, updates, and scalability.
- **Comprehensive Validation**: Incorporates data validation to ensure robustness against sensor noise, data loss, and anomalies.
- **Diagnostic and Logging Capabilities**: Provides detailed logs and diagnostics for monitoring filter performance and troubleshooting.

---

## Message and Service Definitions

### Directory Structure

```plaintext
ekf_system/
├── include/
│   └── ekf_system/
│       ├── data_validator.hpp
│       ├── ekf_node.hpp
│       └── extended_kalman_filter.hpp
├── srv/
│   ├── GetEKFState.srv
│   └── ResetFilter.srv
├── src/
│   ├── data_validator.cpp
│   ├── ekf_node.cpp
│   ├── ekf_node_main.cpp
│   └── extended_kalman_filter.cpp
├── launch/
│   ├── ekf_visualization.launch.py
│   └── ekf_with_ndi.launch.py
├── config/
│   └── ekf_config.yaml
├── CMakeLists.txt
└── package.xml
```

### Service Types

#### GetEKFState.srv

##### GetEKFState - Request

```plaintext
# GetEKFState.srv

# No request fields
---
```

##### GetEKFState - Response

```plaintext
# GetEKFState.srv

# Response
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
float64[] state_covariance  # Flattened state covariance matrix
```

#### ResetFilter.srv

##### ResetFilter - Request

```plaintext
# ResetFilter.srv

bool reset_to_default  # If true, reset to default initial state. If false, use provided state.
geometry_msgs/Pose initial_pose  # Optional initial pose (used if reset_to_default is false)
geometry_msgs/Twist initial_twist  # Optional initial twist (used if reset_to_default is false)
---
```

##### ResetFilter - Response

```plaintext
# ResetFilter.srv

bool success
string message
```

### Message Types

The `ekf_system` package leverages standard ROS2 messages and does **not** define custom message types. It utilizes messages such as `geometry_msgs/PoseWithCovarianceStamped` for state estimates and standard service messages for interaction.

---

## Dependencies

### Required ROS2 Packages

- **Core Dependencies**:
  - `rclcpp`: C++ client library for ROS2.
  - `geometry_msgs`: Common geometry messages.
  - `sensor_msgs`: Sensor data messages.
  - `tf2_ros`: Transformation library for ROS2.
  - `ndi_msgs`: Custom messages for NDI tracking systems.
  - `std_msgs`: Standard ROS2 message types.
  - `std_srvs`: Standard ROS2 service types.

- **Build Tools**:
  - `ament_cmake`: CMake macros for ROS2 packages.
  - `rosidl_default_generators`: Service and message generation.

- **External Libraries**:
  - `Eigen3`: C++ template library for linear algebra.
  - `eigen3_cmake_module`: CMake modules for Eigen3.

### Installation of Dependencies

Ensure all dependencies are installed using `rosdep`:

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

---

## Installation

### Prerequisites

- **Operating System**: Ubuntu 22.04 LTS or later.
- **ROS2 Distribution**: Humble Hawksbill or later.
- **Build Tools**: `colcon`, `cmake`, `make`, etc.

### Workspace Setup

1. **Clone the Repository**

   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone https://github.com/your-org/ekf_system.git
   ```

2. **Install Dependencies**

   ```bash
   cd ~/ros2_ws
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the Workspace**

   ```bash
   colcon build --packages-select ekf_system
   ```

4. **Source the Workspace**

   ```bash
   source install/setup.bash
   ```

   > **Note**: Ensure that the `ndi_msgs` package is built and sourced prior to building `ekf_system`.

---

## Usage Examples

### Publishing Messages

The `ekf_system` package primarily **subscribes** to sensor and command topics to perform state estimation. However, you can create publishers to simulate sensor data for testing.

#### Example: Publishing RigidArray Messages

```cpp
#include "rclcpp/rclcpp.hpp"
#include "ndi_msgs/msg/rigid_array.hpp"
#include "geometry_msgs/msg/pose.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("rigid_array_publisher");
  auto publisher = node->create_publisher<ndi_msgs::msg::RigidArray>("/ndi_tracker/state", 10);

  rclcpp::Rate loop_rate(50); // 50 Hz

  while (rclcpp::ok())
  {
    ndi_msgs::msg::RigidArray msg;
    msg.header.stamp = node->now();
    msg.header.frame_id = "world";

    // Example data for one tracker
    msg.ids.push_back(1);
    msg.frames.push_back("tracker_1");
    msg.inbound.push_back(true);

    geometry_msgs::msg::Pose pose;
    pose.position.x = 1.0;
    pose.position.y = 2.0;
    pose.position.z = 3.0;
    pose.orientation.w = 1.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;

    msg.poses.push_back(pose);

    publisher->publish(msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
```

### Subscribing to Messages

The `ekf_system` package subscribes to:

- `/ndi_tracker/state` (`ndi_msgs::msg::RigidArray`): Provides real-time tracking data.
- `/cmd_vel` (`geometry_msgs::msg::TwistStamped`): Receives velocity commands.

### Using Services

#### 1. **GetEKFState Service**

Retrieve the current state estimate and covariance.

```bash
ros2 service call /ekf_system/get_filter_state ekf_system/srv/GetEKFState
```

**Expected Response**:

```plaintext
pose:
  pose:
    position:
      x: 1.0
      y: 2.0
      z: 3.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  covariance: [ ... ]  # 36 float64 values
twist:
  twist:
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  covariance: [ ... ]  # 36 float64 values
state_covariance: [ ... ]  # 169 float64 values (13x13)
```

#### 2. **ResetFilter Service**

Reset the EKF to its initial state.

- **Reset to Default State**

  ```bash
  ros2 service call /ekf_system/reset_filter ekf_system/srv/ResetFilter "{reset_to_default: true}"
  ```

- **Reset to Provided State**

  ```bash
  ros2 service call /ekf_system/reset_filter ekf_system/srv/ResetFilter "reset_to_default: false
  initial_pose:
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  initial_twist:
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0"
  ```

**Expected Response**:

```plaintext
success: true
message: "Filter reset to default state"  # or "Filter reset to provided state"
```

---

## Best Practices

### Message Handling

1. **Always Set Header Timestamps**: Ensure that all published messages include accurate timestamps for synchronization.
2. **Maintain Array Consistency**: When publishing array-based messages like `RigidArray`, ensure that all arrays (`ids`, `frames`, `inbound`, `poses`) are of equal length.
3. **Validate Incoming Data**: Use the `DataValidator` to verify the integrity and plausibility of incoming sensor and command data before processing.
4. **Use Appropriate Frame IDs**: Consistently use the correct coordinate frames (`world_frame_id`) to avoid transformation errors.

### Configuration Management

1. **YAML Configuration**: Utilize YAML files (`ekf_config.yaml`) for parameter management to allow easy tuning and adjustments without recompiling.
2. **Logging Levels**: Adjust logging verbosity (`info`, `debug`, `warn`, `error`, `fatal`) based on the deployment environment to balance between information richness and performance.

### Performance Optimization

1. **Pre-allocate Vectors**: Where possible, pre-allocate memory for vectors to minimize dynamic memory allocation overhead.
2. **Efficient Matrix Operations**: Leverage `Eigen3` for optimized linear algebra computations, crucial for maintaining real-time performance.

### Error Handling

1. **Graceful Degradation**: Implement fallback mechanisms in case of sensor failures or data dropouts to maintain operational stability.
2. **Comprehensive Logging**: Log all critical events and errors to facilitate debugging and system monitoring.

---

## Development Guide

### Coding Standards

- **Language**: C++
- **Style Guide**: Adhere to ROS2 C++ style guidelines and general best practices for readability and maintainability.
- **Documentation**: Comment complex algorithms and provide clear explanations for design decisions.

---

## Troubleshooting

### Common Issues

1. **Service Call Failures**

   **Issue**:
   - Services like `/ekf_system/get_filter_state` or `/ekf_system/reset_filter` are not responding.

   **Solution**:
   - Verify that the `ekf_system` node is running.
   - Use `ros2 service list` to check if the services are available.
   - Inspect node logs for any errors related to service handlers.

2. **State Estimation Drift**

   **Issue**:
   - The EKF state estimates diverge over time, indicating drift.

   **Solution**:
   - Re-tune the process and measurement noise covariance matrices in `ekf_config.yaml`.
   - Ensure accurate calibration of sensors to minimize measurement noise.
   - Verify the correctness of the process and measurement models implemented in the EKF.

### Diagnostic Tools

- **ROS2 Topic Echo**

  ```bash
  ros2 topic echo /ekf_state_estimate
  ```

- **Service Calls**

  ```bash
  # Reset the EKF filter
  ros2 service call /ekf_system/reset_filter ekf_system/srv/ResetFilter "{reset_to_default: true}"

  # Get the current EKF state
  ros2 service call /ekf_system/get_filter_state ekf_system/srv/GetEKFState
  ```

- **Logging**

  Check the EKF node logs for error messages and warnings. Adjust the logging level in `ekf_config.yaml` as needed.

- **Visualization**

  Use RViz2 to visualize the estimated pose and compare it with actual sensor data for verification.
