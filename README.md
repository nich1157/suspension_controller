# Suspension Controller

A ROS 2 controller plugin for managing the suspension of legged or wheeled robots. The controller exposes individual actuator interfaces, subscribes to IMU data, and supports multiple control strategies (PID, LQR, IK-based, and terrain-aware variants) to keep the chassis stable.

## Features
- Lifecycle-aware controller built on `controller_interface`
- Configurable control modes including PID, LQR, inverse kinematics, and terrain-compensated options
- IMU-based state estimation with median filtering for noise suppression
- Publishable diagnostics for terrain estimation and Kalman filter state
- Ships as a plugin with `pluginlib` metadata

## Getting Started
### Prerequisites
- ROS 2 (Humble or newer recommended)
- `colcon` build tool
- Required dependencies: `rclcpp`, `controller_interface`, `hardware_interface`, `pluginlib`, `Eigen3`, `sensor_msgs`, and `tf2`

### Build
```bash
# From your workspace root
colcon build --packages-select suspension_controller
source install/setup.bash
```

### Usage
1. Add the plugin description to your controller manager configuration using the exported plugin class `suspension_controller/SuspensionController`.
2. Configure parameters such as `control_mode`, `reference_pose`, and PID gains in your controller YAML file.
3. Ensure an IMU publisher (e.g., `/imu_sensor_broadcaster/imu`) is available for the controller to subscribe to during activation.

### Parameters (examples)
- `control_mode`: one of `PID_Base`, `LQR`, `IK_Based`, `PID_w_Terrain`, `PID_Complete`, or `DISABLED`.
- `reference_pose`: `[theta, roll, z, yaw]` reference in radians/meters (default `[0.0, 0.0, -0.377, 0.0]`).
- `K_P`, `K_I`, `K_D`: 4-element vectors defining diagonal PID gains for each actuator.

### Testing
If you have simulation or hardware integration tests, run them from your workspace after building:
```bash
colcon test --packages-select suspension_controller
```

## Contributing
1. Fork the repository and create a feature branch.
2. Format and lint your code according to your ROS 2 tooling preferences.
3. Open a pull request describing your changes and testing steps.

## License
This project is licensed under the Apache License 2.0. See the [LICENSE](LICENSE) file for details.
