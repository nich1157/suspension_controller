#pragma once

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/vector3.hpp"


#include <Eigen/Dense>
#include <vector>
#include <string>
#include <deque>
#include <mutex>

namespace suspension_controller
{

class SuspensionController : public controller_interface::ControllerInterface
{
public:
  SuspensionController();

  // Lifecycle
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  // MANDATORY INTERFACE CONFIGURATION OVERRIDES
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  

private:
  enum ControlMode { PID_Base, LQR, DISABLED };
  ControlMode control_mode_;

  // PID
  Eigen::Vector3d P_r_;
  Eigen::Vector3d e_prev_;
  Eigen::Vector3d e_int_;
  Eigen::Matrix3d K_P_, K_I_, K_D_;

  // LQR
  Eigen::Matrix3d Q_;
  Eigen::Matrix4d R_;
  Eigen::Matrix<double, 4, 3> K_;   
  Eigen::Matrix<double, 4, 3> K0_;   
  rclcpp::Time last_update_time_;


  // IMU filter
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::mutex imu_mutex_;
  double latest_pitch_, latest_roll_;
  std::deque<double> pitch_window_, roll_window_;
  const size_t filter_window_size_ = 5;
  double smoothed_pitch_ = 0.0;
  double smoothed_roll_ = 0.0;
  const double alpha_ = 0.1;

  bool imu_ready_ = false;

  rclcpp::Time last_time_;

  // Core control
  void runPIDBase(const rclcpp::Duration &period);
  void runLQRControl(const rclcpp::Duration& period);

  // Helpers
  Eigen::Vector3d getMeasuredChassisPose();
  Eigen::Vector4d getActuatorPositions();
  Eigen::Matrix<double, 3, 4> manualJacobianFK(const Eigen::Vector4d &d);
  std::tuple<double, double, double, double, double, double> suspensionFK(double d_LF, double d_LR, double d_RF, double d_RR);
  Eigen::Matrix<double, 4, 4> manualJacobianFK4DOF(double d_LF, double d_LR, double d_RF, double d_RR);
  Eigen::Matrix<double, 4, 3> solveContinuousLQR(const Eigen::Matrix3d& A, const Eigen::Matrix<double, 3, 4>& B, const Eigen::Matrix3d& Q, const Eigen::Matrix4d& R);

};

} // namespace suspension_controller
