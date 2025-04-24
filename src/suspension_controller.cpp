#include "suspension_controller/suspension_controller.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <mutex>


// TODO:
// - Add a saturation and minimum error required on delta_d
// - Tune PID
// - Other controllers
// - Delete writeout - switch with publisher
// - Test filter on real IMU


namespace suspension_controller {

    SuspensionController::SuspensionController() : controller_interface::ControllerInterface() {
        // Constructor
    }

    // Requried command and state
    controller_interface::InterfaceConfiguration SuspensionController::command_interface_configuration() const{
    
      // Command interfaces joints
      controller_interface::InterfaceConfiguration config;
      config.type = controller_interface::interface_configuration_type::INDIVIDUAL; // [LF, LR, RF, RR]
      config.names = {
        "left_front_linear/position",
        "left_rear_linear/position",
        "right_front_linear/position",
        "right_rear_linear/position"
      };
      return config;

    }

    controller_interface::InterfaceConfiguration SuspensionController::state_interface_configuration() const {
    
      // State interfaces joints
      controller_interface::InterfaceConfiguration config;
      config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
      config.names = {
        "left_front_linear/position",
        "left_rear_linear/position",
        "right_front_linear/position",
        "right_rear_linear/position"
      };
      return config;

    }



    // Func 1: init - parameters assignment
    controller_interface::CallbackReturn SuspensionController::on_init(){

        RCLCPP_INFO(rclcpp::get_logger("SuspensionController"), "on_init called");

        auto_declare<std::string>("control_mode", "PID_Base");
        auto_declare<std::vector<double>>("reference_pose", {0.0, 0.0, -0.377});  // [theta, roll, z] zero position
        auto_declare<std::vector<double>>("K_P", {1.0, 1.0, 0.0});
        auto_declare<std::vector<double>>("K_I", {0.0, 0.0, 0.0});
        auto_declare<std::vector<double>>("K_D", {0.0, 0.0, 0.0});


        // MISSING: declare what joints you are controlling from input
        // ALSO: can PID be overridden by yaml??

        return controller_interface::CallbackReturn::SUCCESS;
    }



    // Func 2: configure
    controller_interface::CallbackReturn SuspensionController::on_configure(const rclcpp_lifecycle::State &)
    {

      RCLCPP_INFO(rclcpp::get_logger("SuspensionController"), "on_configure called");
    
      try {
        // Control mode
        std::string mode = get_node()->get_parameter("control_mode").as_string();
        RCLCPP_INFO(get_node()->get_logger(), "control_mode = %s", mode.c_str());
    
        if (mode == "PID_Base")
          control_mode_ = PID_Base;
        else
          control_mode_ = DISABLED;
    
        // Reference pose
        auto ref = get_node()->get_parameter("reference_pose").as_double_array(); // check if reference has three entries
        if (ref.size() != 3) {
          RCLCPP_ERROR(get_node()->get_logger(), "reference_pose must have 3 elements.");
          return controller_interface::CallbackReturn::ERROR;
        }
        P_r_ = Eigen::Vector3d(ref[0], ref[1], ref[2]);
    
        // Gains
        auto kp = get_node()->get_parameter("K_P").as_double_array();
        auto ki = get_node()->get_parameter("K_I").as_double_array();
        auto kd = get_node()->get_parameter("K_D").as_double_array();
    
        if (kp.size() != 3 || ki.size() != 3 || kd.size() != 3) {
          RCLCPP_ERROR(get_node()->get_logger(), "PID gain vectors must have 3 elements each.");
          return controller_interface::CallbackReturn::ERROR;
        }
    
        K_P_ = Eigen::Matrix3d::Zero();
        K_I_ = Eigen::Matrix3d::Zero();
        K_D_ = Eigen::Matrix3d::Zero();
    
        for (int i = 0; i < 3; ++i) {
          K_P_(i, i) = kp[i];
          K_I_(i, i) = ki[i];
          K_D_(i, i) = kd[i];
        }
        
        // Reset error terms
        e_prev_ = Eigen::Vector3d::Zero();
        e_int_ = Eigen::Vector3d::Zero();
    
    
        return controller_interface::CallbackReturn::SUCCESS;

      } catch (const std::exception &e) { // error clause

        RCLCPP_ERROR(get_node()->get_logger(), "Exception in on_configure: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;

      }
    }
    


    // Func 3: activate
    controller_interface::CallbackReturn SuspensionController::on_activate(const rclcpp_lifecycle::State &state) {
      
      RCLCPP_INFO(get_node()->get_logger(), "on_activate() called.");

      if (ControllerInterface::on_activate(state) != CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(get_node()->get_logger(), "Base on_activate failed.");
        return CallbackReturn::ERROR;
      }

      // time stamp
      last_time_ = get_node()->now();
    
      // Connect to IMU topic
      imu_sub_ = get_node()->create_subscription<sensor_msgs::msg::Imu>(
        "/imu_sensor_broadcaster/imu", 10,
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
          std::scoped_lock lock(imu_mutex_);
      
          // Convert quaternion to RPY
          tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
          tf2::Matrix3x3 m(q);
          double roll, pitch, unused;
          m.getRPY(roll, pitch, unused);
      
          // Gz to FK frame adjustment (180° around X)
          pitch = -pitch; 
          roll  = -roll; // WHY ROLL NEGATIVE WHEN ROTATION AROUND X!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      
          // Median filtering - spike removal
          pitch_window_.push_back(pitch);
          roll_window_.push_back(roll);
          if (pitch_window_.size() > filter_window_size_) pitch_window_.pop_front();
          if (roll_window_.size() > filter_window_size_) roll_window_.pop_front();
      
          std::vector<double> sorted_pitch(pitch_window_.begin(), pitch_window_.end());
          std::vector<double> sorted_roll(roll_window_.begin(), roll_window_.end());
          std::sort(sorted_pitch.begin(), sorted_pitch.end());
          std::sort(sorted_roll.begin(), sorted_roll.end());
      
          double median_pitch = sorted_pitch[sorted_pitch.size() / 2];
          double median_roll  = sorted_roll[sorted_roll.size() / 2];
      
          // Exponential smoothing (low-pass)
          smoothed_pitch_ = alpha_ * median_pitch + (1.0 - alpha_) * smoothed_pitch_;
          smoothed_roll_  = alpha_ * median_roll  + (1.0 - alpha_) * smoothed_roll_;
      
          // Final output
          latest_pitch_ = smoothed_pitch_;
          latest_roll_  = smoothed_roll_;
          imu_ready_ = true;
        });
      
            
      return CallbackReturn::SUCCESS;
    }
    


    // Func 4: deactivate - empty
    controller_interface::CallbackReturn SuspensionController::on_deactivate(const rclcpp_lifecycle::State &) {

        return controller_interface::CallbackReturn::SUCCESS;
    
    }



    // Func 5: update - main control loop
    controller_interface::return_type SuspensionController::update( const rclcpp::Time &time, const rclcpp::Duration &period) {
        
      RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000, "SuspensionController::update() is running...");

        // Choose control method based on input
        switch (control_mode_) {

            case PID_Base:
                RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                "SuspensionController::Inside PID");
                runPIDBase(period);
                break;

            case DISABLED:

            default:
                RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                                    "SuspensionController is in DISABLED mode.");
                break;
            }


        return controller_interface::return_type::OK;

    }

    // MARK: -- Control methods --

    // 1: Func to PIDBase control
    void SuspensionController::runPIDBase(const rclcpp::Duration &period){
        const double dt = period.seconds();

        // Step 1: Get measured pose from sensors (IMU + FK-estimated z)
        Eigen::Vector3d P_mea = getMeasuredChassisPose();

        // Step 2: Get actuator positions
        Eigen::Vector4d d_current = getActuatorPositions(); // [LF, LR, RF, RR]

        // Step 3: Compute analytical Jacobian (only take θ, φ, z rows)
        Eigen::Matrix<double, 4, 4> J_full = manualJacobianFK4DOF(d_current[0], d_current[1], d_current[2], d_current[3]);
        Eigen::Matrix<double, 3, 4> J = J_full.topRows<3>();  // Extract θ, φ, z rows
        Eigen::Matrix<double, 4, 3> J_inv = J.completeOrthogonalDecomposition().pseudoInverse();

        // Step 4: PID error computation
        Eigen::Vector3d e = P_r_ - P_mea;
        RCLCPP_INFO(get_node()->get_logger(), "err p %.2f", e[0]);
        RCLCPP_INFO(get_node()->get_logger(), "err r %.2f", e[1]);
        RCLCPP_INFO(get_node()->get_logger(), "err z %.2f", e[2]);
        Eigen::Vector3d e_dot = (e - e_prev_) / dt;
        e_int_ += e * dt;
        e_prev_ = e;

        // Control signal
        Eigen::Vector3d u = K_P_ * e + K_I_ * e_int_ + K_D_ * e_dot;

        // Step 5: Convert to actuator delta commands
        Eigen::Vector4d delta_d = J_inv * u;
        
        const double delta_threshold = 0.002;  // 2 mm - minimum delta_d magnitude worth updating
        for (int i = 0; i < 4; ++i) {
          if (std::abs(delta_d[i]) < delta_threshold) {
            delta_d[i] = 0.0;
          }
        }


        // Check for NaN values in delta_d
        if (!delta_d.allFinite()){
            RCLCPP_ERROR(get_node()->get_logger(), "delta_d contains NaNs! Skipping update.");
            return;
        }

        // Print outs (Can delete)
        RCLCPP_INFO(get_node()->get_logger(), "pitch %.2f", P_mea[0]*180/M_PI);
        RCLCPP_INFO(get_node()->get_logger(), "roll %.2f", P_mea[1]*180/M_PI);
        RCLCPP_INFO(get_node()->get_logger(), "z %.2f", P_mea[2]);


        RCLCPP_INFO(get_node()->get_logger(), "delta_dLF %.2f", delta_d[0]);
        RCLCPP_INFO(get_node()->get_logger(), "delta_dLR %.2f", delta_d[1]);  
        RCLCPP_INFO(get_node()->get_logger(), "delta_dRF %.2f", delta_d[2]);
        RCLCPP_INFO(get_node()->get_logger(), "delta_dRR %.2f", delta_d[3]);
        Eigen::Vector4d d_new = d_current + delta_d;

        // Step 6: Clamp actuator positions to valid range
        for (int i = 0; i < 4; ++i)
            d_new[i] = std::clamp(d_new[i], 0.0, 0.175);

        // Step 7: send new command
        for (int i = 0; i < 4; ++i) {
              this->command_interfaces_[i].set_value(d_new[i]);
              RCLCPP_INFO(get_node()->get_logger(), "cmd %s: %.3f",
                          command_interfaces_[i].get_name().c_str(),
                          d_new[i]);
            }
            
    }



    // MARK: -- Helper methods --

    // Read actuator position
    Eigen::Vector4d SuspensionController::getActuatorPositions() {
        
        Eigen::Vector4d d; // init d-position vector

        // Error clause - 4 linear actuators are needed
        if (state_interfaces_.size() != 4){
            RCLCPP_ERROR(get_node()->get_logger(), "Expected 4 state interfaces, got %zu", state_interfaces_.size());
        }

        // Read encoder 
        for (size_t i = 0; i < 4; ++i)
        {
            d[i] = state_interfaces_[i].get_value();
        }

        return d;
    }    
    
    

    // Get measurement pose
    Eigen::Vector3d SuspensionController::getMeasuredChassisPose(){
      std::scoped_lock lock(imu_mutex_);

    
      Eigen::Vector4d d = getActuatorPositions(); // LF, LR, RF, RR
      double z = std::get<3>(suspensionFK(d[0], d[1], d[2], d[3])); // get z from FK model
      RCLCPP_INFO(get_node()->get_logger(), "pitch %.2f, roll %.2f, z %.2f", latest_pitch_, latest_roll_, z);
      return Eigen::Vector3d(latest_pitch_, latest_roll_, z);
    }
    
    
    // FK model
    std::tuple<double, double, double, double, double, double> SuspensionController::suspensionFK(double d_LF, double d_LR, double d_RF, double d_RR){
        // Parameters
        const double H = 0.428;  // rocker length [m]
        const double T = 0.721;  // track width [m]
        const double L = 0.377;  // nominal height [m]

        // Pitch
        double beta_R = std::atan2(d_RF - d_RR, H);
        double beta_L = std::atan2(d_LF - d_LR, H);
        double theta = 0.5 * (beta_R + beta_L);

        // Roll
        double h_R = std::cos(beta_R) * (L + d_RR + 0.5 * H * std::tan(beta_R));
        double h_L = std::cos(beta_L) * (L + d_LR + 0.5 * H * std::tan(beta_L));
        double phi = std::asin((h_R - h_L) / T);

        // Yaw
        double phi_R = std::atan2(d_LR - d_RR, T);
        double phi_F = std::atan2(d_LF - d_RF, T);
        double y_R = (L + (d_RR + d_LR) / 2.0) * std::sin(phi_R);
        double y_F = (L + (d_RF + d_LF) / 2.0) * std::sin(phi_F);
        double psi = std::atan2(y_F - y_R, H);

        // Z
        double z = -0.5 * (h_R + h_L);

        // X (forward offset)
        double x_R = -(h_R - L) * std::tan(beta_R);
        double x_L = -(h_L - L) * std::tan(beta_L);
        double x = 0.5 * (x_R + x_L);

        // Y (lateral offset)
        double y = -(z - L) * std::tan(phi);

        return {theta, phi, psi, z, x, y};
    }

    // FK Jacobian
    Eigen::Matrix<double, 4, 4> SuspensionController::manualJacobianFK4DOF(
        double d_LF, double d_LR, double d_RF, double d_RR)
    {
      const double H = 0.428;
      const double T = 0.721;
      const double L = 0.377;
    
      auto sec = [](double x) { return 1.0 / std::cos(x); };
    
      double beta_R = std::atan((d_RF - d_RR) / H);
      double beta_L = std::atan((d_LF - d_LR) / H);
    
      double d_betaR_d_RR = -1 / (H * (1 + std::pow((d_RF - d_RR) / H, 2)));
      double d_betaR_d_RF = -d_betaR_d_RR;
    
      double d_betaL_d_LR = -1 / (H * (1 + std::pow((d_LF - d_LR) / H, 2)));
      double d_betaL_d_LF = -d_betaL_d_LR;
    
      double h_R = std::cos(beta_R) * (L + d_RR + 0.5 * H * std::tan(beta_R));
      double h_L = std::cos(beta_L) * (L + d_LR + 0.5 * H * std::tan(beta_L));
    
      double dhR_d_RR = -std::sin(beta_R) * d_betaR_d_RR * (L + d_RR + 0.5 * H * std::tan(beta_R)) +
                        std::cos(beta_R) * (1 + 0.5 * H * std::pow(sec(beta_R), 2) * d_betaR_d_RR);
    
      double dhR_d_RF = -std::sin(beta_R) * d_betaR_d_RF * (L + d_RR + 0.5 * H * std::tan(beta_R)) +
                        std::cos(beta_R) * (0.5 * H * std::pow(sec(beta_R), 2) * d_betaR_d_RF);
    
      double dhL_d_LR = -std::sin(beta_L) * d_betaL_d_LR * (L + d_LR + 0.5 * H * std::tan(beta_L)) +
                        std::cos(beta_L) * (1 + 0.5 * H * std::pow(sec(beta_L), 2) * d_betaL_d_LR);
    
      double dhL_d_LF = -std::sin(beta_L) * d_betaL_d_LF * (L + d_LR + 0.5 * H * std::tan(beta_L)) +
                        std::cos(beta_L) * (0.5 * H * std::pow(sec(beta_L), 2) * d_betaL_d_LF);
    
      double common_phi_denom = std::sqrt(1 - std::pow((h_R - h_L) / T, 2));
    
      // Z
      double dz_d_RR = -0.5 * dhR_d_RR;
      double dz_d_RF = -0.5 * dhR_d_RF;
      double dz_d_LR = -0.5 * dhL_d_LR;
      double dz_d_LF = -0.5 * dhL_d_LF;
    
      // Theta
      double dtheta_d_RR = 0.5 * d_betaR_d_RR;
      double dtheta_d_RF = 0.5 * d_betaR_d_RF;
      double dtheta_d_LR = 0.5 * d_betaL_d_LR;
      double dtheta_d_LF = 0.5 * d_betaL_d_LF;
    
      // Phi
      double dphi_d_RR = (1 / (T * common_phi_denom)) * dhR_d_RR;
      double dphi_d_RF = (1 / (T * common_phi_denom)) * dhR_d_RF;
      double dphi_d_LR = -(1 / (T * common_phi_denom)) * dhL_d_LR;
      double dphi_d_LF = -(1 / (T * common_phi_denom)) * dhL_d_LF;
    
      // Yaw
      double phi_R = std::atan2(d_LR - d_RR, T);
      double phi_F = std::atan2(d_LF - d_RF, T);
    
      double y_R = (L + 0.5 * (d_RR + d_LR)) * std::sin(phi_R);
      double y_F = (L + 0.5 * (d_RF + d_LF)) * std::sin(phi_F);
    
      double dphiR_d_RR = -1 / (T * (1 + std::pow((d_LR - d_RR) / T, 2)));
      double dphiR_d_LR = -dphiR_d_RR;
    
      double dphiF_d_RF = -1 / (T * (1 + std::pow((d_LF - d_RF) / T, 2)));
      double dphiF_d_LF = -dphiF_d_RF;
    
      double dyR_d_RR = 0.5 * std::sin(phi_R) + (L + 0.5 * (d_RR + d_LR)) * std::cos(phi_R) * dphiR_d_RR;
      double dyR_d_LR = 0.5 * std::sin(phi_R) + (L + 0.5 * (d_RR + d_LR)) * std::cos(phi_R) * dphiR_d_LR;
    
      double dyF_d_RF = 0.5 * std::sin(phi_F) + (L + 0.5 * (d_RF + d_LF)) * std::cos(phi_F) * dphiF_d_RF;
      double dyF_d_LF = 0.5 * std::sin(phi_F) + (L + 0.5 * (d_RF + d_LF)) * std::cos(phi_F) * dphiF_d_LF;
    
      double common_psi_denom = 1 + std::pow((y_F - y_R) / H, 2);
      double dpsi_d_RR = (1 / H) * (-dyR_d_RR) / common_psi_denom;
      double dpsi_d_LR = (1 / H) * (-dyR_d_LR) / common_psi_denom;
      double dpsi_d_RF = (1 / H) * (dyF_d_RF) / common_psi_denom;
      double dpsi_d_LF = (1 / H) * (dyF_d_LF) / common_psi_denom;
    
      Eigen::Matrix<double, 4, 4> J;
      J << dtheta_d_LF, dtheta_d_LR, dtheta_d_RF, dtheta_d_RR,
           dphi_d_LF,   dphi_d_LR,   dphi_d_RF,   dphi_d_RR,
           dz_d_LF,     dz_d_LR,     dz_d_RF,     dz_d_RR,
           dpsi_d_LF,   dpsi_d_LR,   dpsi_d_RF,   dpsi_d_RR;
    
      return J;
    }
    



} // namespace suspension_controller

PLUGINLIB_EXPORT_CLASS(suspension_controller::SuspensionController, controller_interface::ControllerInterface)
