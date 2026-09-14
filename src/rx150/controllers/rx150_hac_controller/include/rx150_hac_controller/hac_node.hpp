#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <ruckig/ruckig.hpp>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "interbotix_xs_msgs/msg/joint_group_command.hpp"
#include "interbotix_xs_msgs/srv/operating_modes.hpp"
#include "interbotix_xs_msgs/srv/robot_info.hpp"
#include "interbotix_xs_msgs/srv/torque_enable.hpp"
#include "rx150_hac_controller/gravity_comp.hpp"

class HacNode : public rclcpp::Node {
 public:
  HacNode();

 private:
  static constexpr size_t kProfileDoF = 5;

  std::string group_name_;
  double loop_rate_ = 100.0;
  double watchdog_timeout_ = 0.05;
  double debug_publish_rate_ = 50.0;
  double velocity_filter_tau_ = 0.006;
  uint64_t dbg_cnt_ = 0;
  uint32_t dbg_div_ = 2;

  std::vector<double> filtered_velocity_;
  rclcpp::Time last_filter_time_;
  bool filter_initialized_ = false;

  std::vector<float> last_cmd_pwm_;
  std::vector<float> last_grav_pwm_;
  rclcpp::Time watchdog_trip_time_;
  bool in_watchdog_ramp_ = false;

  std::vector<double> error_limit_;
  std::vector<double> error_dot_limit_;
  std::vector<double> u_max_;
  std::vector<double> joint_gain_scale_;  // nhân vào đầu ra mặt HAC, riêng từng khớp (≈ Ku của fuzzy)
  std::vector<double> reference_;

  double a_ = 0.3;
  double b_ = 12.0;
  double c_ = 1200.0;

  bool enable_profile_ = false;
  std::vector<double> max_velocities_;
  std::vector<double> max_accelerations_;
  double max_jerk_ = 0.0;
  bool sync_mode_ = false;

  std::unique_ptr<GravityCompensation> grav_comp_;
  bool enable_gravity_comp_ = true;
  std::vector<double> Gff_;
  std::vector<double> gravity_sign_;
  std::string gravity_model_source_ = "pinocchio";  // "pinocchio" | "fitted"
  std::vector<double> fitted_gravity_coeffs_;        // 12 hệ số, xem gravity_comp.hpp

  std::vector<double> friction_coulomb_;
  std::vector<double> friction_viscous_;
  double friction_eps_ = 0.05;

  std::optional<ruckig::Ruckig<kProfileDoF>> otg_;
  ruckig::InputParameter<kProfileDoF> otg_in_;
  ruckig::OutputParameter<kProfileDoF> otg_out_;
  bool profile_configured_ = false;
  bool profile_seeded_ = false;
  std::array<double, kProfileDoF> q_ref_{};
  std::array<double, kProfileDoF> qdot_ref_{};

  rclcpp::Client<interbotix_xs_msgs::srv::RobotInfo>::SharedPtr cli_info_;
  rclcpp::Client<interbotix_xs_msgs::srv::OperatingModes>::SharedPtr cli_modes_;
  rclcpp::Client<interbotix_xs_msgs::srv::TorqueEnable>::SharedPtr cli_torque_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_setpoint_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  rclcpp::Publisher<interbotix_xs_msgs::msg::JointGroupCommand>::SharedPtr pub_cmd_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_err_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_edot_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_eff_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_ref_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_grav_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_grav_torque_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_fric_;
  // Đo chi phí tính (A/B với fuzzy): gộp theo cửa sổ debug, phát ở hac/timing.
  // Giữ ĐỐI XỨNG với rx150_fuzzy_node.hpp, nếu không số so sánh vô nghĩa.
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_timing_;
  double tm_law_sum_ns_ = 0.0;
  double tm_law_max_ns_ = 0.0;
  double tm_cyc_sum_ns_ = 0.0;
  double tm_cyc_max_ns_ = 0.0;
  uint32_t tm_n_ = 0;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::string> joint_names_;
  std::unordered_map<std::string, size_t> js_index_;
  sensor_msgs::msg::JointState last_js_;
  rclcpp::Time last_js_stamp_;
  bool have_js_ = false;
  bool ready_ = false;

  void configureProfile();
  void onJointStates(const sensor_msgs::msg::JointState::SharedPtr msg);
  void onSetpoint(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  rcl_interfaces::msg::SetParametersResult onParamChange(
      const std::vector<rclcpp::Parameter> & params);
  void onRobotInfo(
      rclcpp::Client<interbotix_xs_msgs::srv::RobotInfo>::SharedFuture future);
  void onTimer();
};