#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <ruckig/ruckig.hpp>  // TOTG online: sinh q_ref(t), q̇_ref, q̈_ref có v,a giới hạn trước bộ fuzzy

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/parameter.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "interbotix_xs_msgs/msg/joint_group_command.hpp"
#include "interbotix_xs_msgs/srv/robot_info.hpp"
#include "interbotix_xs_msgs/srv/operating_modes.hpp"
#include "interbotix_xs_msgs/srv/torque_enable.hpp"

// ff_node: song song fuzzy_node nhưng feedforward = vận tốc + gia tốc từ profile
// (PWM_ff = Kv·q̇_profile + Ka·q̈_profile), KHÔNG bù trọng lực (không Pinocchio).
class FFNode : public rclcpp::Node {
 public:
  FFNode();

 private:
  // Số bậc tự do được profile = nhóm "arm" của rx150 (5 khớp).
  static constexpr size_t kProfileDoF = 5;

  std::string group_name_;
  double loop_rate_ = 100.0;
  double watchdog_timeout_ = 0.2;
  std::vector<double> Ke_;
  std::vector<double> Ked_;
  std::vector<double> Ku_;      // gain đầu ra (tunable) — tách riêng khỏi u_max (ngưỡng an toàn)
  std::vector<double> u_max_;
  std::vector<double> reference_;  // đích tĩnh (= reference_pose); q_ref profile tiến về đây

  // --- Tham số profile vận tốc/gia tốc (Ruckig = TOTG, tương đương MoveIt2) ---
  bool enable_profile_ = false;
  std::vector<double> max_velocities_;     // rad/s
  std::vector<double> max_accelerations_;  // rad/s^2 (= 5.0 của MoveIt2)
  double max_jerk_ = 0.0;  // 0 -> jerk lớn (~bang-bang accel, gần TOTG nhất); >0 -> mượt kiểu S-curve
  bool sync_mode_ = false;  // false = per-khớp độc lập (nhanh nhất); true = đồng bộ cùng thời gian

  // --- Feedforward vận tốc + gia tốc từ profile (thay cho bù trọng lực) ---
  std::vector<double> Kv_;  // gain FF vận tốc, PWM/(rad/s)
  std::vector<double> Ka_;  // gain FF gia tốc,  PWM/(rad/s²)

  // --- Trạng thái profile (Ruckig) ---
  std::optional<ruckig::Ruckig<kProfileDoF>> otg_;
  ruckig::InputParameter<kProfileDoF> otg_in_;
  ruckig::OutputParameter<kProfileDoF> otg_out_;
  bool profile_configured_ = false;  // đã set limit + target
  bool profile_seeded_ = false;      // đã seed current_position từ đo lần đầu
  std::array<double, kProfileDoF> q_ref_{};      // vị trí tham chiếu đã profile
  std::array<double, kProfileDoF> qdot_ref_{};   // vận tốc tham chiếu đã profile
  std::array<double, kProfileDoF> qddot_ref_{};  // gia tốc tham chiếu đã profile (FF)

  rclcpp::Client<interbotix_xs_msgs::srv::RobotInfo>::SharedPtr cli_info_;
  rclcpp::Client<interbotix_xs_msgs::srv::OperatingModes>::SharedPtr cli_modes_;
  rclcpp::Client<interbotix_xs_msgs::srv::TorqueEnable>::SharedPtr cli_torque_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
  // Nhận vị trí setpoint runtime (5 khớp) để tune mà không relaunch/sửa yaml.
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_setpoint_;
  // Giữ callback handle sống để live-tuning Ke/Ked/Ku/u_max/Kv/Ka qua `ros2 param set`.
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  rclcpp::Publisher<interbotix_xs_msgs::msg::JointGroupCommand>::SharedPtr pub_cmd_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_err_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_edot_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_eff_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_ref_;  // debug: profile reference
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_ff_;   // debug: feedforward vel/acc
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::string> joint_names_;
  std::unordered_map<std::string, size_t> js_index_;
  sensor_msgs::msg::JointState last_js_;
  rclcpp::Time last_js_stamp_;
  bool have_js_ = false;
  bool ready_ = false;

  void configureProfile();  // set limit/target cho Ruckig (gọi 1 lần trong onRobotInfo)
  void onJointStates(const sensor_msgs::msg::JointState::SharedPtr msg);
  void onSetpoint(const std_msgs::msg::Float64MultiArray::SharedPtr msg);  // cập nhật reference_ runtime
  rcl_interfaces::msg::SetParametersResult onParamChange(const std::vector<rclcpp::Parameter> & params);
  void onRobotInfo(rclcpp::Client<interbotix_xs_msgs::srv::RobotInfo>::SharedFuture future);
  void onTimer();
};
