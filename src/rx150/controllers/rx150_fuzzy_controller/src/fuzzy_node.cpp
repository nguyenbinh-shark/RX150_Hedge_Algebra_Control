#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "interbotix_xs_msgs/msg/joint_group_command.hpp"
#include "interbotix_xs_msgs/srv/robot_info.hpp"
#include "interbotix_xs_msgs/srv/operating_modes.hpp"
#include "interbotix_xs_msgs/srv/torque_enable.hpp"
#include "rx150_fuzzy_controller/rx150_fuzzy_node.hpp"
#include "fuzzy_type1.h"   // extern "C": float fuzzy_type1_eval(float e, float ed)

FuzzyNode::FuzzyNode() : rclcpp::Node("fuzzy_node") {
  // 1. Parameters
  group_name_ = this->declare_parameter<std::string>("group_name", "arm");
  loop_rate_ = this->declare_parameter<double>("loop_rate", 100.0);
  watchdog_timeout_ = this->declare_parameter<double>("watchdog_timeout", 0.2);
  Ke_ = this->declare_parameter<std::vector<double>>("Ke", std::vector<double>());
  Ked_ = this->declare_parameter<std::vector<double>>("Ked", std::vector<double>());
  u_max_ = this->declare_parameter<std::vector<double>>("u_max", std::vector<double>());
  Ku_ = this->declare_parameter<std::vector<double>>("Ku", std::vector<double>());
  reference_ = this->declare_parameter<std::vector<double>>("reference_pose", std::vector<double>());

  // Profile vận tốc/gia tốc (Ruckig = TOTG) ráp trước bộ fuzzy.
  enable_profile_ = this->declare_parameter<bool>("enable_profile", true);
  max_velocities_ = this->declare_parameter<std::vector<double>>(
      "max_velocities", std::vector<double>{3.14, 3.14, 3.14, 3.14, 3.14});
  max_accelerations_ = this->declare_parameter<std::vector<double>>(
      "max_accelerations", std::vector<double>{5.0, 5.0, 5.0, 5.0, 5.0});
  max_jerk_ = this->declare_parameter<double>("max_jerk", 0.0);
  sync_mode_ = this->declare_parameter<bool>("sync_mode", false);

  // --- Gravity Compensation ---
  enable_gravity_comp_ = this->declare_parameter<bool>("enable_gravity_comp", true);
  Gff_ = this->declare_parameter<std::vector<double>>("Gff", std::vector<double>{885.0, 632.0, 632.0, 885.0, 885.0});
  gravity_sign_ = this->declare_parameter<std::vector<double>>("gravity_sign", std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0});
  this->declare_parameter<std::string>("robot_description", "");

  // 2. Clients (relative names -> resolved under launch namespace "rx150")
  cli_info_ = this->create_client<interbotix_xs_msgs::srv::RobotInfo>("get_robot_info");
  cli_modes_ = this->create_client<interbotix_xs_msgs::srv::OperatingModes>("set_operating_modes");
  cli_torque_ = this->create_client<interbotix_xs_msgs::srv::TorqueEnable>("torque_enable");

  // 3. Wait for get_robot_info service (no spin)
  if (!cli_info_->wait_for_service(std::chrono::seconds(10))) {
    RCLCPP_ERROR(this->get_logger(), "get_robot_info service not available after 10s");
    return;
  }

  // 4. Async (non-blocking) bootstrap: query robot info, finish init in callback
  auto req = std::make_shared<interbotix_xs_msgs::srv::RobotInfo::Request>();
  req->cmd_type = "group";
  req->name = group_name_;
  cli_info_->async_send_request(
      req, std::bind(&FuzzyNode::onRobotInfo, this, std::placeholders::_1));

  // 5. Subscription + publishers (relative topics)
  sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS(),
      std::bind(&FuzzyNode::onJointStates, this, std::placeholders::_1));

  // Setpoint runtime (5 khớp, rad) — ghi đè reference_ để tune mà không sửa yaml/relaunch.
  // Topic tương đối -> /rx150/fuzzy/setpoint. QoS reliable để không mất lệnh.
  sub_setpoint_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "fuzzy/setpoint", 10,
      std::bind(&FuzzyNode::onSetpoint, this, std::placeholders::_1));
  pub_err_ = this->create_publisher<sensor_msgs::msg::JointState>("fuzzy/error", rclcpp::SensorDataQoS());
  pub_edot_ = this->create_publisher<sensor_msgs::msg::JointState>("fuzzy/edot", rclcpp::SensorDataQoS());
  pub_eff_ = this->create_publisher<sensor_msgs::msg::JointState>("fuzzy/effort", rclcpp::SensorDataQoS());
  pub_ref_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "fuzzy/reference", rclcpp::SensorDataQoS());  // profile q_ref/qdot_ref để plot
  pub_grav_ = this->create_publisher<sensor_msgs::msg::JointState>("fuzzy/gravity", rclcpp::SensorDataQoS());
  pub_cmd_ = this->create_publisher<interbotix_xs_msgs::msg::JointGroupCommand>("commands/joint_group", 10);

  // 6. Control timer
  timer_ = this->create_wall_timer(
      std::chrono::microseconds(static_cast<int64_t>(1e6 / loop_rate_)),
      std::bind(&FuzzyNode::onTimer, this));

  // 6b. Live-tuning: ros2 param set Ke/Ked/Ku/u_max cập nhật thẳng vào vòng điều khiển.
  // Đăng ký SAU khi các declare_parameter ở mục 1 hoàn tất (chỉ set runtime mới trigger).
  param_cb_handle_ = this->add_on_set_parameters_callback(
      std::bind(&FuzzyNode::onParamChange, this, std::placeholders::_1));

  // 7. Best-effort shutdown: zero PWM + torque off
  rclcpp::on_shutdown([this]() {
    try {
      interbotix_xs_msgs::msg::JointGroupCommand zero_msg;
      zero_msg.name = group_name_;
      zero_msg.cmd.assign(joint_names_.size(), 0.0f);
      pub_cmd_->publish(zero_msg);

      auto tq_req = std::make_shared<interbotix_xs_msgs::srv::TorqueEnable::Request>();
      tq_req->cmd_type = "group";
      tq_req->name = group_name_;
      tq_req->enable = false;
      cli_torque_->async_send_request(tq_req);

      RCLCPP_INFO(this->get_logger(), "shutdown: zero + torque off");
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "shutdown handler error: %s", e.what());
    }
  });
}

void FuzzyNode::onRobotInfo(
    rclcpp::Client<interbotix_xs_msgs::srv::RobotInfo>::SharedFuture future) {
  auto resp = future.get();
  joint_names_ = resp->joint_names;

  // Map each group joint name -> its index in the full published JointState
  // (which also carries the gripper finger joint(s)).
  js_index_.clear();
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    js_index_[joint_names_[i]] = static_cast<size_t>(resp->joint_state_indices.at(i));
  }

  // Khởi tạo GravityCompensation
  std::string urdf_xml = this->get_parameter("robot_description").as_string();
  if (!urdf_xml.empty()) {
    grav_comp_ = std::make_unique<GravityCompensation>(urdf_xml, joint_names_);
    if (!grav_comp_->isValid()) {
      RCLCPP_WARN(this->get_logger(), "GravityCompensation khởi tạo thất bại, sẽ tắt bù trọng lực.");
      enable_gravity_comp_ = false;
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "Không tìm thấy URDF (robot_description), tắt bù trọng lực.");
    enable_gravity_comp_ = false;
  }

  // Fall back to sleep positions if no reference was configured.
  if (reference_.empty()) {
    reference_.assign(resp->joint_sleep_positions.begin(), resp->joint_sleep_positions.end());
  }

  // Sanity-check array sizes against the actual joint count.
  const size_t n = joint_names_.size();
  if (Ke_.size() != n || Ked_.size() != n || Ku_.size() != n || u_max_.size() != n ||
      reference_.size() != n) {
    RCLCPP_ERROR(
        this->get_logger(),
        "param size mismatch: joints=%zu, Ke=%zu, Ked=%zu, Ku=%zu, u_max=%zu, reference=%zu",
        n, Ke_.size(), Ked_.size(), Ku_.size(), u_max_.size(), reference_.size());
    return;  // ready_ stays false -> timer no-ops
  }

  // Cấu hình Ruckig (profile TOTG) nếu bật và số khớp đúng (= kProfileDoF).
  if (enable_profile_) {
    if (n != kProfileDoF) {
      RCLCPP_WARN(this->get_logger(),
          "enable_profile=true nhưng số khớp arm=%zu != %zu -> tắt profile", n, kProfileDoF);
      enable_profile_ = false;
    } else if (max_velocities_.size() != n || max_accelerations_.size() != n) {
      RCLCPP_WARN(this->get_logger(),
          "size max_velocities/max_accelerations != %zu -> tắt profile", n);
      enable_profile_ = false;
    } else {
      configureProfile();
    }
  }

  // Switch the group to PWM mode + torque on (fire-and-forget; empty responses).
  auto mode_req = std::make_shared<interbotix_xs_msgs::srv::OperatingModes::Request>();
  mode_req->cmd_type = "group";
  mode_req->name = group_name_;
  mode_req->mode = "pwm";
  mode_req->profile_type = "time";
  mode_req->profile_velocity = 0;
  mode_req->profile_acceleration = 0;
  cli_modes_->async_send_request(mode_req);

  auto tq_req = std::make_shared<interbotix_xs_msgs::srv::TorqueEnable::Request>();
  tq_req->cmd_type = "group";
  tq_req->name = group_name_;
  tq_req->enable = true;
  cli_torque_->async_send_request(tq_req);

  ready_ = true;
  RCLCPP_INFO(this->get_logger(), "fuzzy ready: %zu joints, pwm mode%s",
      n, enable_profile_ ? ", profile TOTG on" : "");
}

void FuzzyNode::configureProfile() {
  const double dt = 1.0 / loop_rate_;
  otg_.emplace(dt);
  for (size_t i = 0; i < kProfileDoF; ++i) {
    otg_in_.target_position[i] = reference_[i];
    otg_in_.target_velocity[i] = 0.0;
    otg_in_.target_acceleration[i] = 0.0;
    otg_in_.max_velocity[i] = max_velocities_[i];
    otg_in_.max_acceleration[i] = max_accelerations_[i];
    // max_jerk: 0 -> giá trị lớn (≈bang-bang accel, gần TOTG nhất); >0 -> jerk hữu hạn (mượt).
    otg_in_.max_jerk[i] = (max_jerk_ > 0.0) ? max_jerk_ : 100.0 * max_accelerations_[i];
  }
  otg_in_.synchronization =
      sync_mode_ ? ruckig::Synchronization::Time : ruckig::Synchronization::None;
  profile_configured_ = true;
  RCLCPP_INFO(this->get_logger(),
      "profile TOTG(Ruckig): dt=%.3fs v=%.2f a=%.2f jerk=%.0f sync=%s",
      dt, max_velocities_[0], max_accelerations_[0],
      (max_jerk_ > 0.0) ? max_jerk_ : 100.0 * max_accelerations_[0],
      sync_mode_ ? "Time" : "None");
}

void FuzzyNode::onJointStates(const sensor_msgs::msg::JointState::SharedPtr msg) {
  last_js_ = *msg;
  last_js_stamp_ = this->now();
  have_js_ = true;
}

void FuzzyNode::onSetpoint(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  // Chấp nhận setpoint mới một khi đã biết số khớp (ready_).
  if (!ready_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "setpoint đến trước onRobotInfo -> bỏ qua");
    return;
  }
  if (msg->data.size() != reference_.size()) {
    RCLCPP_WARN(this->get_logger(),
        "setpoint sai số khớp: %zu != %zu -> bỏ qua", msg->data.size(), reference_.size());
    return;
  }
  reference_.assign(msg->data.begin(), msg->data.end());
  // Nếu profile đã cấu hình, cập nhật target để (khi wire) Ruckig tiến về setpoint mới.
  if (profile_configured_) {
    for (size_t i = 0; i < kProfileDoF; ++i) otg_in_.target_position[i] = reference_[i];
  }
  // Throttle 1 Hz: bridge stream 100 Hz, log mỗi msg sẽ flood console.
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "setpoint cập nhật: [%.3f %.3f %.3f %.3f %.3f]",
      reference_[0], reference_[1], reference_[2], reference_[3], reference_[4]);
}

rcl_interfaces::msg::SetParametersResult FuzzyNode::onParamChange(
    const std::vector<rclcpp::Parameter> & params) {
  // Live-tuning: chỉ Ke/Ked/Ku/u_max/Gff (mảng double, đúng số khớp) có hiệu lực online.
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & p : params) {
    const std::string & name = p.get_name();
    const bool is_gain = (name == "Ke" || name == "Ked" || name == "Ku" || name == "u_max" || name == "Gff");
    if (!is_gain) {
      continue;  // param khác (loop_rate, reference_pose, profile...) — nhận nhưng không tác dụng online
    }
    if (p.get_type() != rclcpp::PARAMETER_DOUBLE_ARRAY) {
      result.successful = false;
      result.reason = name + " cần mảng double";
      break;
    }
    std::vector<double> v = p.as_double_array();
    if (v.size() != Ke_.size()) {
      result.successful = false;
      result.reason = name + " size " + std::to_string(v.size()) + " != " +
                      std::to_string(Ke_.size());
      break;
    }
    if (name == "Ke") Ke_ = v;
    else if (name == "Ked") Ked_ = v;
    else if (name == "Ku") Ku_ = v;
    else if (name == "u_max") u_max_ = v;
    else if (name == "Gff") Gff_ = v;
    RCLCPP_INFO(this->get_logger(), "live param %s = [%g %g %g %g %g]",
        name.c_str(), v[0], v[1], v[2], v[3], v[4]);
  }
  return result;
}

void FuzzyNode::onTimer() {
  if (!ready_) {
    return;
  }
  if (!have_js_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "chưa có joint_states");
    return;
  }
  if ((this->now() - last_js_stamp_).seconds() > watchdog_timeout_) {
    RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000, "stale joint_states -> zero PWM");
    interbotix_xs_msgs::msg::JointGroupCommand zero_msg;
    zero_msg.name = group_name_;
    zero_msg.cmd.assign(joint_names_.size(), 0.0f);
    pub_cmd_->publish(zero_msg);
    return;
  }

  const size_t n = joint_names_.size();
  std::vector<float> cmd(n, 0.0f);

  // Debug JointStates (only the relevant field is populated per spec).
  sensor_msgs::msg::JointState err_msg;
  sensor_msgs::msg::JointState edot_msg;
  sensor_msgs::msg::JointState eff_msg;
  const rclcpp::Time stamp = this->now();
  err_msg.header.stamp = stamp;
  edot_msg.header.stamp = stamp;
  eff_msg.header.stamp = stamp;
  err_msg.name = joint_names_;
  edot_msg.name = joint_names_;
  eff_msg.name = joint_names_;
  err_msg.position.assign(n, 0.0);
  edot_msg.velocity.assign(n, 0.0);
  eff_msg.effort.assign(n, 0.0);

  sensor_msgs::msg::JointState grav_msg;
  grav_msg.header.stamp = stamp;
  grav_msg.name = joint_names_;
  grav_msg.effort.assign(n, 0.0);

  std::vector<double> current_positions(n, 0.0);
  for (size_t i = 0; i < n; ++i) {
    const size_t idx = js_index_.at(joint_names_[i]);
    current_positions[i] = last_js_.position.size() > idx ? last_js_.position[idx] : 0.0;
  }

  // --- Ruckig Profile Update ---
  sensor_msgs::msg::JointState ref_msg;
  ref_msg.header.stamp = stamp;
  ref_msg.name = joint_names_;
  ref_msg.position.assign(n, 0.0);
  ref_msg.velocity.assign(n, 0.0);

  if (enable_profile_ && profile_configured_ && otg_) {
    if (!profile_seeded_) {
      for (size_t i = 0; i < kProfileDoF; ++i) {
        otg_in_.current_position[i] = current_positions[i];
        otg_in_.current_velocity[i] = 0.0;
        otg_in_.current_acceleration[i] = 0.0;
      }
      profile_seeded_ = true;
    }
    auto res = otg_->update(otg_in_, otg_out_);
    if (res == ruckig::Result::Working || res == ruckig::Result::Finished) {
      otg_out_.pass_to_input(otg_in_);
      for (size_t i = 0; i < kProfileDoF; ++i) {
        q_ref_[i] = otg_out_.new_position[i];
        qdot_ref_[i] = otg_out_.new_velocity[i];
      }
    } else {
      for (size_t i = 0; i < kProfileDoF; ++i) {
        q_ref_[i] = reference_[i];
        qdot_ref_[i] = 0.0;
      }
    }
  } else {
    for (size_t i = 0; i < kProfileDoF; ++i) {
      q_ref_[i] = reference_[i];
      qdot_ref_[i] = 0.0;
    }
  }

  for (size_t i = 0; i < n; ++i) {
    ref_msg.position[i] = (i < kProfileDoF) ? q_ref_[i] : reference_[i];
    ref_msg.velocity[i] = (i < kProfileDoF) ? qdot_ref_[i] : 0.0;
  }
  pub_ref_->publish(ref_msg);

  std::vector<double> grav_torques(n, 0.0);
  if (grav_comp_ && enable_gravity_comp_) {
    grav_torques = grav_comp_->compute(current_positions);
  }

  for (size_t i = 0; i < n; ++i) {
    const size_t idx = js_index_.at(joint_names_[i]);
    const double pos = current_positions[i];
    const double vel = last_js_.velocity.size() > idx ? last_js_.velocity[idx] : 0.0;

    const double ref_p = (i < kProfileDoF) ? q_ref_[i] : reference_[i];
    const double ref_v = (i < kProfileDoF) ? qdot_ref_[i] : 0.0;

    const double e = ref_p - pos;
    const double ed = ref_v - vel;
    const double en = std::clamp(Ke_[i] * e, -1.0, 1.0);
    const double edn = std::clamp(Ked_[i] * ed, -1.0, 1.0);

    const float un = fuzzy_type1_eval(static_cast<float>(en), static_cast<float>(edn));
    // Ku = gain đầu ra (tunable); u_max = ngưỡng bão hòa an toàn (hardware cap, ≤ ±885).
    // Tách Ku khỏi u_max: tune độ khuếch đại vòng kín độc lập với giới hạn an toàn.
    double u = static_cast<double>(un) * Ku_[i];

    // --- Gravity compensation feedforward ---
    double grav_pwm = 0.0;
    if (grav_comp_ && enable_gravity_comp_) {
      grav_pwm = grav_torques[i] * Gff_[i] * gravity_sign_[i];
    }
    u += grav_pwm;

    u = std::clamp(u, -u_max_[i], u_max_[i]);

    cmd[i] = static_cast<float>(u);
    err_msg.position[i] = e;
    edot_msg.velocity[i] = ed;
    eff_msg.effort[i] = u;
    grav_msg.effort[i] = grav_pwm;
  }

  interbotix_xs_msgs::msg::JointGroupCommand cmd_msg;
  cmd_msg.name = group_name_;
  cmd_msg.cmd = cmd;
  pub_cmd_->publish(cmd_msg);

  pub_err_->publish(err_msg);
  pub_edot_->publish(edot_msg);
  pub_eff_->publish(eff_msg);
  pub_grav_->publish(grav_msg);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FuzzyNode>());
  rclcpp::shutdown();
  return 0;
}
