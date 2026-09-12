#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "hac.h"
#include "interbotix_xs_msgs/msg/joint_group_command.hpp"
#include "interbotix_xs_msgs/srv/operating_modes.hpp"
#include "interbotix_xs_msgs/srv/robot_info.hpp"
#include "interbotix_xs_msgs/srv/torque_enable.hpp"
#include "rx150_hac_controller/hac_node.hpp"

namespace {

bool validHacParameters(double a, double b, double c) {
  const float a_float = static_cast<float>(a);
  const float b_float = static_cast<float>(b);
  const float c_float = static_cast<float>(c);
  if (!std::isfinite(a) || !std::isfinite(b) || !std::isfinite(c) ||
      !std::isfinite(a_float) || !std::isfinite(b_float) ||
      !std::isfinite(c_float) || a_float <= 0.0f || b_float <= 0.0f) {
    return false;
  }

  const float position_gain = (2.0f * c_float) / (3.0f * a_float);
  const float velocity_gain = c_float / (3.0f * b_float);
  return std::isfinite(position_gain) && std::isfinite(velocity_gain) &&
         std::isfinite(std::fabs(position_gain) + std::fabs(velocity_gain));
}

}  // namespace

HacNode::HacNode() : rclcpp::Node("hac_node") {
  group_name_ = this->declare_parameter<std::string>("group_name", "arm");
  loop_rate_ = this->declare_parameter<double>("loop_rate", 100.0);
  watchdog_timeout_ = this->declare_parameter<double>("watchdog_timeout", 0.05);
  debug_publish_rate_ = this->declare_parameter<double>("debug_publish_rate", 50.0);
  velocity_filter_tau_ = this->declare_parameter<double>("velocity_filter_tau", 0.006);
  dbg_div_ = std::max(1, static_cast<int>(std::round(loop_rate_ / debug_publish_rate_)));
  error_limit_ = this->declare_parameter<std::vector<double>>("error_limit", std::vector<double>());
  error_dot_limit_ = this->declare_parameter<std::vector<double>>("error_dot_limit", std::vector<double>());
  u_max_ = this->declare_parameter<std::vector<double>>("u_max", std::vector<double>());

  reference_ =
      this->declare_parameter<std::vector<double>>("reference_pose", std::vector<double>());

  a_ = this->declare_parameter<double>("a", 0.3);
  b_ = this->declare_parameter<double>("b", 12.0);
  c_ = this->declare_parameter<double>("c", 1200.0);
  if (!validHacParameters(a_, b_, c_)) {
    throw std::invalid_argument(
        "invalid HAC parameters: a, b, c, and the derived gains must be finite; "
        "a and b must be positive");
  }

  enable_profile_ = this->declare_parameter<bool>("enable_profile", true);
  max_velocities_ = this->declare_parameter<std::vector<double>>(
      "max_velocities", std::vector<double>{3.14, 3.14, 3.14, 3.14, 3.14});
  max_accelerations_ = this->declare_parameter<std::vector<double>>(
      "max_accelerations", std::vector<double>{5.0, 5.0, 5.0, 5.0, 5.0});
  max_jerk_ = this->declare_parameter<double>("max_jerk", 0.0);
  sync_mode_ = this->declare_parameter<bool>("sync_mode", false);

  enable_gravity_comp_ = this->declare_parameter<bool>("enable_gravity_comp", true);
  Gff_ = this->declare_parameter<std::vector<double>>(
      "Gff", std::vector<double>{150.0, 255.0, 255.0, 500.0, 200.0});
  gravity_sign_ = this->declare_parameter<std::vector<double>>(
      "gravity_sign", std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0});
  gravity_model_source_ = this->declare_parameter<std::string>(
      "gravity_model_source", "pinocchio");
  fitted_gravity_coeffs_ = this->declare_parameter<std::vector<double>>(
      "fitted_gravity_coeffs", std::vector<double>(12, 0.0));

  friction_coulomb_ = this->declare_parameter<std::vector<double>>(
      "friction_coulomb", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0});
  friction_viscous_ = this->declare_parameter<std::vector<double>>(
      "friction_viscous", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0});
  friction_eps_ = this->declare_parameter<double>("friction_eps", 0.05);

  this->declare_parameter<std::string>("robot_description", "");

  cli_info_ =
      this->create_client<interbotix_xs_msgs::srv::RobotInfo>("get_robot_info");
  cli_modes_ = this->create_client<interbotix_xs_msgs::srv::OperatingModes>(
      "set_operating_modes");
  cli_torque_ =
      this->create_client<interbotix_xs_msgs::srv::TorqueEnable>("torque_enable");

  if (!cli_info_->wait_for_service(std::chrono::seconds(10))) {
    RCLCPP_ERROR(this->get_logger(), "get_robot_info service not available after 10s");
    return;
  }

  auto req = std::make_shared<interbotix_xs_msgs::srv::RobotInfo::Request>();
  req->cmd_type = "group";
  req->name = group_name_;
  cli_info_->async_send_request(
      req, std::bind(&HacNode::onRobotInfo, this, std::placeholders::_1));

  sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&HacNode::onJointStates, this, std::placeholders::_1));

  sub_setpoint_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "hac/setpoint", 10,
      std::bind(&HacNode::onSetpoint, this, std::placeholders::_1));
  pub_err_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "hac/error", rclcpp::SensorDataQoS());
  pub_edot_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "hac/edot", rclcpp::SensorDataQoS());
  pub_eff_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "hac/effort", rclcpp::SensorDataQoS());
  pub_ref_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "hac/reference", rclcpp::SensorDataQoS());
  pub_grav_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "hac/gravity", rclcpp::SensorDataQoS());
  pub_grav_torque_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "hac/gravity_torque", rclcpp::SensorDataQoS());
  pub_fric_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "hac/friction", rclcpp::SensorDataQoS());
  pub_cmd_ = this->create_publisher<interbotix_xs_msgs::msg::JointGroupCommand>(
      "commands/joint_group", 10);

  timer_ = this->create_wall_timer(
      std::chrono::microseconds(static_cast<int64_t>(1e6 / loop_rate_)),
      std::bind(&HacNode::onTimer, this));

  param_cb_handle_ = this->add_on_set_parameters_callback(
      std::bind(&HacNode::onParamChange, this, std::placeholders::_1));

  rclcpp::on_shutdown([this]() {
    try {
      interbotix_xs_msgs::msg::JointGroupCommand zero_msg;
      zero_msg.name = group_name_;
      zero_msg.cmd.assign(joint_names_.size(), 0.0f);
      pub_cmd_->publish(zero_msg);

      auto torque_req =
          std::make_shared<interbotix_xs_msgs::srv::TorqueEnable::Request>();
      torque_req->cmd_type = "group";
      torque_req->name = group_name_;
      torque_req->enable = false;
      cli_torque_->async_send_request(torque_req);

      RCLCPP_INFO(this->get_logger(), "shutdown: zero + torque off");
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "shutdown handler error: %s", e.what());
    }
  });
}

void HacNode::onRobotInfo(
    rclcpp::Client<interbotix_xs_msgs::srv::RobotInfo>::SharedFuture future) {
  auto resp = future.get();
  joint_names_ = resp->joint_names;

  js_index_.clear();
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    js_index_[joint_names_[i]] =
        static_cast<size_t>(resp->joint_state_indices.at(i));
  }

  const std::string urdf_xml = this->get_parameter("robot_description").as_string();
  if (!urdf_xml.empty()) {
    grav_comp_ = std::make_unique<GravityCompensation>(urdf_xml, joint_names_);
    if (!grav_comp_->isValid()) {
      RCLCPP_WARN(
          this->get_logger(),
          "GravityCompensation initialization failed; disabling gravity compensation");
      enable_gravity_comp_ = false;
    }
  } else {
    RCLCPP_WARN(
        this->get_logger(),
        "robot_description is empty; disabling gravity compensation");
    enable_gravity_comp_ = false;
  }

  if (reference_.empty()) {
    reference_.assign(
        resp->joint_sleep_positions.begin(), resp->joint_sleep_positions.end());
  }

  const size_t n = joint_names_.size();
  if (error_limit_.size() != n || error_dot_limit_.size() != n ||
      u_max_.size() != n || reference_.size() != n ||
      friction_coulomb_.size() != n || friction_viscous_.size() != n) {
    RCLCPP_ERROR(
        this->get_logger(),
        "param size mismatch: joints=%zu, error_limit=%zu, error_dot_limit=%zu, u_max=%zu, "
        "reference=%zu, friction_coulomb=%zu, friction_viscous=%zu",
        n, error_limit_.size(), error_dot_limit_.size(), u_max_.size(), reference_.size(),
        friction_coulomb_.size(), friction_viscous_.size());
    return;
  }
  if (fitted_gravity_coeffs_.size() != 12) {
    RCLCPP_ERROR(
        this->get_logger(), "fitted_gravity_coeffs must have exactly 12 values (got %zu)",
        fitted_gravity_coeffs_.size());
    return;
  }

  if (enable_profile_) {
    if (n != kProfileDoF) {
      RCLCPP_WARN(
          this->get_logger(),
          "enable_profile=true but arm joints=%zu != %zu; disabling profile", n,
          kProfileDoF);
      enable_profile_ = false;
    } else if (max_velocities_.size() != n || max_accelerations_.size() != n) {
      RCLCPP_WARN(
          this->get_logger(),
          "max_velocities/max_accelerations size != %zu; disabling profile", n);
      enable_profile_ = false;
    } else {
      configureProfile();
    }
  }

  auto mode_req =
      std::make_shared<interbotix_xs_msgs::srv::OperatingModes::Request>();
  mode_req->cmd_type = "group";
  mode_req->name = group_name_;
  mode_req->mode = "pwm";
  mode_req->profile_type = "time";
  mode_req->profile_velocity = 0;
  mode_req->profile_acceleration = 0;
  cli_modes_->async_send_request(mode_req);

  auto torque_req =
      std::make_shared<interbotix_xs_msgs::srv::TorqueEnable::Request>();
  torque_req->cmd_type = "group";
  torque_req->name = group_name_;
  torque_req->enable = true;
  cli_torque_->async_send_request(torque_req);

  ready_ = true;
  RCLCPP_INFO(
      this->get_logger(), "hac ready: %zu joints, pwm mode%s", n,
      enable_profile_ ? ", profile TOTG on" : "");
}

void HacNode::configureProfile() {
  const double dt = 1.0 / loop_rate_;
  otg_.emplace(dt);
  for (size_t i = 0; i < kProfileDoF; ++i) {
    otg_in_.target_position[i] = reference_[i];
    otg_in_.target_velocity[i] = 0.0;
    otg_in_.target_acceleration[i] = 0.0;
    otg_in_.max_velocity[i] = max_velocities_[i];
    otg_in_.max_acceleration[i] = max_accelerations_[i];
    otg_in_.max_jerk[i] =
        (max_jerk_ > 0.0) ? max_jerk_ : 100.0 * max_accelerations_[i];
  }
  otg_in_.synchronization =
      sync_mode_ ? ruckig::Synchronization::Time : ruckig::Synchronization::None;
  profile_configured_ = true;
  RCLCPP_INFO(
      this->get_logger(),
      "profile TOTG(Ruckig): dt=%.3fs v=%.2f a=%.2f jerk=%.0f sync=%s", dt,
      max_velocities_[0], max_accelerations_[0],
      (max_jerk_ > 0.0) ? max_jerk_ : 100.0 * max_accelerations_[0],
      sync_mode_ ? "Time" : "None");
}

void HacNode::onJointStates(const sensor_msgs::msg::JointState::SharedPtr msg) {
  last_js_ = *msg;
  rclcpp::Time now = this->now();
  if (!filter_initialized_) {
    filtered_velocity_ = last_js_.velocity;
    last_filter_time_ = now;
    filter_initialized_ = true;
  } else {
    double dt = (now - last_filter_time_).seconds();
    if (dt > 0.0 && dt < 0.2 && filtered_velocity_.size() == last_js_.velocity.size()) {
      double alpha = dt / (velocity_filter_tau_ + dt);
      for (size_t i = 0; i < last_js_.velocity.size(); ++i) {
        filtered_velocity_[i] = alpha * last_js_.velocity[i] + (1.0 - alpha) * filtered_velocity_[i];
      }
    } else {
      filtered_velocity_ = last_js_.velocity;
    }
    last_filter_time_ = now;
  }
  last_js_stamp_ = now;
  have_js_ = true;
}

void HacNode::onSetpoint(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if (!ready_) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "setpoint arrived before onRobotInfo; ignoring");
    return;
  }
  if (msg->data.size() != reference_.size()) {
    RCLCPP_WARN(
        this->get_logger(), "setpoint joint count mismatch: %zu != %zu; ignoring",
        msg->data.size(), reference_.size());
    return;
  }

  reference_.assign(msg->data.begin(), msg->data.end());
  if (profile_configured_) {
    for (size_t i = 0; i < kProfileDoF; ++i) {
      otg_in_.target_position[i] = reference_[i];
    }
  }
  RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "setpoint updated: [%.3f %.3f %.3f %.3f %.3f]", reference_[0],
      reference_[1], reference_[2], reference_[3], reference_[4]);
}

rcl_interfaces::msg::SetParametersResult HacNode::onParamChange(
    const std::vector<rclcpp::Parameter> & params) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  double next_a = a_;
  double next_b = b_;
  double next_c = c_;
  double next_friction_eps = friction_eps_;

  for (const auto & parameter : params) {
    const std::string & name = parameter.get_name();
    const bool is_scalar = name == "a" || name == "b" || name == "c" || name == "friction_eps";
    if (is_scalar) {
      if (parameter.get_type() != rclcpp::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = name + " must be a double scalar";
        break;
      }

      const double value = parameter.as_double();
      if (name == "a") {
        next_a = value;
      } else if (name == "b") {
        next_b = value;
      } else if (name == "c") {
        next_c = value;
      } else {
        next_friction_eps = value;
      }
      continue;
    }

    const bool is_joint_array = name == "error_limit" || name == "error_dot_limit" ||
                               name == "u_max" || name == "Gff" ||
                               name == "friction_coulomb" || name == "friction_viscous" ||
                               name == "max_velocities" || name == "max_accelerations";
    const bool is_coeff_array = name == "fitted_gravity_coeffs";
    if (!is_joint_array && !is_coeff_array) {
      continue;
    }
    if (parameter.get_type() != rclcpp::PARAMETER_DOUBLE_ARRAY) {
      result.successful = false;
      result.reason = name + " must be a double array";
      break;
    }

    const std::vector<double> values = parameter.as_double_array();
    const size_t expected = is_coeff_array ? 12 : error_limit_.size();
    if (values.size() != expected) {
      result.successful = false;
      result.reason = name + " size " + std::to_string(values.size()) + " != " +
                      std::to_string(expected);
      break;
    }
  }

  if (!result.successful) {
    return result;
  }
  if (!validHacParameters(next_a, next_b, next_c)) {
    result.successful = false;
    result.reason =
        "a, b, c, and the derived HAC gains must be finite; a and b must be "
        "positive double scalars";
    return result;
  }
  if (!std::isfinite(next_friction_eps) || next_friction_eps <= 0.0) {
    result.successful = false;
    result.reason = "friction_eps must be finite and positive";
    return result;
  }

  for (const auto & parameter : params) {
    const std::string & name = parameter.get_name();
    if (name == "a") {
      a_ = parameter.as_double();
      RCLCPP_INFO(this->get_logger(), "live param a = %g", a_);
    } else if (name == "b") {
      b_ = parameter.as_double();
      RCLCPP_INFO(this->get_logger(), "live param b = %g", b_);
    } else if (name == "c") {
      c_ = parameter.as_double();
      RCLCPP_INFO(this->get_logger(), "live param c = %g", c_);
    } else if (name == "friction_eps") {
      friction_eps_ = parameter.as_double();
      RCLCPP_INFO(this->get_logger(), "live param friction_eps = %g", friction_eps_);
    } else if (
        name == "error_limit" || name == "error_dot_limit" || name == "u_max" ||
        name == "Gff" || name == "friction_coulomb" || name == "friction_viscous" ||
        name == "fitted_gravity_coeffs" || name == "max_velocities" ||
        name == "max_accelerations") {
      const std::vector<double> values = parameter.as_double_array();
      if (name == "error_limit") {
        error_limit_ = values;
      } else if (name == "error_dot_limit") {
        error_dot_limit_ = values;
      } else if (name == "u_max") {
        u_max_ = values;
      } else if (name == "Gff") {
        Gff_ = values;
      } else if (name == "friction_coulomb") {
        friction_coulomb_ = values;
      } else if (name == "friction_viscous") {
        friction_viscous_ = values;
      } else if (name == "max_velocities") {
        max_velocities_ = values;
        if (profile_configured_ && max_velocities_.size() == kProfileDoF) {
          for (size_t i = 0; i < kProfileDoF; ++i) {
            otg_in_.max_velocity[i] = max_velocities_[i];
          }
        }
      } else if (name == "max_accelerations") {
        max_accelerations_ = values;
        if (profile_configured_ && max_accelerations_.size() == kProfileDoF) {
          for (size_t i = 0; i < kProfileDoF; ++i) {
            otg_in_.max_acceleration[i] = max_accelerations_[i];
          }
        }
      } else {
        fitted_gravity_coeffs_ = values;
      }
      RCLCPP_INFO(
          this->get_logger(), "live param %s updated (%zu values)", name.c_str(),
          values.size());
    }
  }

  return result;
}

void HacNode::onTimer() {
  if (!ready_) {
    return;
  }
  if (!have_js_) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000, "waiting for joint_states");
    return;
  }
  if ((this->now() - last_js_stamp_).seconds() > watchdog_timeout_) {
    if (!in_watchdog_ramp_) {
      in_watchdog_ramp_ = true;
      watchdog_trip_time_ = this->now();
    }
    const double elapsed = (this->now() - watchdog_trip_time_).seconds();

    // Ba giai đoạn. Cắt PWM về 0 tức thì sẽ làm tay máy RƠI, nên phải hạ dần;
    // nhưng giữ bù trọng lực VĨNH VIỄN cũng sai: giá trị đó tính từ tư thế cuối
    // cùng đọc được, mà tư thế thì vẫn thay đổi sau khi mất joint_states (người
    // đẩy tay, hoặc tay tự trượt). Ôm một giá trị đóng băng không còn khớp tư thế
    // thật thì chính nó thành nguồn mô-men sai. Vì vậy giữ có thời hạn rồi thả.
    //
    //   0 .. 20 ms          : hạ phần điều khiển, giữ nguyên phần trọng lực
    //   20 ms .. 520 ms     : giữ trọng lực — đủ để bus/DDS vấp rồi hồi
    //   520 ms .. 1020 ms   : hạ nốt trọng lực về 0
    //   > 1020 ms           : PWM = 0 (motor tự do, cần kê đỡ)
    const double kCmdRamp = 0.020;
    const double kGravHold = 0.500;
    const double kGravRamp = 0.500;

    const double cmd_scale =
        (elapsed < kCmdRamp) ? (1.0 - elapsed / kCmdRamp) : 0.0;
    double grav_scale = 1.0;
    if (elapsed >= kCmdRamp + kGravHold) {
      const double t = elapsed - (kCmdRamp + kGravHold);
      grav_scale = (t < kGravRamp) ? (1.0 - t / kGravRamp) : 0.0;
    }

    RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "stale joint_states (%.3f s) -> cmd x%.2f, gravity x%.2f",
        elapsed, cmd_scale, grav_scale);

    interbotix_xs_msgs::msg::JointGroupCommand safe_msg;
    safe_msg.name = group_name_;
    safe_msg.cmd.resize(joint_names_.size(), 0.0f);
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      const float grav = (i < last_grav_pwm_.size()) ? last_grav_pwm_[i] : 0.0f;
      const float base = (i < last_cmd_pwm_.size()) ? last_cmd_pwm_[i] : 0.0f;
      safe_msg.cmd[i] = static_cast<float>(grav_scale) * grav +
                        static_cast<float>(cmd_scale) * (base - grav);
    }
    pub_cmd_->publish(safe_msg);
    return;
  }
  in_watchdog_ramp_ = false;

  const size_t n = joint_names_.size();
  std::vector<float> command(n, 0.0f);

  sensor_msgs::msg::JointState error_msg;
  sensor_msgs::msg::JointState edot_msg;
  sensor_msgs::msg::JointState effort_msg;
  const rclcpp::Time stamp = this->now();
  error_msg.header.stamp = stamp;
  edot_msg.header.stamp = stamp;
  effort_msg.header.stamp = stamp;
  error_msg.name = joint_names_;
  edot_msg.name = joint_names_;
  effort_msg.name = joint_names_;
  error_msg.position.assign(n, 0.0);
  edot_msg.velocity.assign(n, 0.0);
  effort_msg.effort.assign(n, 0.0);

  sensor_msgs::msg::JointState gravity_msg;
  gravity_msg.header.stamp = stamp;
  gravity_msg.name = joint_names_;
  gravity_msg.effort.assign(n, 0.0);

  sensor_msgs::msg::JointState grav_torque_msg;
  grav_torque_msg.header.stamp = stamp;
  grav_torque_msg.name = joint_names_;

  sensor_msgs::msg::JointState friction_msg;
  friction_msg.header.stamp = stamp;
  friction_msg.name = joint_names_;
  friction_msg.effort.assign(n, 0.0);

  std::vector<double> current_positions(n, 0.0);
  for (size_t i = 0; i < n; ++i) {
    const size_t index = js_index_.at(joint_names_[i]);
    current_positions[i] =
        last_js_.position.size() > index ? last_js_.position[index] : 0.0;
  }

  sensor_msgs::msg::JointState reference_msg;
  reference_msg.header.stamp = stamp;
  reference_msg.name = joint_names_;
  reference_msg.position.assign(n, 0.0);
  reference_msg.velocity.assign(n, 0.0);

  if (enable_profile_ && profile_configured_ && otg_) {
    if (!profile_seeded_) {
      for (size_t i = 0; i < kProfileDoF; ++i) {
        otg_in_.current_position[i] = current_positions[i];
        otg_in_.current_velocity[i] = 0.0;
        otg_in_.current_acceleration[i] = 0.0;
      }
      profile_seeded_ = true;
    }

    const auto result = otg_->update(otg_in_, otg_out_);
    if (result == ruckig::Result::Working || result == ruckig::Result::Finished) {
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
    const double kVelLimit = 13.7;  // ~131 Dynamixel units (0.229 rpm/unit) ≈ 13.7 rad/s
    for (size_t i = 0; i < kProfileDoF; ++i) {
      qdot_ref_[i] = (reference_[i] - q_ref_[i]) * loop_rate_;
      qdot_ref_[i] = std::clamp(qdot_ref_[i], -kVelLimit, kVelLimit);
      q_ref_[i] = reference_[i];
    }
  }

  for (size_t i = 0; i < n; ++i) {
    reference_msg.position[i] = i < kProfileDoF ? q_ref_[i] : reference_[i];
    reference_msg.velocity[i] = i < kProfileDoF ? qdot_ref_[i] : 0.0;
  }

  // Pinocchio raw torque (N·m) LUÔN được tính (kể cả enable_gravity_comp=false)
  // để B0/B1 (rx150_gravity_id.py) có tín hiệu tham chiếu độc lập với
  // Gff/gravity_sign qua topic hac/gravity_torque; cũng dùng làm cross-check
  // khi gravity_model_source=fitted.
  std::vector<double> gravity_pinocchio(n, 0.0);
  if (grav_comp_) {
    gravity_pinocchio = grav_comp_->compute(current_positions);
  }
  grav_torque_msg.effort = gravity_pinocchio;

  // fitted_gravity_coeffs được fit TRỰC TIẾP ở đơn vị PWM (rx150_gravity_id.py,
  // mode identify) — hấp thụ luôn sai số Gff lẫn khối lượng URDF, nên KHÔNG
  // nhân thêm Gff/gravity_sign (khác với đường Pinocchio bên dưới, vốn ra N·m
  // và cần Gff*sign để đổi sang PWM).
  std::vector<double> fitted_gravity_pwm(n, 0.0);
  if (gravity_model_source_ == "fitted") {
    fitted_gravity_pwm = GravityCompensation::computeFitted(current_positions, fitted_gravity_coeffs_);
  }

  for (size_t i = 0; i < n; ++i) {
    const size_t index = js_index_.at(joint_names_[i]);
    const double position = current_positions[i];
    const double velocity =
        filtered_velocity_.size() > index ? filtered_velocity_[index] :
        (last_js_.velocity.size() > index ? last_js_.velocity[index] : 0.0);

    const double ref_position = i < kProfileDoF ? q_ref_[i] : reference_[i];
    const double ref_velocity = i < kProfileDoF ? qdot_ref_[i] : 0.0;
    const double error = ref_position - position;
    const double error_dot = ref_velocity - velocity;
    // Direct saturation (NOT normalization): clamp error/error_dot vào [-limit, +limit]
    const double saturated_error = std::clamp(error, -error_limit_[i], error_limit_[i]);
    const double saturated_error_dot =
        std::clamp(error_dot, -error_dot_limit_[i], error_dot_limit_[i]);

    const float un = hac_eval(
        static_cast<float>(saturated_error),
        static_cast<float>(saturated_error_dot), static_cast<float>(a_),
        static_cast<float>(b_), static_cast<float>(c_));
    double output = static_cast<double>(un);

    double gravity_pwm = 0.0;
    if (enable_gravity_comp_) {
      if (gravity_model_source_ == "fitted") {
        gravity_pwm = fitted_gravity_pwm[i];
      } else if (grav_comp_) {
        gravity_pwm = gravity_pinocchio[i] * Gff_[i] * gravity_sign_[i];
      }
    }
    output += gravity_pwm;

    // Friction feedforward (Coulomb + viscous), SAU gravity, TRƯỚC clamp
    // ±u_max. Dùng vận tốc ĐO đã lọc (velocity), không dùng error_dot: error_dot
    // phản ánh sai số bám theo profile, không phải chuyển động thực của khớp.
    const double friction_pwm =
        friction_coulomb_[i] * std::tanh(velocity / friction_eps_) +
        friction_viscous_[i] * velocity;
    output += friction_pwm;

    output = std::clamp(output, -u_max_[i], u_max_[i]);

    command[i] = static_cast<float>(output);
    error_msg.position[i] = error;
    edot_msg.velocity[i] = error_dot;
    effort_msg.effort[i] = output;
    gravity_msg.effort[i] = gravity_pwm;
    friction_msg.effort[i] = friction_pwm;
  }

  // Lưu trạng thái lệnh gần nhất phục vụ ramp khi watchdog trip
  last_cmd_pwm_ = command;
  last_grav_pwm_.resize(n, 0.0f);
  for (size_t i = 0; i < n; ++i) {
    last_grav_pwm_[i] = static_cast<float>(gravity_msg.effort[i]);
  }

  interbotix_xs_msgs::msg::JointGroupCommand command_msg;
  command_msg.name = group_name_;
  command_msg.cmd = command;
  pub_cmd_->publish(command_msg);

  // Throttle các topic debug xuống debug_publish_rate (mặc định 50Hz)
  if (++dbg_cnt_ % dbg_div_ == 0) {
    pub_ref_->publish(reference_msg);
    pub_err_->publish(error_msg);
    pub_edot_->publish(edot_msg);
    pub_eff_->publish(effort_msg);
    pub_grav_->publish(gravity_msg);
    pub_grav_torque_->publish(grav_torque_msg);
    pub_fric_->publish(friction_msg);
  }
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HacNode>());
  rclcpp::shutdown();
  return 0;
}
