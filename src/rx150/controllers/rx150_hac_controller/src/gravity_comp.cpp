#include "rx150_hac_controller/gravity_comp.hpp"

#include <cmath>
#include <iostream>

#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

GravityCompensation::GravityCompensation(
    const std::string & urdf_xml, const std::vector<std::string> & joint_names) {
  try {
    pinocchio::urdf::buildModelFromXML(urdf_xml, model_);
    data_ = pinocchio::Data(model_);

    joint_id_map_.resize(joint_names.size(), -1);
    valid_ = true;

    for (size_t i = 0; i < joint_names.size(); ++i) {
      if (model_.existJointName(joint_names[i])) {
        const pinocchio::JointIndex joint_id = model_.getJointId(joint_names[i]);
        joint_id_map_[i] = model_.joints[joint_id].idx_v();
      } else {
        std::cerr << "[GravityCompensation] Joint not found in URDF: "
                  << joint_names[i] << std::endl;
        valid_ = false;
      }
    }
  } catch (const std::exception & e) {
    std::cerr << "[GravityCompensation] Failed to parse URDF: " << e.what()
              << std::endl;
    valid_ = false;
  }
}

std::vector<double> GravityCompensation::compute(const std::vector<double> & q) const {
  if (!valid_) {
    return std::vector<double>(joint_id_map_.size(), 0.0);
  }

  Eigen::VectorXd q_full = Eigen::VectorXd::Zero(model_.nq);
  for (size_t i = 0; i < joint_id_map_.size(); ++i) {
    if (joint_id_map_[i] >= 0 && joint_id_map_[i] < model_.nq) {
      q_full[joint_id_map_[i]] = q[i];
    }
  }

  const Eigen::VectorXd & gravity =
      pinocchio::computeGeneralizedGravity(model_, data_, q_full);

  std::vector<double> torques(joint_id_map_.size(), 0.0);
  for (size_t i = 0; i < joint_id_map_.size(); ++i) {
    if (joint_id_map_[i] >= 0 && joint_id_map_[i] < model_.nv) {
      torques[i] = gravity[joint_id_map_[i]];
    }
  }

  return torques;
}

std::vector<double> GravityCompensation::computeFitted(
    const std::vector<double> & q, const std::vector<double> & coeffs) {
  std::vector<double> torques(5, 0.0);
  if (q.size() < 4 || coeffs.size() != 12) {
    return torques;
  }

  const double m2 = q[1];
  const double m3 = q[1] + q[2];
  const double m4 = q[1] + q[2] + q[3];

  // waist (0): trục thẳng đứng, không chịu mô-men trọng lực -> 0.
  torques[1] = coeffs[0] * std::cos(m2) + coeffs[1] * std::sin(m2) +
               coeffs[2] * std::cos(m3) + coeffs[3] * std::sin(m3) +
               coeffs[4] * std::cos(m4) + coeffs[5] * std::sin(m4);
  torques[2] = coeffs[6] * std::cos(m3) + coeffs[7] * std::sin(m3) +
               coeffs[8] * std::cos(m4) + coeffs[9] * std::sin(m4);
  torques[3] = coeffs[10] * std::cos(m4) + coeffs[11] * std::sin(m4);
  // wrist_rotate (4): trục roll, không chịu mô-men trọng lực -> 0.

  return torques;
}
