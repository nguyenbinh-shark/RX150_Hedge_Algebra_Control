#include "rx150_fuzzy_controller/gravity_comp.hpp"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <iostream>

GravityCompensation::GravityCompensation(const std::string& urdf_xml, const std::vector<std::string>& joint_names) {
  try {
    pinocchio::urdf::buildModelFromXML(urdf_xml, model_);
    data_ = pinocchio::Data(model_);

    // Map requested joint names to pinocchio model joint IDs
    joint_id_map_.resize(joint_names.size(), -1);
    valid_ = true;

    for (size_t i = 0; i < joint_names.size(); ++i) {
      if (model_.existJointName(joint_names[i])) {
        // In Pinocchio, joint configuration vector q has indices corresponding to idx_q()
        // But for revolute joints, idx_q and idx_v are the same as the joint index in the q vector.
        // Let's get the joint ID
        pinocchio::JointIndex jid = model_.getJointId(joint_names[i]);
        // We need the index in the configuration vector q (and velocity vector v)
        // For revolute joints, nq = 1, nv = 1.
        int q_idx = model_.joints[jid].idx_v(); // We use idx_v because computeGeneralizedGravity returns torque vector of size nv
        joint_id_map_[i] = q_idx;
      } else {
        std::cerr << "[GravityCompensation] Joint not found in URDF: " << joint_names[i] << std::endl;
        valid_ = false;
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "[GravityCompensation] Failed to parse URDF: " << e.what() << std::endl;
    valid_ = false;
  }
}

std::vector<double> GravityCompensation::compute(const std::vector<double>& q) const {
  if (!valid_) {
    return std::vector<double>(joint_id_map_.size(), 0.0);
  }

  // Create full configuration vector for Pinocchio
  Eigen::VectorXd q_full = Eigen::VectorXd::Zero(model_.nq);

  // Fill in the known joint positions
  // Assuming the robot has only revolute joints, so nq == nv and q_full indices match joint_id_map_
  for (size_t i = 0; i < joint_id_map_.size(); ++i) {
    if (joint_id_map_[i] >= 0 && joint_id_map_[i] < model_.nq) {
      q_full[joint_id_map_[i]] = q[i];
    }
  }

  // Compute generalized gravity torque
  const Eigen::VectorXd& g = pinocchio::computeGeneralizedGravity(model_, data_, q_full);

  // Extract torques for the requested joints
  std::vector<double> torques(joint_id_map_.size(), 0.0);
  for (size_t i = 0; i < joint_id_map_.size(); ++i) {
    if (joint_id_map_[i] >= 0 && joint_id_map_[i] < model_.nv) {
      torques[i] = g[joint_id_map_[i]];
    }
  }

  return torques;
}
