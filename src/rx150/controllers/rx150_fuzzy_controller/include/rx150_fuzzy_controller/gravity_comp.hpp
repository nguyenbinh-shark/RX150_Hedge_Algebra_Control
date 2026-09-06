#pragma once

#include <string>
#include <vector>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

class GravityCompensation {
public:
  GravityCompensation(const std::string& urdf_xml, const std::vector<std::string>& joint_names);

  std::vector<double> compute(const std::vector<double>& q) const;

  bool isValid() const { return valid_; }

private:
  pinocchio::Model model_;
  mutable pinocchio::Data data_;
  std::vector<int> joint_id_map_;  // map arm joint index -> pinocchio joint id
  bool valid_ = false;
};
