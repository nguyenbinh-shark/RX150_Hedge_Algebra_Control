#pragma once

#include <string>
#include <vector>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

class GravityCompensation {
 public:
  GravityCompensation(
      const std::string & urdf_xml, const std::vector<std::string> & joint_names);

  std::vector<double> compute(const std::vector<double> & q) const;

  // Mô hình gravity fitted (basis lượng giác, xem rx150_gravity_id.py):
  // coeffs[0..5]  = shoulder [cos m2, sin m2, cos m3, sin m3, cos m4, sin m4]
  // coeffs[6..9]  = elbow    [cos m3, sin m3, cos m4, sin m4]
  // coeffs[10..11]= wrist_angle [cos m4, sin m4]
  // với m2=q[1], m3=q[1]+q[2], m4=q[1]+q[2]+q[3] (q=[waist,shoulder,elbow,
  // wrist_angle,wrist_rotate]). waist/wrist_rotate luôn 0 (trục thẳng đứng /
  // trục roll — không chịu mô-men trọng lực). Không phụ thuộc pinocchio model
  // (chạy được kể cả khi isValid()==false), chỉ cần q có đủ 5 phần tử.
  static std::vector<double> computeFitted(
      const std::vector<double> & q, const std::vector<double> & coeffs);

  bool isValid() const { return valid_; }

 private:
  pinocchio::Model model_;
  mutable pinocchio::Data data_;
  std::vector<int> joint_id_map_;
  bool valid_ = false;
};
