"""rx150_modules — thư viện dùng chung cho mọi ứng dụng RX150 (Application Support).

Tách khỏi node để: (1) hết copy-paste 3 bản MoveGroup primitive giữa
pick_place_moveit_node / tube_rack_node / rx150_hri, (2) toán học hình học
kiểm chứng được offline bằng pytest (không cần robot).

  kinematics — IK/FK giải tích 5-DoF rx150 (thay IK-oracle của SDK)
  motion     — MoveGroup client (joint goal + linear approach) + verify pose
  gripper    — đóng/mở + XÁC NHẬN đã kẹp được vật (part-present check)
  scene      — planning scene: box/cylinder + attach/detach vật đang kẹp
  status     — state machine + status JSON + thống kê chu kỳ
"""
