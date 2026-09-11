#!/usr/bin/env python3
"""Bậc B1/B5 — mọi action server + service mà pick-place cần đã sẵn sàng chưa?

Đây là bài test trả lời câu hỏi "vì sao node treo ở wait_ready()" mà không phải
đọc log. Nó KHÔNG gửi goal nào — chỉ hỏi discovery xem server có mặt không, nên
an toàn tuyệt đối khi robot đã cấp điện.

Kiểm theo NHÓM, vì mỗi nhóm hỏng cho một triệu chứng khác nhau:

  arm      /rx150/arm_controller/follow_joint_trajectory      -> thiếu = bridge chưa chạy
  gripper  /rx150/gripper_controller/follow_joint_trajectory  -> thiếu = ff_moveit.launch.py
                                                                 (nhánh này không có bridge)
  moveit   move_action, execute_trajectory                    -> thiếu = move_group chưa lên
  scene    /apply_planning_scene, /get_planning_scene         -> thiếu = scene ops treo 3 s/lần
  driver   /rx150/set_operating_modes, /rx150/torque_enable   -> thiếu = xs_sdk chưa chạy

Với `motion_backend: moveit` cần đủ CẢ NĂM. Với `direct` chỉ cần arm + driver;
thiếu moveit/scene chỉ làm chậm (xem RUNBOOK B5b), không chết.

Dùng:
  python3 module_tests/run_test.py moveit/action_servers_test.py
  python3 module_tests/run_test.py moveit/action_servers_test.py -- --backend direct
"""
from __future__ import annotations

import argparse
import sys
import time

ARM_ACTION = "/rx150/arm_controller/follow_joint_trajectory"
GRIPPER_ACTION = "/rx150/gripper_controller/follow_joint_trajectory"

# (nhóm, tên, bắt buộc cho backend nào, gợi ý khi thiếu)
ACTIONS = [
    ("arm", ARM_ACTION, ("moveit", "direct"),
     "fuzzy_trajectory_bridge chưa chạy — './rx150.sh t1'"),
    ("gripper", GRIPPER_ACTION, ("moveit",),
     "gripper_trajectory_bridge chưa chạy — ff_moveit.launch.py KHÔNG khởi động nó"),
    ("moveit", "/move_action", ("moveit",),
     "move_group chưa lên; kiểm 'ros2 pkg prefix moveit_ros_move_group'"),
    ("moveit", "/execute_trajectory", ("moveit",),
     "move_group chưa lên (đoạn đi thẳng Descartes cần action này)"),
]

SERVICES = [
    ("scene", "/apply_planning_scene", ("moveit",),
     "move_group chưa lên — scene ops sẽ treo 3 s rồi WARN"),
    ("scene", "/get_planning_scene", ("moveit",),
     "move_group chưa lên — set_collision_allowed sẽ bỏ qua"),
    ("driver", "/rx150/set_operating_modes", ("moveit", "direct"),
     "xs_sdk chưa chạy — không đặt được gripper sang PWM mode"),
    ("driver", "/rx150/torque_enable", ("moveit", "direct"),
     "xs_sdk chưa chạy"),
    ("driver", "/rx150/get_robot_info", ("moveit", "direct"),
     "xs_sdk chưa chạy"),
]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--backend", choices=("moveit", "direct"), default="moveit",
                    help="quyết định cái nào là BẮT BUỘC (mặc định: moveit)")
    ap.add_argument("--settle", type=float, default=2.0,
                    help="giây chờ discovery ổn định trước khi liệt kê")
    args = ap.parse_args()

    try:
        import rclpy
        from rclpy.node import Node
    except ImportError as exc:
        print(f"FAIL: không import được rclpy ({exc}).")
        print("      Hãy 'source source_all.sh' trước.")
        return 1

    rclpy.init()
    node = Node("action_servers_test")

    # Discovery không tức thời: hỏi ngay sau init thì danh sách còn rỗng.
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.settle:
        rclpy.spin_once(node, timeout_sec=0.05)

    have_actions = _list_actions(node)
    have_services = {name for name, _ in node.get_service_names_and_types()}
    node_names = sorted(
        (ns.rstrip("/") + "/" + n).replace("//", "/")
        for n, ns in node.get_node_names_and_namespaces()
    )

    node.destroy_node()
    rclpy.shutdown()

    print(f"Backend kiểm theo: {args.backend}")
    print(f"Node đang chạy ({len(node_names)}): {', '.join(node_names) or '(không có)'}")
    print()

    missing_required: list[str] = []

    def check(group, name, required_for, hint, present):
        required = args.backend in required_for
        tag = "BẮT BUỘC" if required else "tuỳ chọn"
        if present:
            print(f"OK  : [{group}] {name}")
        elif required:
            missing_required.append(name)
            print(f"FAIL: [{group}] {name} — THIẾU ({tag})")
            print(f"      → {hint}")
        else:
            print(f"WARN: [{group}] {name} — thiếu ({tag} với backend={args.backend})")
            print(f"      → {hint}")

    print("--- ACTION ---")
    for group, name, required_for, hint in ACTIONS:
        check(group, name, required_for, hint, name in have_actions)

    print("\n--- SERVICE ---")
    for group, name, required_for, hint in SERVICES:
        check(group, name, required_for, hint, name in have_services)

    if missing_required:
        print(f"\n=> NO-GO: thiếu {len(missing_required)} thứ BẮT BUỘC: "
              + ", ".join(missing_required))
        return 1
    print(f"\n=> GO: đủ mọi action/service bắt buộc cho backend={args.backend}.")
    return 0


def _list_actions(node) -> set:
    """Tên action server đang có, dạng đầy đủ.

    rclpy không có API 'liệt kê action' trực tiếp; cách chuẩn (cũng là cách
    `ros2 action list` dùng) là suy ra từ topic feedback của mỗi action.
    """
    actions = set()
    for topic, types in node.get_topic_names_and_types():
        if topic.endswith("/_action/feedback"):
            actions.add(topic[: -len("/_action/feedback")])
    return actions


if __name__ == "__main__":
    sys.exit(main())
