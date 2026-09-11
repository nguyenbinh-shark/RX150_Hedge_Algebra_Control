"""Đo sai số xác lập (steady-state) của hac_node trên 1 bộ tư thế cố định.

Dùng lại cùng bộ tư thế trước/sau hiệu chuẩn gravity để so sánh có nghĩa.
Publish thẳng /rx150/hac/setpoint (không qua MoveIt) -> đo đúng vòng PWM.
"""
import json, math, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

ARM = ["waist", "shoulder", "elbow", "wrist_angle", "wrist_rotate"]
# home của pick_place + các tư thế vươn mà chuỗi APPROACH/DESCEND thực sự đi qua.
POSES = {
    "sleep":      [0.0, -1.80,  1.55,  0.80, 0.0],
    "home":       [0.0, -1.260, 1.381, -0.120, 0.0],
    "reach_far":  [0.0, -0.90,  1.00,  0.30, 0.0],
    "reach_low":  [0.0, -0.55,  0.95,  0.55, 0.0],
    "reach_side": [0.6, -0.90,  1.10,  0.20, -0.6],
    "up_high":    [0.0, -1.50,  0.70, -0.20, 0.0],
}

class Bench(Node):
    def __init__(self):
        super().__init__("ss_bench")
        self.js = None
        self.err = None
        self.pwm = None
        self.create_subscription(JointState, "/rx150/joint_states",
                                 lambda m: setattr(self, "js", m), qos_profile_sensor_data)
        self.create_subscription(JointState, "/rx150/hac/error",
                                 lambda m: setattr(self, "err", m), qos_profile_sensor_data)
        self.create_subscription(JointState, "/rx150/hac/effort",
                                 lambda m: setattr(self, "pwm", m), qos_profile_sensor_data)
        self.pub = self.create_publisher(Float64MultiArray, "/rx150/hac/setpoint", 10)

    def hold(self, q, seconds):
        msg = Float64MultiArray(); msg.data = [float(v) for v in q]
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)

    def sample(self, q, seconds=2.0):
        msg = Float64MultiArray(); msg.data = [float(v) for v in q]
        errs, pwms = [], []
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.err is not None:
                idx = {n: i for i, n in enumerate(self.err.name)}
                errs.append([self.err.position[idx[j]] for j in ARM])
            if self.pwm is not None:
                idx = {n: i for i, n in enumerate(self.pwm.name)}
                pwms.append([self.pwm.effort[idx[j]] for j in ARM])
        return (np.mean(np.array(errs), axis=0) if errs else np.full(5, np.nan),
                np.mean(np.array(pwms), axis=0) if pwms else np.full(5, np.nan))

def main():
    label = sys.argv[1] if len(sys.argv) > 1 else "run"
    out = sys.argv[2] if len(sys.argv) > 2 else None
    rclpy.init()
    b = Bench()
    b.hold(POSES["sleep"], 3.0)          # điểm xuất phát chung
    results = {}
    print(f"{'pose':<11} " + "  ".join(f"{j[:5]:>7}" for j in ARM) + "     max|e|")
    for name, q in POSES.items():
        b.hold(q, 5.0)                    # profile 0.5 rad/s -> đủ tới nơi
        e, p = b.sample(q, 2.0)
        deg = np.degrees(e)
        results[name] = {"err_deg": deg.tolist(), "pwm": p.tolist()}
        print(f"{name:<11} " + "  ".join(f"{v:+7.2f}" for v in deg)
              + f"   {np.nanmax(np.abs(deg)):6.2f}°")
    b.hold(POSES["sleep"], 6.0)          # trả về sleep để tắt an toàn
    allmax = max(np.nanmax(np.abs(v["err_deg"])) for v in results.values())
    perjoint = np.nanmax(np.abs(np.array([v["err_deg"] for v in results.values()])), axis=0)
    print(f"\n{'MAX theo khớp':<11} " + "  ".join(f"{v:+7.2f}" for v in perjoint)
          + f"   {allmax:6.2f}°")
    if out:
        with open(out, "w") as fh:
            json.dump({"label": label, "results": results,
                       "max_abs_deg": float(allmax),
                       "per_joint_max_deg": perjoint.tolist()}, fh, indent=2)
        print(f"ghi: {out}")
    b.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
