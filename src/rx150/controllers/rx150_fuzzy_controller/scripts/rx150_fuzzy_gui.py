#!/usr/bin/env python3
"""
fuzzy_gui — GUI tune node fuzzy (rx150): 2 tab.

Tab "Setpoint": 5 slider đặt vị trí đích (rad) -> publish /rx150/fuzzy/setpoint.
Tab "Gains":    lưới 4 gain (Ke/Ked/Ku/u_max) × 5 khớp -> set_parameters
                trên /rx150/fuzzy_node (dùng callback live-gain, không relaunch).

Chạy:
  ros2 run fuzzy_controller fuzzy_gui
  ros2 run fuzzy_controller fuzzy_gui --ros-args \
      -p setpoint_topic:=/rx150/fuzzy/setpoint \
      -p target_node:=/rx150/fuzzy_node

Lưu ý: KHÔNG qua MoveIt (MoveIt cần POSITION mode, xung đột PWM mode của fuzzy).
"""

import sys
import math
import tkinter as tk
from tkinter import ttk, simpledialog

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterValue, ParameterType
from std_msgs.msg import Float64MultiArray

# Khớp fuzzy_gains.yaml.
DEFAULT_JOINTS = ["waist", "shoulder", "elbow", "wrist_angle", "wrist_rotate"]
DEFAULT_POSE = [0.0, -1.80, 1.55, 0.8, 0.0]
DEFAULT_RANGE = math.pi
GAIN_ORDER = ["Ke", "Ked", "Ku", "u_max", "Gff"]
GAIN_DEFAULTS = {
    "Ke":   [0.2, 0.2, 0.2, 0.2, 0.2],
    "Ked":  [0.0005, 0.0005, 0.0005, 0.0005, 0.0005],
    "Ku":   [700.0, 700.0, 700.0, 700.0, 700.0],
    "u_max": [600.0, 800.0, 800.0, 600.0, 600.0],
    "Gff":   [885.0, 632.0, 632.0, 885.0, 885.0],
}


class FuzzyGuiNode(Node):
    def __init__(self):
        super().__init__("fuzzy_gui")
        self.declare_parameter("setpoint_topic", "/rx150/fuzzy/setpoint")
        self.declare_parameter("target_node", "/rx150/fuzzy_node")
        self.declare_parameter("publish_rate", 20.0)
        self.joints = list(DEFAULT_JOINTS)
        self.pose = list(DEFAULT_POSE)

        self.topic = self.get_parameter("setpoint_topic").value
        self.target_node = self.get_parameter("target_node").value
        self.rate = float(self.get_parameter("publish_rate").value)

        self.pub = self.create_publisher(Float64MultiArray, self.topic, 10)
        # Parameter SERVICE client (raw) -> gọi đúng on_set_parameters_callback trong fuzzy_node.
        # Dùng service chuẩn set_parameters/get_parameters (luôn có ở mọi node) để không phụ
        # thuộc module rclpy.parameter_client (chỉ có từ Iron trở đi).
        self.cli_set = self.create_client(SetParameters, f"{self.target_node}/set_parameters")
        self.cli_get = self.create_client(GetParameters, f"{self.target_node}/get_parameters")
        self.get_logger().info(
            f"setpoint -> {self.topic} | gain svc -> {self.target_node}/[set|get]_parameters")

    def make_set_params_request(self, names_vals):
        req = SetParameters.Request()
        for nm, vals in names_vals.items():
            p = ParamMsg()
            p.name = nm
            p.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
            p.value.double_array_value = [float(v) for v in vals]
            req.parameters.append(p)
        return req

    def publish(self, values):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in values]
        self.pub.publish(msg)

    def param_client_ready(self):
        return self.cli_set.service_is_ready()


class FuzzyGuiApp:
    def __init__(self, node):
        self.node = node
        self.nj = len(node.joints)
        self.last_pub = None

        self.root = tk.Tk()
        self.root.title("rx150 fuzzy tuning")
        nb = ttk.Notebook(self.root)
        nb.pack(fill="both", expand=True, padx=6, pady=6)

        self.setpoint_vars, self.val_labels = [], []
        self._build_setpoint_tab(nb)
        self.gain_vars = {}   # name -> [DoubleVar x nj]
        self._build_gains_tab(nb)

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._period_ms = max(10, int(1000.0 / max(node.rate, 1.0)))
        self._tick()

    # ---------- Tab Setpoint ----------
    def _build_setpoint_tab(self, nb):
        f = ttk.Frame(nb); nb.add(f, text="Setpoint")
        ttk.Label(f, text="Kéo slider đặt setpoint (rad). Auto-publish khi đổi.",
                  foreground="#555").grid(row=0, column=0, columnspan=3, padx=8, pady=(8, 2), sticky="w")
        for i, jname in enumerate(self.node.joints):
            ttk.Label(f, text=jname, width=12).grid(row=i + 1, column=0, padx=(8, 2), pady=2, sticky="e")
            sv = tk.DoubleVar(value=self.node.pose[i])
            s = ttk.Scale(f, from_=-DEFAULT_RANGE, to=DEFAULT_RANGE, orient="horizontal",
                          variable=sv, length=340,
                          command=lambda _v, idx=i: self._sp_drag(idx))
            s.grid(row=i + 1, column=1, padx=2, pady=2, sticky="ew")
            lbl = ttk.Label(f, text=self._fmt(self.node.pose[i]), width=18)
            lbl.grid(row=i + 1, column=2, padx=(2, 8), pady=2, sticky="w")
            self.setpoint_vars.append(sv); self.val_labels.append(lbl)
            s.bind("<ButtonRelease-1>", lambda _e: self._sp_force())
        btn = ttk.Frame(f); btn.grid(row=self.nj + 1, column=0, columnspan=3, padx=8, pady=8)
        ttk.Button(btn, text="Publish now", command=self._sp_force).grid(row=0, column=0, padx=4)
        ttk.Button(btn, text="Zero", command=self._sp_zero).grid(row=0, column=1, padx=4)
        ttk.Button(btn, text="Sleep pose", command=self._sp_sleep).grid(row=0, column=2, padx=4)
        self.sp_status = ttk.Label(f, text="ready", foreground="#0a7")
        self.sp_status.grid(row=self.nj + 2, column=0, columnspan=3, sticky="w", padx=8, pady=(0, 8))

    @staticmethod
    def _fmt(rad):
        return f"{rad:+.3f} rad ({math.degrees(rad):+6.1f}°)"

    def _sp_drag(self, idx):
        self.val_labels[idx].config(text=self._fmt(self.setpoint_vars[idx].get()))

    def _sp_values(self):
        return [s.get() for s in self.setpoint_vars]

    def _sp_maybe(self):
        vals = self._sp_values()
        if self.last_pub is None or any(abs(a - b) > 1e-3 for a, b in zip(vals, self.last_pub)):
            self.node.publish(vals); self.last_pub = list(vals)
            self.sp_status.config(text="published [" + ", ".join(f"{v:+.2f}" for v in vals) + "]")

    def _sp_force(self):
        vals = self._sp_values(); self.node.publish(vals); self.last_pub = list(vals)
        self.sp_status.config(text="published [" + ", ".join(f"{v:+.2f}" for v in vals) + "]")

    def _sp_zero(self):
        for sv in self.setpoint_vars: sv.set(0.0)
        for i in range(self.nj): self._sp_drag(i)
        self._sp_force()

    def _sp_sleep(self):
        for i, sv in enumerate(self.setpoint_vars): sv.set(self.node.pose[i])
        for i in range(self.nj): self._sp_drag(i)
        self._sp_force()

    # ---------- Tab Gains ----------
    def _build_gains_tab(self, nb):
        f = ttk.Frame(nb); nb.add(f, text="Gains")
        ttk.Label(f, text="Sửa ô rồi Enter (hoặc 'Áp dụng hết') -> set_parameters lên "
                  f"{self.node.target_node}. u_max là nắp an toàn (≤885).",
                  foreground="#555").grid(row=0, column=0, columnspan=self.nj + 3, padx=8, pady=(8, 4), sticky="w")
        # Header cột: tên khớp
        ttk.Label(f, text="").grid(row=1, column=0)
        for j, jname in enumerate(self.node.joints):
            ttk.Label(f, text=jname, width=9, anchor="center").grid(row=1, column=1 + j, padx=2)
        ttk.Label(f, text="", width=10).grid(row=1, column=1 + self.nj)

        for r, gname in enumerate(GAIN_ORDER):
            ttk.Label(f, text=gname, width=8).grid(row=2 + r, column=0, padx=(8, 2), pady=2, sticky="e")
            row_vars = []
            for j in range(self.nj):
                sv = tk.StringVar(value=self._numfmt(GAIN_DEFAULTS[gname][j]))
                e = ttk.Entry(f, textvariable=sv, width=9, justify="center")
                e.grid(row=2 + r, column=1 + j, padx=2, pady=2)
                # Enter trong ô -> áp dụng cả hàng gain đó.
                e.bind("<Return>", lambda _e, nm=gname: self._gain_apply_one(nm))
                row_vars.append(sv)
            self.gain_vars[gname] = row_vars
            ttk.Button(f, text="Áp dụng", width=8,
                       command=lambda nm=gname: self._gain_apply_one(nm)).grid(row=2 + r, column=1 + self.nj, padx=4)
            ttk.Button(f, text="≈ đều", width=5,
                       command=lambda nm=gname: self._gain_uniform(nm)).grid(row=2 + r, column=2 + self.nj, padx=2)

        bf = ttk.Frame(f); bf.grid(row=2 + len(GAIN_ORDER), column=0, columnspan=self.nj + 3, padx=8, pady=10)
        ttk.Button(bf, text="Áp dụng hết", command=self._gain_apply_all).grid(row=0, column=0, padx=4)
        ttk.Button(bf, text="Đọc hiện tại", command=self._gain_read).grid(row=0, column=1, padx=4)
        ttk.Button(bf, text="Khôi phục mặc định", command=self._gain_defaults).grid(row=0, column=2, padx=4)
        ttk.Button(bf, text="Lưu vào yaml", command=self._gain_save).grid(row=0, column=3, padx=4)
        self.gain_status = ttk.Label(f, text="ready", foreground="#0a7")
        self.gain_status.grid(row=3 + len(GAIN_ORDER), column=0, columnspan=self.nj + 3, sticky="w", padx=8)

    @staticmethod
    def _numfmt(x):
        return f"{x:g}"

    def _gain_status(self, msg, ok=True):
        self.gain_status.config(text=msg, foreground="#0a7" if ok else "#c33")

    def _parse_row(self, gname):
        out = []
        for j in range(self.nj):
            try:
                out.append(float(self.gain_vars[gname][j].get()))
            except ValueError:
                self._gain_status(f"{gname}[{self.node.joints[j]}] không phải số", ok=False)
                return None
        return out

    def _send_params(self, names_vals):
        if not self.node.param_client_ready():
            self._gain_status(f"{self.node.target_node} không sẵn sàng (fuzzy_node đang chạy?)", ok=False)
            return
        req = self.node.make_set_params_request(names_vals)
        self.node.cli_set.call_async(req)
        self._gain_status("đã gửi: " + ", ".join(names_vals.keys()))

    def _gain_apply_one(self, gname):
        vals = self._parse_row(gname)
        if vals is None: return
        self._send_params({gname: vals})

    def _gain_apply_all(self):
        nv = {}
        for gname in GAIN_ORDER:
            vals = self._parse_row(gname)
            if vals is None: return
            nv[gname] = vals
        self._send_params(nv)

    def _gain_uniform(self, gname):
        v = simpledialog.askfloat("Đồng nhất", f"Đặt cả 5 khớp của {gname} = ", parent=self.root)
        if v is None: return
        for sv in self.gain_vars[gname]: sv.set(self._numfmt(v))
        self._gain_apply_one(gname)

    def _gain_read(self):
        if not self.node.param_client_ready():
            self._gain_status(f"{self.node.target_node} không sẵn sàng", ok=False); return
        req = GetParameters.Request()
        req.names = GAIN_ORDER
        fut = self.node.cli_get.call_async(req)
        fut.add_done_callback(self._on_params_recv)

    def _on_params_recv(self, fut):
        try:
            resp = fut.result()  # GetParameters.Response: .values = list[ParameterValue]
        except Exception as ex:
            self._gain_status(f"lỗi đọc: {ex}", ok=False); return
        for idx, gname in enumerate(GAIN_ORDER):
            pv = resp.values[idx] if idx < len(resp.values) else None
            if pv is None:
                continue
            if pv.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
                arr = list(pv.double_array_value)
                for j in range(min(self.nj, len(arr))):
                    self.gain_vars[gname][j].set(self._numfmt(arr[j]))
        self._gain_status("đã đọc giá trị hiện tại")

    def _gain_defaults(self):
        for gname in GAIN_ORDER:
            for j in range(self.nj):
                self.gain_vars[gname][j].set(self._numfmt(GAIN_DEFAULTS[gname][j]))
        self._gain_status("đã khôi phục mặc định (chưa gửi)")

    def _yaml_num(self, v):
        # Luôn giữ dấu chấm để yaml suy luận DOUBLE_ARRAY (tránh int array).
        s = f"{float(v):g}"
        if "." not in s and "e" not in s.lower() and "inf" not in s and "nan" not in s:
            s += ".0"
        return s

    def _gain_save(self):
        # Ghi giá trị đang hiển thị trong các ô vào fuzzy_gains.yaml (hiệu lực lần relaunch sau).
        # Bấm "Đọc hiện tại" trước nếu muốn lưu đúng giá trị node đang chạy.
        import os
        import re
        nv = {}
        for gname in GAIN_ORDER:
            vals = self._parse_row(gname)
            if vals is None:
                return
            nv[gname] = vals
        try:
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(get_package_share_directory("fuzzy_controller"),
                                "config", "fuzzy_gains.yaml")
        except Exception as ex:
            self._gain_status(f"không tìm thấy package: {ex}", ok=False); return
        try:
            with open(path) as fh:
                text = fh.read()
        except Exception as ex:
            self._gain_status(f"đọc yaml lỗi: {ex}", ok=False); return
        for gname, vals in nv.items():
            arr = "[" + ", ".join(self._yaml_num(v) for v in vals) + "]"
            text, n = re.subn(rf"^(\s*){gname}:\s*\[.*\]\s*$",
                              rf"\g<1>{gname}: {arr}", text, count=1, flags=re.MULTILINE)
            if n == 0:
                self._gain_status(f"không thấy dòng '{gname}:' trong yaml", ok=False); return
        try:
            with open(path, "w") as fh:
                fh.write(text)
        except Exception as ex:
            self._gain_status(f"ghi yaml lỗi: {ex}", ok=False); return
        self._gain_status(f"đã lưu vào fuzzy_gains.yaml (hiệu lực lần relaunch sau)")

    # ---------- Loop ----------
    def _tick(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self._sp_maybe()
        self.root.after(self._period_ms, self._tick)

    def _on_close(self):
        self.node.get_logger().info("GUI đóng.")
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init(args=sys.argv)
    node = FuzzyGuiNode()
    app = FuzzyGuiApp(node)
    try:
        app.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
