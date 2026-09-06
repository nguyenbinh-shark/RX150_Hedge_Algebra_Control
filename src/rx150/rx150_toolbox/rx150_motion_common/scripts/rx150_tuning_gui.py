#!/usr/bin/env python3
"""
rx150_tuning_gui — GUI tune node controller (rx150) chọn bằng param target: 3 tab.

Tab "Setpoint": 5 slider đặt vị trí đích (rad) -> publish /rx150/{target}/setpoint.
Tab "Gains":    lưới gain (Ke/Ked/Ku/u_max + Gff/gravity_sign nếu fuzzy,
                Kv/Ka nếu ff, a/b/c scalar + error_limit/error_dot_limit/u_max/Gff
                nếu HAC) × 5 khớp -> set_parameters trên /rx150/{target}_node
                (live-gain, không relaunch).
Tab "Plots":    live plot (q/q_ref, error, PWM tách gravity/friction/phần điều
                khiển) — cảm nhận tức thời khi kéo gain, không cần tool ngoài.

Parameters:
  target:         'fuzzy' (default), 'hac' hoặc 'ff' → chọn controller để tune
  setpoint_topic: topic setpoint (default từ target)
  target_node:    node set param (default từ target)
  publish_rate:   tần suất publish setpoint (default 20 Hz)

Chạy:
  ros2 run rx150_motion_common rx150_tuning_gui.py                → tune fuzzy_node
  ros2 run rx150_motion_common rx150_tuning_gui.py --ros-args -p target:=hac
                                                                  → tune hac_node
  ros2 run rx150_motion_common rx150_tuning_gui.py --ros-args -p target:=ff
                                                                  → tune ff_node

Lưu ý: KHÔNG qua MoveIt (MoveIt cần POSITION mode, xung đột PWM mode của controller).
"""

import sys
import math
import time
import collections
import tkinter as tk
from tkinter import ttk, simpledialog

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterValue, ParameterType
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# Khớp trong gains yaml.
DEFAULT_JOINTS = ["waist", "shoulder", "elbow", "wrist_angle", "wrist_rotate"]
DEFAULT_POSE = [0.0, -1.80, 1.55, 0.8, 0.0]
DEFAULT_RANGE = math.pi

# Gains chung cho cả fuzzy và ff.
COMMON_GAIN_ORDER = ["Ke", "Ked", "Ku", "u_max"]
COMMON_GAIN_DEFAULTS = {
    "Ke":   [0.2, 0.2, 0.2, 0.2, 0.2],
    "Ked":  [0.0005, 0.0005, 0.0005, 0.0005, 0.0005],
    "Ku":   [700.0, 700.0, 700.0, 700.0, 700.0],
    "u_max": [600.0, 800.0, 800.0, 600.0, 600.0],
}

# Gains riêng fuzzy (bù trọng lực).
FUZZY_GAIN_ORDER = COMMON_GAIN_ORDER + ["Gff", "gravity_sign"]
FUZZY_GAIN_DEFAULTS = {
    **COMMON_GAIN_DEFAULTS,
    "Gff":   [885.0, 632.0, 632.0, 885.0, 885.0],
    "gravity_sign": [1.0, 1.0, 1.0, 1.0, 1.0],
}

# HAC: a/b/c là hệ số mặt bão hòa CỦA HAC (scalar — KHÔNG per-joint), khác hẳn
# Ke/Ked/Ku của fuzzy. hac_node không khai báo Ke/Ked/Ku nên gửi các tên đó bị
# rclcpp reject ngầm (BUG cũ: GUI không đọc response nên không ai thấy reject).
# error_limit/error_dot_limit/u_max/Gff là per-joint, giống fuzzy.
HAC_SCALAR_GAIN_ORDER = ["a", "b", "c", "friction_eps"]
HAC_SCALAR_GAIN_DEFAULTS = {"a": 0.3, "b": 12.0, "c": 1200.0, "friction_eps": 0.05}
HAC_ARRAY_GAIN_ORDER = ["error_limit", "error_dot_limit", "u_max", "Gff",
                        "friction_coulomb", "friction_viscous"]
HAC_ARRAY_GAIN_DEFAULTS = {
    "error_limit": [0.5, 0.333, 0.333, 0.2, 0.5],
    "error_dot_limit": [3.14, 3.14, 3.14, 3.14, 3.14],
    "u_max": [600.0, 600.0, 600.0, 600.0, 600.0],
    "Gff": [150.0, 255.0, 255.0, 500.0, 200.0],
    "friction_coulomb": [0.0, 0.0, 0.0, 0.0, 0.0],
    "friction_viscous": [0.0, 0.0, 0.0, 0.0, 0.0],
}

# Gains riêng ff (feedforward vel/acc).
FF_GAIN_ORDER = COMMON_GAIN_ORDER + ["Kv", "Ka"]
FF_GAIN_DEFAULTS = {
    **COMMON_GAIN_DEFAULTS,
    "Kv": [0.0, 0.0, 0.0, 0.0, 0.0],
    "Ka": [0.0, 0.0, 0.0, 0.0, 0.0],
}

# Topic/plot: mỗi ring buffer giữ ~10s dữ liệu ở giả định 100 Hz (joint_states).
PLOT_BUFFER_LEN = 1000
PLOT_REDRAW_PERIOD_S = 0.1  # 10 Hz


class Rx150TuningGuiNode(Node):
    def __init__(self):
        # Node phải được khởi tạo trước khi declare/get parameter. Tên node cố
        # định; controller đích vẫn được phân biệt bằng parameter `target`.
        super().__init__("rx150_tuning_gui")

        # ---------- parameters ----------
        self.declare_parameter("target", "fuzzy")
        self.declare_parameter("setpoint_topic", "")
        self.declare_parameter("target_node", "")
        self.declare_parameter("publish_rate", 20.0)

        target = self.get_parameter("target").value
        if target not in ("fuzzy", "hac", "ff"):
            self.get_logger().error(
                f"target phải là 'fuzzy', 'hac' hoặc 'ff', nhận: '{target}'")
            raise ValueError(f"Invalid target: {target}")
        self.target = target

        # Nếu setpoint_topic trống, suy ra từ target.
        setpoint_topic = self.get_parameter("setpoint_topic").value
        if not setpoint_topic:
            setpoint_topic = f"/rx150/{target}/setpoint"
            self.set_parameters([Parameter("setpoint_topic", value=setpoint_topic)])

        # Nếu target_node trống, suy ra từ target.
        target_node = self.get_parameter("target_node").value
        if not target_node:
            target_node = f"/rx150/{target}_node"
            self.set_parameters([Parameter("target_node", value=target_node)])

        self.rate = float(self.get_parameter("publish_rate").value)

        # Chọn gain scalar/array theo target. a/b/c (HAC) là scalar; mọi gain
        # khác là mảng 5 phần tử (1 / khớp).
        if target == "fuzzy":
            self.scalar_gain_order = []
            self.scalar_gain_defaults = {}
            self.array_gain_order = FUZZY_GAIN_ORDER
            self.array_gain_defaults = FUZZY_GAIN_DEFAULTS
            self.config_package = "rx150_fuzzy_controller"
            self.config_file = "rx150_fuzzy_gains.yaml"
        elif target == "hac":
            self.scalar_gain_order = HAC_SCALAR_GAIN_ORDER
            self.scalar_gain_defaults = HAC_SCALAR_GAIN_DEFAULTS
            self.array_gain_order = HAC_ARRAY_GAIN_ORDER
            self.array_gain_defaults = HAC_ARRAY_GAIN_DEFAULTS
            self.config_package = "rx150_hac_controller"
            self.config_file = "rx150_hac_gains.yaml"
        else:  # ff
            self.scalar_gain_order = []
            self.scalar_gain_defaults = {}
            self.array_gain_order = FF_GAIN_ORDER
            self.array_gain_defaults = FF_GAIN_DEFAULTS
            self.config_package = "rx150_ff_controller"
            self.config_file = "rx150_ff_gains.yaml"

        # gộp scalar+array — dùng cho get_parameters (nút "Đọc hiện tại").
        self.gain_order = self.scalar_gain_order + self.array_gain_order

        self.joints = list(DEFAULT_JOINTS)
        self.pose = list(DEFAULT_POSE)

        self.topic = setpoint_topic
        self.target_node = target_node

        self.pub = self.create_publisher(Float64MultiArray, self.topic, 10)
        self.cli_set = self.create_client(SetParameters, f"{self.target_node}/set_parameters")
        self.cli_get = self.create_client(GetParameters, f"{self.target_node}/get_parameters")
        self.get_logger().info(
            f"Target={target}, setpoint -> {self.topic} | gain svc -> {self.target_node}/[set|get]_parameters")

        self._setup_plot_subscriptions()

    # ---------- Plot tab: telemetry ----------
    def _setup_plot_subscriptions(self):
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.plot_buffers = {
            j: {
                "t": collections.deque(maxlen=PLOT_BUFFER_LEN),
                "q": collections.deque(maxlen=PLOT_BUFFER_LEN),
                "q_ref": collections.deque(maxlen=PLOT_BUFFER_LEN),
                "err": collections.deque(maxlen=PLOT_BUFFER_LEN),
                "pwm": collections.deque(maxlen=PLOT_BUFFER_LEN),
                "grav": collections.deque(maxlen=PLOT_BUFFER_LEN),
                "fric": collections.deque(maxlen=PLOT_BUFFER_LEN),
            }
            for j in self.joints
        }
        self._plot_latest = {}
        self._plot_t0 = None

        self._plot_sub_js = self.create_subscription(
            JointState, "/rx150/joint_states", self._on_js_for_plot, sensor_qos)

        plot_topics = {
            "ref": f"/rx150/{self.target}/reference",
            "err": f"/rx150/{self.target}/error",
            "eff": f"/rx150/{self.target}/effort",
            "grav": f"/rx150/{self.target}/gravity",
            "fric": f"/rx150/{self.target}/friction",
        }
        for key, topic_name in plot_topics.items():
            self.create_subscription(
                JointState, topic_name,
                lambda msg, k=key: self._plot_latest.__setitem__(k, msg),
                sensor_qos)

    @staticmethod
    def _joint_value(msg, joint, field, default=float("nan")):
        if msg is None:
            return default
        try:
            idx = list(msg.name).index(joint)
        except ValueError:
            return default
        arr = getattr(msg, field)
        return arr[idx] if idx < len(arr) else default

    def _on_js_for_plot(self, msg):
        if self._plot_t0 is None:
            self._plot_t0 = time.monotonic()
        t = time.monotonic() - self._plot_t0

        ref_msg = self._plot_latest.get("ref")
        err_msg = self._plot_latest.get("err")
        eff_msg = self._plot_latest.get("eff")
        grav_msg = self._plot_latest.get("grav")
        fric_msg = self._plot_latest.get("fric")

        for j in self.joints:
            buf = self.plot_buffers[j]
            buf["t"].append(t)
            buf["q"].append(self._joint_value(msg, j, "position"))
            buf["q_ref"].append(self._joint_value(ref_msg, j, "position"))
            buf["err"].append(self._joint_value(err_msg, j, "position"))
            buf["pwm"].append(self._joint_value(eff_msg, j, "effort"))
            buf["grav"].append(self._joint_value(grav_msg, j, "effort"))
            buf["fric"].append(self._joint_value(fric_msg, j, "effort", default=0.0))

    # ---------- Gain param helpers ----------
    def make_set_params_request(self, names_vals):
        """names_vals: dict tên -> float (scalar) hoặc list[float] (mảng 5 khớp)."""
        req = SetParameters.Request()
        for nm, vals in names_vals.items():
            p = ParamMsg()
            p.name = nm
            if isinstance(vals, (list, tuple)):
                p.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
                p.value.double_array_value = [float(v) for v in vals]
            else:
                p.value.type = ParameterType.PARAMETER_DOUBLE
                p.value.double_value = float(vals)
            req.parameters.append(p)
        return req

    def publish(self, values):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in values]
        self.pub.publish(msg)

    def param_client_ready(self):
        return self.cli_set.service_is_ready()


class Rx150TuningGuiApp:
    def __init__(self, node):
        self.node = node
        self.nj = len(node.joints)
        self.last_pub = None

        self.root = tk.Tk()
        self.root.title(f"rx150 {node.get_parameter('target').value} tuning")
        nb = ttk.Notebook(self.root)
        nb.pack(fill="both", expand=True, padx=6, pady=6)

        self.setpoint_vars, self.val_labels = [], []
        self._build_setpoint_tab(nb)
        self.gain_vars = {}   # name -> DoubleVar (scalar) hoặc [StringVar x nj] (mảng)
        self._build_gains_tab(nb)
        self._build_plots_tab(nb)

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._period_ms = max(10, int(1000.0 / max(node.rate, 1.0)))
        self._last_redraw = 0.0
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

        row = 1
        # --- Scalar gains (HAC: a, b, c) — 1 giá trị chung, không per-joint ---
        if self.node.scalar_gain_order:
            ttk.Label(f, text="Scalar (mặt bão hòa HAC):", foreground="#555").grid(
                row=row, column=0, columnspan=3, padx=8, pady=(2, 2), sticky="w")
            row += 1
            for gname in self.node.scalar_gain_order:
                ttk.Label(f, text=gname, width=8).grid(row=row, column=0, padx=(8, 2), pady=2, sticky="e")
                sv = tk.StringVar(value=self._numfmt(self.node.scalar_gain_defaults[gname]))
                e = ttk.Entry(f, textvariable=sv, width=9, justify="center")
                e.grid(row=row, column=1, padx=2, pady=2, sticky="w")
                e.bind("<Return>", lambda _e, nm=gname: self._gain_apply_one(nm))
                self.gain_vars[gname] = sv
                ttk.Button(f, text="Áp dụng", width=8,
                           command=lambda nm=gname: self._gain_apply_one(nm)).grid(row=row, column=2, padx=4)
                row += 1
            row += 1

        # --- Array gains (per-joint) ---
        ttk.Label(f, text="Per-joint:", foreground="#555").grid(
            row=row, column=0, columnspan=3, padx=8, pady=(2, 2), sticky="w")
        row += 1
        header_row = row
        ttk.Label(f, text="").grid(row=header_row, column=0)
        for j, jname in enumerate(self.node.joints):
            ttk.Label(f, text=jname, width=9, anchor="center").grid(row=header_row, column=1 + j, padx=2)
        ttk.Label(f, text="", width=10).grid(row=header_row, column=1 + self.nj)
        row += 1

        for gname in self.node.array_gain_order:
            ttk.Label(f, text=gname, width=8).grid(row=row, column=0, padx=(8, 2), pady=2, sticky="e")
            row_vars = []
            for j in range(self.nj):
                sv = tk.StringVar(value=self._numfmt(self.node.array_gain_defaults[gname][j]))
                e = ttk.Entry(f, textvariable=sv, width=9, justify="center")
                e.grid(row=row, column=1 + j, padx=2, pady=2)
                e.bind("<Return>", lambda _e, nm=gname: self._gain_apply_one(nm))
                row_vars.append(sv)
            self.gain_vars[gname] = row_vars
            ttk.Button(f, text="Áp dụng", width=8,
                       command=lambda nm=gname: self._gain_apply_one(nm)).grid(row=row, column=1 + self.nj, padx=4)
            ttk.Button(f, text="≈ đều", width=5,
                       command=lambda nm=gname: self._gain_uniform(nm)).grid(row=row, column=2 + self.nj, padx=2)
            row += 1

        bf = ttk.Frame(f); bf.grid(row=row, column=0, columnspan=self.nj + 3, padx=8, pady=10)
        ttk.Button(bf, text="Áp dụng hết", command=self._gain_apply_all).grid(row=0, column=0, padx=4)
        ttk.Button(bf, text="Đọc hiện tại", command=self._gain_read).grid(row=0, column=1, padx=4)
        ttk.Button(bf, text="Khôi phục mặc định", command=self._gain_defaults).grid(row=0, column=2, padx=4)
        ttk.Button(bf, text="Lưu vào yaml", command=self._gain_save).grid(row=0, column=3, padx=4)
        row += 1
        self.gain_status = ttk.Label(f, text="ready", foreground="#0a7")
        self.gain_status.grid(row=row, column=0, columnspan=self.nj + 3, sticky="w", padx=8)

    @staticmethod
    def _numfmt(x):
        return f"{x:g}"

    def _gain_status(self, msg, ok=True):
        self.gain_status.config(text=msg, foreground="#0a7" if ok else "#c33")

    def _is_scalar(self, gname):
        return gname in self.node.scalar_gain_order

    def _parse_row(self, gname):
        if self._is_scalar(gname):
            try:
                return float(self.gain_vars[gname].get())
            except ValueError:
                self._gain_status(f"{gname} không phải số", ok=False)
                return None
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
            self._gain_status(f"{self.node.target_node} không sẵn sàng (controller đang chạy?)", ok=False)
            return
        req = self.node.make_set_params_request(names_vals)
        fut = self.node.cli_set.call_async(req)
        names = list(names_vals.keys())
        fut.add_done_callback(lambda f, n=names: self._on_set_params_done(f, n))
        self._gain_status("đang gửi: " + ", ".join(names))

    def _on_set_params_done(self, fut, names):
        try:
            resp = fut.result()
        except Exception as ex:
            self._gain_status(f"lỗi set_parameters: {ex}", ok=False)
            return
        failed = []
        for nm, res in zip(names, resp.results):
            if not res.successful:
                failed.append(f"{nm} ({res.reason or 'rejected'})")
        if failed:
            self._gain_status("BỊ TỪ CHỐI: " + "; ".join(failed), ok=False)
        else:
            self._gain_status("đã áp dụng: " + ", ".join(names))

    def _gain_apply_one(self, gname):
        vals = self._parse_row(gname)
        if vals is None: return
        self._send_params({gname: vals})

    def _gain_apply_all(self):
        nv = {}
        for gname in self.node.gain_order:
            vals = self._parse_row(gname)
            if vals is None: return
            nv[gname] = vals
        self._send_params(nv)

    def _gain_uniform(self, gname):
        if self._is_scalar(gname):
            return  # scalar không có khái niệm "đều" giữa các khớp
        v = simpledialog.askfloat("Đồng nhất", f"Đặt cả 5 khớp của {gname} = ", parent=self.root)
        if v is None: return
        for sv in self.gain_vars[gname]: sv.set(self._numfmt(v))
        self._gain_apply_one(gname)

    def _gain_read(self):
        if not self.node.param_client_ready():
            self._gain_status(f"{self.node.target_node} không sẵn sàng", ok=False); return
        req = GetParameters.Request()
        req.names = self.node.gain_order
        fut = self.node.cli_get.call_async(req)
        fut.add_done_callback(self._on_params_recv)

    def _on_params_recv(self, fut):
        try:
            resp = fut.result()
        except Exception as ex:
            self._gain_status(f"lỗi đọc: {ex}", ok=False); return
        for idx, gname in enumerate(self.node.gain_order):
            pv = resp.values[idx] if idx < len(resp.values) else None
            if pv is None:
                continue
            if pv.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
                arr = list(pv.double_array_value)
                for j in range(min(self.nj, len(arr))):
                    self.gain_vars[gname][j].set(self._numfmt(arr[j]))
            elif pv.type == ParameterType.PARAMETER_DOUBLE:
                if self._is_scalar(gname):
                    self.gain_vars[gname].set(self._numfmt(pv.double_value))
        self._gain_status("đã đọc giá trị hiện tại")

    def _gain_defaults(self):
        for gname in self.node.scalar_gain_order:
            self.gain_vars[gname].set(self._numfmt(self.node.scalar_gain_defaults[gname]))
        for gname in self.node.array_gain_order:
            for j in range(self.nj):
                self.gain_vars[gname][j].set(self._numfmt(self.node.array_gain_defaults[gname][j]))
        self._gain_status("đã khôi phục mặc định (chưa gửi)")

    def _yaml_num(self, v):
        s = f"{float(v):g}"
        if "." not in s and "e" not in s.lower() and "inf" not in s and "nan" not in s:
            s += ".0"
        return s

    def _find_src_yaml(self, install_path):
        """Tìm bản src/<pkg>/config/<file> tương ứng với install_path (share/<pkg>/config/<file>).

        Lưu vào install thì lần colcon build sau sẽ ghi đè mất — phải lưu vào src.
        """
        import os
        parts = install_path.replace(os.sep, "/").split("/")
        try:
            install_idx = parts.index("install")
        except ValueError:
            return None
        ws_root = "/".join(parts[:install_idx])
        pkg = self.node.config_package
        candidate = os.path.join(ws_root, "src", pkg, "config", self.node.config_file)
        if os.path.isfile(candidate):
            return candidate
        # Fallback: package lồng sâu hơn trong src/ (một số package interbotix).
        for root, dirs, _files in os.walk(os.path.join(ws_root, "src")):
            if os.path.basename(root) == pkg:
                cand2 = os.path.join(root, "config", self.node.config_file)
                if os.path.isfile(cand2):
                    return cand2
        return None

    def _gain_save(self):
        import os
        import re
        nv = {}
        for gname in self.node.gain_order:
            vals = self._parse_row(gname)
            if vals is None:
                return
            nv[gname] = vals
        try:
            from ament_index_python.packages import get_package_share_directory
            install_path = os.path.join(get_package_share_directory(self.node.config_package),
                                         "config", self.node.config_file)
        except Exception as ex:
            self._gain_status(f"không tìm thấy package: {ex}", ok=False); return

        path = self._find_src_yaml(install_path) or install_path
        try:
            with open(path) as fh:
                text = fh.read()
        except Exception as ex:
            self._gain_status(f"đọc yaml lỗi: {ex}", ok=False); return
        for gname, vals in nv.items():
            if self._is_scalar(gname):
                pattern = rf"^(\s*){gname}:\s*[^\s#]+(\s*(#.*)?)$"
                repl = rf"\g<1>{gname}: {self._yaml_num(vals)}\2"
            else:
                arr = "[" + ", ".join(self._yaml_num(v) for v in vals) + "]"
                pattern = rf"^(\s*){gname}:\s*\[.*?\](\s*(#.*)?)$"
                repl = rf"\g<1>{gname}: {arr}\2"
            text, n = re.subn(pattern, repl, text, count=1, flags=re.MULTILINE)
            if n == 0:
                self._gain_status(f"không thấy dòng '{gname}:' trong yaml", ok=False); return
        try:
            with open(path, "w") as fh:
                fh.write(text)
        except Exception as ex:
            self._gain_status(f"ghi yaml lỗi: {ex}", ok=False); return
        src_note = " (src)" if path != install_path else " (CHỈ install — không tìm thấy src!)"
        self._gain_status(f"đã lưu vào {path}{src_note} — cần colcon build + relaunch để có hiệu lực")

    # ---------- Tab Plots ----------
    def _build_plots_tab(self, nb):
        f = ttk.Frame(nb); nb.add(f, text="Plots")
        ttk.Label(f, text=f"Live telemetry từ /rx150/{self.node.target}/* (10 Hz redraw, buffer ~10s).",
                  foreground="#555").pack(anchor="w", padx=8, pady=(6, 2))

        self.fig = Figure(figsize=(10, 9.5), dpi=90)
        self.plot_axes = {}   # joint -> (ax_pos, ax_err, ax_pwm)
        self.plot_lines = {}  # joint -> dict of Line2D

        n = self.nj
        for i, j in enumerate(self.node.joints):
            ax_pos = self.fig.add_subplot(n, 3, 3 * i + 1)
            ax_err = self.fig.add_subplot(n, 3, 3 * i + 2)
            ax_pwm = self.fig.add_subplot(n, 3, 3 * i + 3)

            (l_q,) = ax_pos.plot([], [], color="#3498db", linewidth=0.9, label="q")
            (l_qref,) = ax_pos.plot([], [], "--", color="gray", linewidth=0.9, label="q_ref")
            ax_pos.set_ylabel(j, fontsize=8)
            if i == 0:
                ax_pos.set_title("q vs q_ref (rad)", fontsize=9)
                ax_err.set_title("error (rad)", fontsize=9)
                ax_pwm.set_title("PWM: total/grav/fric/ctrl", fontsize=9)
                ax_pos.legend(fontsize=6, loc="upper right")

            (l_err,) = ax_err.plot([], [], color="#e74c3c", linewidth=0.9)
            ax_err.axhline(0.0, color="gray", linewidth=0.5, linestyle=":")

            (l_pwm,) = ax_pwm.plot([], [], color="#2c3e50", linewidth=0.8, label="total")
            (l_grav,) = ax_pwm.plot([], [], color="orange", linewidth=0.7, label="grav")
            (l_fric,) = ax_pwm.plot([], [], color="#9b59b6", linewidth=0.7, label="fric")
            (l_ctrl,) = ax_pwm.plot([], [], color="#2ecc71", linewidth=0.7, label="ctrl-only")
            ax_pwm.axhline(0.0, color="gray", linewidth=0.5, linestyle=":")
            if i == 0:
                ax_pwm.legend(fontsize=6, loc="upper right")

            for ax in (ax_pos, ax_err, ax_pwm):
                ax.tick_params(labelsize=6)
            if i == n - 1:
                ax_pos.set_xlabel("t (s)", fontsize=7)
                ax_err.set_xlabel("t (s)", fontsize=7)
                ax_pwm.set_xlabel("t (s)", fontsize=7)

            self.plot_axes[j] = (ax_pos, ax_err, ax_pwm)
            self.plot_lines[j] = {
                "q": l_q, "q_ref": l_qref, "err": l_err,
                "pwm": l_pwm, "grav": l_grav, "fric": l_fric, "ctrl": l_ctrl,
            }

        self.fig.tight_layout()
        self.canvas = FigureCanvasTkAgg(self.fig, master=f)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=4, pady=4)

    def _plot_redraw(self):
        for j in self.node.joints:
            buf = self.node.plot_buffers[j]
            t = list(buf["t"])
            if not t:
                continue
            lines = self.plot_lines[j]
            ax_pos, ax_err, ax_pwm = self.plot_axes[j]

            q = list(buf["q"]); q_ref = list(buf["q_ref"])
            lines["q"].set_data(t, q)
            lines["q_ref"].set_data(t, q_ref)
            ax_pos.relim(); ax_pos.autoscale_view()

            err = list(buf["err"])
            lines["err"].set_data(t, err)
            ax_err.relim(); ax_err.autoscale_view()

            pwm = list(buf["pwm"]); grav = list(buf["grav"]); fric = list(buf["fric"])
            ctrl = [
                (p - g - fr) if (p == p and g == g and fr == fr) else float("nan")
                for p, g, fr in zip(pwm, grav, fric)
            ]
            lines["pwm"].set_data(t, pwm)
            lines["grav"].set_data(t, grav)
            lines["fric"].set_data(t, fric)
            lines["ctrl"].set_data(t, ctrl)
            ax_pwm.relim(); ax_pwm.autoscale_view()

        self.canvas.draw_idle()

    # ---------- Loop ----------
    def _tick(self):
        # Drain nhiều spin_once mỗi tick để không tụt backlog telemetry 100Hz
        # trong khi GUI redraw ở publish_rate (mặc định 20Hz) hoặc chậm hơn.
        for _ in range(20):
            if rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.0)
            else:
                break
        self._sp_maybe()

        now = time.monotonic()
        if now - self._last_redraw >= PLOT_REDRAW_PERIOD_S:
            self._last_redraw = now
            self._plot_redraw()

        self.root.after(self._period_ms, self._tick)

    def _on_close(self):
        self.node.get_logger().info("GUI đóng.")
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init(args=sys.argv)
    node = Rx150TuningGuiNode()
    app = Rx150TuningGuiApp(node)
    try:
        app.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
