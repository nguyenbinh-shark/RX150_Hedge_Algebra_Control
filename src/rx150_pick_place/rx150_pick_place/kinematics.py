#!/usr/bin/env python3
"""
kinematics — IK/FK giải tích cho rx150 (5-DoF: x, y, z, pitch + roll tự do).

TẠI SAO KHÔNG DÙNG bot.arm.set_ee_pose_components(execute=False) LÀM IK-ORACLE:
  1. SDK gọi mr.IKinSpace (Newton lặp) với 3 seed cố định → FAIL ngẫu nhiên ở các
     pose hoàn toàn với tới được (đã đo: pitch=90° tại (0.25, 0, 0.15) fail).
  2. _check_joint_limits() của SDK còn kiểm tra GIỚI HẠN VẬN TỐC dựa trên
     self.joint_commands / self.moving_time. Khi execute=False, joint_commands
     KHÔNG cập nhật (đứng ở pose SDK ra lệnh lần cuối) → nghiệm hợp lệ về hình học
     bị loại vì "quá nhanh". Đây là false-negative kinh điển: log ra "IK fail"
     nhưng thực ra pose với tới được.
  3. Không chọn được nhánh nghiệm → APPROACH và DESCEND có thể ra 2 cấu hình
     khuỷu khác nhau ⇒ tay lật khuỷu ngay trên đầu vật.

Ở đây dùng nghiệm ĐÓNG (2R + wrist) nên: xác định, nhanh, trả VỀ MỌI nhánh
(elbow-up/elbow-down) và chọn nhánh gần seed nhất ⇒ liên tục theo chuỗi bước.

QUY ƯỚC (đã kiểm chứng bằng mr.FKinSpace — xem test/test_kinematics.py):
  waist = atan2(y, x)                      (5-DoF: yaw bị ép theo hướng bán kính)
  pitch = shoulder + elbow + wrist_angle   (pitch > 0 = ee_x CHÚC XUỐNG)
  pitch = +pi/2 → ee_x xuống thẳng (gắp từ trên), ee_z nằm ngang
  pitch = 0     → ee_x nằm ngang,  ee_z THẲNG ĐỨNG
  Trục ee_z (= trục vật kẹp ngang giữa 2 ngón) có phương vị = waist − wrist_rotate
  ⇒ muốn trục vật hướng theo yaw:  wrist_rotate = waist − yaw   (KHÔNG phải yaw − waist)
"""
import math

# Giới hạn khớp (rx150.urdf.xacro:26-49). pi_offset 1e-5 đã trừ ở waist/wrist_rotate.
JOINT_LIMITS = {
    'waist': (-math.pi + 1e-5, math.pi - 1e-5),
    'shoulder': (math.radians(-106.0), math.radians(100.0)),
    'elbow': (math.radians(-102.0), math.radians(95.0)),
    'wrist_angle': (math.radians(-100.0), math.radians(123.0)),
    'wrist_rotate': (-math.pi + 1e-5, math.pi - 1e-5),
}
ARM_JOINTS = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']

# Hình học mặc định (m) — lấy từ mr_descriptions.rx150 (Slist/M). Chỉ dùng khi
# không import được SDK (ví dụ chạy pytest trên máy không có interbotix).
_FALLBACK_GEOM = {
    'shoulder': (0.0, 0.10457),
    'elbow': (0.05, 0.25457),
    'wrist': (0.2, 0.25457),
    'ee': (0.35855, 0.25457),
}


def _geometry_from_sdk(robot_model):
    """Trích (r, z) của shoulder/elbow/wrist/ee từ Slist+M của mr_descriptions.

    Với khớp quay quanh ω=(0,1,0): v = −ω×q ⇒ q = (v_z, ·, −v_x)
    (Slist[3:6, j] = v, Slist[0:3, j] = ω).
    """
    from interbotix_xs_modules.xs_robot import mr_descriptions as mrd
    des = getattr(mrd, robot_model)
    s = des.Slist
    return {
        'shoulder': (float(s[5, 1]), float(-s[3, 1])),
        'elbow': (float(s[5, 2]), float(-s[3, 2])),
        'wrist': (float(s[5, 3]), float(-s[3, 3])),
        'ee': (float(des.M[0, 3]), float(des.M[2, 3])),
    }


def wrap(a):
    """Đưa góc về (−pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def wrist_rotate_for_axis(waist, axis_yaw, pitch=math.pi / 2):
    """wrist_rotate để PHƯƠNG KẸP (trục ee_y) vuông góc trục vật.

    Vật hình ống nằm ngang, trục có phương vị axis_yaw. Điều kiện kẹp đúng là
    ee_y ⟂ trục vật (hai ngón ép vào hai bên thân ống). Với
    R_ee = Rz(waist)·Ry(pitch)·Rx(roll):

        ee_y · u = sin(roll)·sin(pitch)·cos(waist − yaw) + cos(roll)·sin(waist − yaw)

    cho  tan(roll) = tan(waist − yaw) / sin(pitch).

    pitch = pi/2 (chúc thẳng xuống) ⇒ roll = waist − yaw (kết quả quen thuộc).
    pitch nhỏ ⇒ phải xoay ngón nhiều hơn mới ép được vào thân ống.
    Trục ống đối xứng mod pi ⇒ chọn nghiệm gần 0 nhất (ngón đi ít nhất, tránh
    giới hạn ±pi của wrist_rotate).
    """
    delta = wrap(waist - axis_yaw)
    sp = math.sin(pitch)
    if abs(sp) < 1e-6:
        # ee_x nằm ngang: trục ee_z thẳng đứng, không kẹp được ống nằm ngang
        # theo cách này — trả nghiệm biên để caller thấy pitch chọn sai.
        base = math.copysign(math.pi / 2, math.sin(delta) or 1.0)
    else:
        base = math.atan2(math.sin(delta), math.cos(delta) * sp)
    base = wrap(base)
    alt = wrap(base + math.pi)
    return base if abs(base) <= abs(alt) else alt


class Rx150Kinematics:
    """IK/FK giải tích 5-DoF. Không cần ROS, không cần robot đang chạy."""

    def __init__(self, robot_model='rx150', limit_margin=0.02, geometry=None):
        if geometry is None:
            try:
                geometry = _geometry_from_sdk(robot_model)
            except Exception:            # noqa: BLE001 — chạy offline/pytest
                geometry = _FALLBACK_GEOM
        sh, el, wr, ee = (geometry['shoulder'], geometry['elbow'],
                          geometry['wrist'], geometry['ee'])
        self.shoulder = sh
        # Mỗi link: (độ dài, phương vị trong mặt phẳng r-z khi mọi khớp = 0)
        self.l1, self.a1 = self._link(sh, el)
        self.l2, self.a2 = self._link(el, wr)
        self.l3, self.a3 = self._link(wr, ee)
        self.max_reach = self.l1 + self.l2 + self.l3
        self.limit_margin = float(limit_margin)
        self.limits = {
            k: (lo + self.limit_margin, hi - self.limit_margin)
            for k, (lo, hi) in JOINT_LIMITS.items()
        }

    @staticmethod
    def _link(p_from, p_to):
        dr, dz = p_to[0] - p_from[0], p_to[1] - p_from[1]
        return math.hypot(dr, dz), math.atan2(dz, dr)

    # ── kiểm tra giới hạn ────────────────────────────────────────────────
    def in_limits(self, joints):
        return all(self.limits[n][0] <= v <= self.limits[n][1]
                   for n, v in zip(ARM_JOINTS, joints))

    def limit_violation(self, joints):
        """Trả tên khớp + giá trị vi phạm đầu tiên (để log lỗi có ích), hoặc None."""
        for n, v in zip(ARM_JOINTS, joints):
            lo, hi = self.limits[n]
            if not lo <= v <= hi:
                return (f'{n}={math.degrees(v):.1f}° ngoài '
                        f'[{math.degrees(lo):.1f},{math.degrees(hi):.1f}]°')
        return None

    # ── FK ──────────────────────────────────────────────────────────────
    def fk(self, joints):
        """joints[5] → (x, y, z, pitch). Dùng để verify pose thực tế đạt được."""
        waist, s, e, w = joints[0], joints[1], joints[2], joints[3]
        pitch = s + e + w
        r = self.shoulder[0] + self.l1 * math.cos(self.a1 - s)
        z = self.shoulder[1] + self.l1 * math.sin(self.a1 - s)
        r += self.l2 * math.cos(self.a2 - (s + e))
        z += self.l2 * math.sin(self.a2 - (s + e))
        r += self.l3 * math.cos(self.a3 - pitch)
        z += self.l3 * math.sin(self.a3 - pitch)
        return r * math.cos(waist), r * math.sin(waist), z, pitch

    # ── IK ──────────────────────────────────────────────────────────────
    def ik_all(self, x, y, z, pitch, wrist_rotate=0.0):
        """Mọi nghiệm (elbow-up / elbow-down) thoả giới hạn khớp. [] nếu không có."""
        r = math.hypot(x, y)
        waist = math.atan2(y, x) if r > 1e-6 else 0.0
        # ngược từ ee về tâm wrist (khâu cuối quay theo pitch)
        a3 = self.a3 - pitch
        rw = r - self.l3 * math.cos(a3)
        zw = z - self.l3 * math.sin(a3)
        dr, dz = rw - self.shoulder[0], zw - self.shoulder[1]
        d = math.hypot(dr, dz)
        if d > self.l1 + self.l2 or d < abs(self.l1 - self.l2) or d < 1e-9:
            return []
        cos_i = max(-1.0, min(1.0,
                              (d * d - self.l1 ** 2 - self.l2 ** 2) / (2 * self.l1 * self.l2)))
        psi = math.atan2(dz, dr)
        out = []
        for sign in (-1.0, 1.0):
            iota = sign * math.acos(cos_i)
            beta = math.atan2(self.l2 * math.sin(iota),
                              self.l1 + self.l2 * math.cos(iota))
            alpha1 = psi - beta
            s = self.a1 - alpha1
            e = self.a2 - s - alpha1 - iota
            w = pitch - s - e
            joints = [waist, s, e, w, wrap(wrist_rotate)]
            if self.in_limits(joints):
                out.append(joints)
            if abs(iota) < 1e-9:        # nghiệm suy biến (thẳng khuỷu): 1 nhánh
                break
        return out

    def ik(self, x, y, z, pitch, seed=None, wrist_rotate=0.0):
        """Nghiệm GẦN SEED NHẤT (liên tục theo chuỗi bước) hoặc None.

        seed = joints hiện tại của tay ⇒ không lật khuỷu giữa APPROACH và DESCEND.
        """
        sols = self.ik_all(x, y, z, pitch, wrist_rotate)
        if not sols:
            return None
        if seed is None:
            # không có seed: ưu tiên khuỷu "lên" (shoulder nhỏ hơn) — dáng tự nhiên
            return min(sols, key=lambda j: j[1])
        return min(sols, key=lambda j: sum((a - b) ** 2 for a, b in zip(j[1:4], seed[1:4])))

    def ik_ladder(self, x, y, z, pitch_ladder, seed=None, wrist_rotate=0.0):
        """Thử lần lượt các pitch trong ladder. Trả (joints, pitch) hoặc (None, None)."""
        for p in pitch_ladder:
            j = self.ik(x, y, z, float(p), seed=seed, wrist_rotate=wrist_rotate)
            if j is not None:
                return j, float(p)
        return None, None

    # ── chẩn đoán (dùng cho tool kiểm tra offline + log lỗi) ─────────────
    def max_pitch(self, x, y, z, lo_deg=-45.0, hi_deg=90.0, step_deg=1.0):
        """pitch LỚN NHẤT (chúc xuống nhất) với tới được tại (x,y,z), hoặc None.

        rx150 KHÔNG chúc thẳng đứng 90° được ở mọi nơi: hết tầm từ r ≈ 0.29 m.
        """
        pd = hi_deg
        while pd >= lo_deg:
            if self.ik_all(x, y, z, math.radians(pd)):
                return math.radians(pd)
            pd -= step_deg
        return None

    def reach_report(self, x, y, z, pitch):
        """Chuỗi mô tả lý do không với tới (đưa vào log thay vì chỉ 'IK fail')."""
        r = math.hypot(x, y)
        if r > self.max_reach:
            return f'r={r:.3f}m > tầm tối đa {self.max_reach:.3f}m'
        if self.ik_all(x, y, z, pitch):
            return 'OK'
        mp = self.max_pitch(x, y, z)
        if mp is None:
            return f'(x={x:.3f},y={y:.3f},z={z:.3f}) ngoài vùng làm việc với mọi pitch'
        return (f'pitch={math.degrees(pitch):.1f}° quá dốc tại r={r:.3f}m z={z:.3f}m '
                f'— tối đa {math.degrees(mp):.1f}°')
