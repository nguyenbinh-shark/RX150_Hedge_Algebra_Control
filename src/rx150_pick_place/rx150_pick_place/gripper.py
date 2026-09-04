#!/usr/bin/env python3
"""
gripper — đóng/mở gripper rx150 VÀ xác nhận có kẹp được vật hay không.

Điểm khác bản cũ (quan trọng nhất của cả gói sửa này):
  Bản cũ: `self._gripper(grasp=True)` trả True là đi tiếp — kể cả khi ngón kẹp
  đóng sập vào KHÔNG KHÍ (gạt vật ra, hoặc detection lệch). Chuỗi vẫn "PLACE"
  rồi "RELEASE" rồi báo "✓ thành công". Ở dây chuyền thật đó là hàng lỗi trôi
  xuống công đoạn sau.
  Bản mới: sau khi đóng, đọc left_finger từ joint_states —
      finger ≈ giới hạn dưới  → đóng hết ⇒ KHÔNG có vật (grasp fail)
      finger ≈ giới hạn trên  → chưa đóng được ⇒ fail
      ở giữa (stall vào vật)  → CÓ vật ⇒ pass
  Và holding() cho phép kiểm tra lại giữa đường (phát hiện tuột vật khi vận chuyển).

Ngón kẹp chạy PWM mode nên "vị trí" chỉ tin được ở trạng thái tĩnh — mọi phép
đọc đều sau settle_s.
"""
import time

from interbotix_xs_msgs.msg import JointSingleCommand

FINGER_JOINT = 'left_finger'
FINGER_CLOSED = 0.015     # m — SRDF group_state "Grasping"
FINGER_OPEN = 0.037       # m — SRDF group_state "Released"


class Gripper:
    def __init__(self, node, executor, js, *, publisher=None,
                 gripper_group='interbotix_gripper', use_bridge=True,
                 finger_closed=FINGER_CLOSED, finger_open=FINGER_OPEN,
                 empty_margin=0.0025, open_margin=0.0025, settle_s=0.4,
                 velocity_scale=0.2, allowed_time=4.0,
                 pwm_grasp=250.0, pwm_release=-250.0, pwm_timeout_s=2.5,
                 gripper_name='gripper', dry_run=False):
        self._node = node
        self._log = node.get_logger()
        self._exec = executor
        self._js = js
        self._pub = publisher
        self.group = gripper_group
        self.use_bridge = bool(use_bridge)
        self.closed = float(finger_closed)
        self.open_pos = float(finger_open)
        self.empty_margin = float(empty_margin)
        self.open_margin = float(open_margin)
        self.settle_s = float(settle_s)
        self.velocity_scale = float(velocity_scale)
        self.allowed_time = float(allowed_time)
        self.pwm_grasp = float(pwm_grasp)
        self.pwm_release = float(pwm_release)
        self.pwm_timeout = float(pwm_timeout_s)
        self.gripper_name = gripper_name
        self.dry_run = bool(dry_run)

    # ── trạng thái ngón kẹp ─────────────────────────────────────────────
    def finger(self):
        return self._js.position(FINGER_JOINT)

    def holding(self):
        """True/False nếu đọc được ngón kẹp, None nếu không có dữ liệu.

        Có vật ⇔ ngón dừng ở KHOẢNG GIỮA: không sập tới giới hạn dưới (kẹp
        không khí) và cũng không còn ở giới hạn trên (chưa đóng).
        """
        pos = self.finger()
        if pos is None:
            return None
        return (pos > self.closed + self.empty_margin
                and pos < self.open_pos - self.open_margin)

    # ── lệnh ────────────────────────────────────────────────────────────
    def close(self, *, verify=True, expected_finger=None):
        """Kẹp. verify=True → False nếu không thực sự có vật trong ngón."""
        if not self._command(self.closed, self.pwm_grasp, 'GRASP'):
            return False
        if self.dry_run or not verify:
            return True
        time.sleep(self.settle_s)
        pos = self.finger()
        if pos is None:
            self._log.warn('Không đọc được left_finger — bỏ qua kiểm tra kẹp '
                           '(kiểm tra topic joint_states).')
            return True
        if pos <= self.closed + self.empty_margin:
            self._log.error(f'GRASP FAIL: ngón đóng hết (finger={pos:.4f}m) — '
                            'không có vật trong ngón (detection lệch / gạt trượt vật).')
            return False
        if pos >= self.open_pos - self.open_margin:
            self._log.error(f'GRASP FAIL: ngón chưa đóng (finger={pos:.4f}m) — '
                            'gripper bị kẹt hoặc chưa ở PWM mode?')
            return False
        if expected_finger is not None and abs(pos - float(expected_finger)) > 0.006:
            self._log.warn(f'Đã kẹp được nhưng finger={pos:.4f}m lệch nhiều so với '
                           f'kỳ vọng {float(expected_finger):.4f}m — vật lạ / kẹp lệch?')
        self._log.info(f'GRASP OK — có vật trong ngón (finger={pos:.4f}m).')
        return True

    def open(self, *, verify=True):
        if not self._command(self.open_pos, self.pwm_release, 'RELEASE'):
            return False
        if self.dry_run or not verify:
            return True
        time.sleep(self.settle_s)
        pos = self.finger()
        if pos is not None and pos < self.open_pos - 3 * self.open_margin:
            self._log.warn(f'RELEASE: ngón mới ở {pos:.4f}m (chưa mở hết) — '
                           'vật có thể còn dính trong ngón.')
            return False
        return True

    def _command(self, target, pwm, label):
        if self.dry_run:
            self._log.info(f'[DRY-RUN] gripper {label} → {target:.4f}m')
            return True
        if self.use_bridge:
            return self._exec.move_joints(
                [target], self.velocity_scale, label=f'GRIPPER/{label}',
                group=self.group, joint_names=[FINGER_JOINT],
                allowed_time=self.allowed_time, retries=0, verify=False)
        return self._pwm(pwm, label)

    def _pwm(self, pwm, label):
        """Fallback không qua MoveIt: PWM trực tiếp + CHỜ ĐẾN KHI STALL.

        Bản cũ dùng time.sleep(2.0) rồi trả True vô điều kiện — không biết ngón
        đã đóng hay chưa, và luôn tốn đúng 2 s.
        """
        if self._pub is None:
            self._log.error('Không có publisher JointSingleCommand cho PWM fallback.')
            return False
        self._pub.publish(JointSingleCommand(name=self.gripper_name, cmd=float(pwm)))
        t0 = time.monotonic()
        last, still_since = self.finger(), None
        while time.monotonic() - t0 < self.pwm_timeout:
            time.sleep(0.05)
            pos = self.finger()
            if pos is None or last is None:
                last = pos
                continue
            if abs(pos - last) < 0.0004:
                still_since = still_since or time.monotonic()
                if time.monotonic() - still_since > 0.25:
                    self._log.info(f'{label} (PWM): dừng ở finger={pos:.4f}m sau '
                                   f'{time.monotonic() - t0:.2f}s.')
                    return True
            else:
                still_since = None
            last = pos
        self._log.warn(f'{label} (PWM): hết {self.pwm_timeout:.1f}s mà ngón chưa dừng.')
        return True
