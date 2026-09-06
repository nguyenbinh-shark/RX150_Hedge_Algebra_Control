#!/usr/bin/env python3
"""
status — state machine + báo trạng thái ra ngoài + thống kê chu kỳ.

Node pick-place chạy tay là "chạy rồi xem log". Trên dây chuyền thì lớp trên
(HMI/PLC/MES) phải đọc được: đang ở bước nào, vật thứ mấy, lỗi gì, bao nhiêu
lần thành công, cycle time bao nhiêu. Ở đây là 1 topic JSON duy nhất
(std_msgs/String) — đủ cho `ros2 topic echo` và cho một GUI đọc.
"""
import json
import time


class State:
    INIT = 'INIT'
    IDLE = 'IDLE'
    SCANNING = 'SCANNING'
    PLANNING = 'PLANNING'
    APPROACH = 'APPROACH'
    DESCEND = 'DESCEND'
    GRASP = 'GRASP'
    LIFT = 'LIFT'
    TRANSPORT = 'TRANSPORT'
    PLACE = 'PLACE'
    INSERT = 'INSERT'
    RELEASE = 'RELEASE'
    RETRACT = 'RETRACT'
    HOMING = 'HOMING'
    RECOVERY = 'RECOVERY'
    DONE = 'DONE'
    FAULT = 'FAULT'
    ESTOP = 'ESTOP'


class TaskStatus:
    """Giữ state hiện tại + counter, publish mỗi lần đổi."""

    def __init__(self, node, topic, *, extra_defaults=None):
        from std_msgs.msg import String
        self._String = String
        self._node = node
        self._log = node.get_logger()
        self._pub = node.create_publisher(String, topic, 10)
        self.state = State.INIT
        self.detail = ''
        self.attempted = 0
        self.succeeded = 0
        self.faults = 0
        self.last_error = ''
        self.cycle_start = None
        self.last_cycle_s = None
        self._extra = dict(extra_defaults or {})

    # ── state ───────────────────────────────────────────────────────────
    def set(self, state, detail='', log=True, **extra):
        self.state = state
        self.detail = detail
        self._extra.update(extra)
        if log:
            msg = f'[{state}]' + (f' {detail}' if detail else '')
            if state in (State.FAULT, State.ESTOP):
                self._log.error(msg)
            else:
                self._log.info(msg)
        self.publish()

    def fault(self, reason, **extra):
        self.faults += 1
        self.last_error = str(reason)
        self.set(State.FAULT, str(reason), **extra)

    # ── chu kỳ ──────────────────────────────────────────────────────────
    def begin_cycle(self):
        self.cycle_start = time.monotonic()
        self.attempted += 1

    def end_cycle(self, ok):
        if self.cycle_start is not None:
            self.last_cycle_s = round(time.monotonic() - self.cycle_start, 2)
            self.cycle_start = None
        if ok:
            self.succeeded += 1

    @property
    def success_rate(self):
        return round(self.succeeded / self.attempted, 3) if self.attempted else None

    # ── publish ─────────────────────────────────────────────────────────
    def publish(self):
        payload = {
            'state': self.state,
            'detail': self.detail,
            'attempted': self.attempted,
            'succeeded': self.succeeded,
            'faults': self.faults,
            'success_rate': self.success_rate,
            'last_cycle_s': self.last_cycle_s,
            'last_error': self.last_error,
            'stamp': round(time.time(), 3),
        }
        payload.update(self._extra)
        self._pub.publish(self._String(data=json.dumps(payload, ensure_ascii=False)))

    def summary(self):
        rate = '—' if self.success_rate is None else f'{self.success_rate * 100:.0f}%'
        return (f'{self.succeeded}/{self.attempted} thành công ({rate}), '
                f'{self.faults} fault, cycle cuối {self.last_cycle_s}s')
