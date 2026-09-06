# rx150_modules — thư viện dùng chung cho mọi ứng dụng RX150

**Tầng IRROS:** Application Support · **Ngôn ngữ:** Python thuần (không có node)

Mọi primitive mà một ứng dụng gắp–thả cần đều ở đây. Ứng dụng chỉ việc `<exec_depend>`
rồi `from rx150_modules.… import …` — **không được** tự viết lại MoveGroup, gripper hay
planning scene. Đó là lý do package này tồn tại: trước khi tách ra, cùng một khối ~200
dòng MoveGroup + gripper + scene bị copy ở `pick_place_moveit_node`, `tube_rack_node` và
`hri_motion_node`, rồi mỗi bản lệch nhau một chút.

## Các module

| Module | Nội dung | Điểm đáng chú ý |
| :--- | :--- | :--- |
| `kinematics` | IK/FK **giải tích** 5-DoF (x, y, z, pitch; roll tự do) | Thay cho IK-oracle của SDK: `mr.IKinSpace` là Newton lặp với 3 seed cố định nên **fail ngẫu nhiên**. Bản giải tích có pytest phủ, chạy không cần robot |
| `motion` | `MoveItExecutor` (MoveGroup + ExecuteTrajectory) và `JointStateMonitor` | Thực thi có **verify**: goal xong không có nghĩa là tới nơi |
| `gripper` | `Gripper` đóng/mở kèm **xác nhận kẹp được vật** | Bản cũ coi `grasp=True` là thành công kể cả khi ngón kẹp trượt |
| `scene` | `SceneManager`: vật cản tĩnh + attach/detach vật đang kẹp | |
| `skills` | Các "verb" pick-place ghép từ motion + gripper + scene | |
| `status` | State machine + công bố trạng thái + thống kê chu kỳ | Để tầng trên (HMI/PLC/MES) đọc được đang ở bước nào, lỗi gì |
| `detection` | Nguồn vật thể từ tầng nhận diện, **có kiểm tra tuổi dữ liệu** | Detector chết mà không kiểm tuổi thì tay vẫn lao xuống |
| `params` | Bộ tham số chung + `build_stack()` dựng cả stack trong một lời gọi | Mỗi node tự `declare_parameter` một danh sách hơi khác nhau là nguồn lỗi cũ |

## Dùng trong một ứng dụng mới

```python
from rx150_modules.params import build_stack, declare_common, read_common, table_object
from rx150_modules.status import State

class MyApp(Node):
    def __init__(self):
        super().__init__('my_app')
        declare_common(self)                 # khai báo đúng bộ tham số chuẩn
        cfg = read_common(self)
        self.stack = build_stack(self, cfg)  # kinematics + motion + gripper + scene + skills
```

Xem [`rx150_pick_place`](../../apps/rx150_pick_place/) làm mẫu tham chiếu đầy đủ.

## Test

Cả hai bài chạy được **không cần robot lẫn MoveIt**:

```bash
./tools/build.sh --packages-select rx150_modules
colcon test --packages-select rx150_modules && colcon test-result --verbose

# hoặc chạy thẳng
python3 -m pytest src/rx150/rx150_toolbox/rx150_modules/test -q
```

## Lưu ý cài đặt

Thư viện được cài bằng `install(DIRECTORY)` + env-hook PYTHONPATH, **không** qua
`ament_python_install_package` — trên máy này `setuptools 84` (~/.local) không hợp với
`packaging 21.3` (apt) nên bước build egg của `ament_cmake_python` sẽ fail. Chi tiết trong
`CMakeLists.txt` và [`tools/build.sh`](../../../../tools/build.sh).
