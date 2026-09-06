# apps/ — tầng Application

Code người dùng cuối: đọc nhận diện, ra quyết định, gọi xuống tầng dưới để chấp hành.
Tương ứng với tầng **Research/Application** trong
[IRROS](https://github.com/Interbotix/interbotix_ros_core#code-structure) (upstream đặt
thư mục này tên là `examples/`).

| Ứng dụng | Việc nó làm |
| :--- | :--- |
| [`rx150_pick_place`](rx150_pick_place/) | Gắp ống nghiệm cắm lên giá (`tube_rack_node`); gắp vật được chỉ bằng cử chỉ tay (`pick_place_moveit_node`); kiểm tra tầm với offline (`rx150_reach_check`) |
| [`rx150_hri`](rx150_hri/) | Tương tác người–máy: tách đôi task (quyết định) và executor (chấp hành), nói chuyện qua topic nên test tay được bằng `ros2 topic pub` |

## Thêm một ứng dụng mới

1. **Tạo package `ament_cmake`** trong thư mục này (không dùng `ament_python` — xem
   `rx150_perception/CMakeLists.txt` để biết lý do), bố cục:

   ```
   apps/my_app/
   ├── CMakeLists.txt        install(PROGRAMS scripts/…) + install(DIRECTORY launch config)
   ├── package.xml
   ├── README.md             ← bắt buộc, theo yêu cầu của upstream
   ├── config/my_app.yaml    tham số, KHÔNG hardcode trong code
   ├── launch/my_app.launch.py
   └── scripts/my_app_node.py
   ```

2. **Khai báo phụ thuộc** — chỉ xuống tầng dưới:

   ```xml
   <exec_depend>rx150_modules</exec_depend>     <!-- IK, MoveIt, gripper, scene, skills -->
   <exec_depend>rx150_perception</exec_depend>  <!-- nếu cần nhận diện -->
   ```

3. **Dùng lại, đừng viết lại.** Không tự cài đặt MoveGroup client, gripper, planning
   scene hay IK — tất cả đã có trong [`rx150_modules`](../rx150_toolbox/rx150_modules/README.md).
   Nếu thấy thiếu một primitive thì **thêm vào `rx150_modules`**, đừng thêm vào app.

4. **Thêm một dòng `<exec_depend>`** vào [`../rx150/package.xml`](../rx150/package.xml)
   để metapackage kéo được app mới.

5. **Chạy detector đúng cách.** Node YOLO chỉ được có **một** bản trong toàn hệ. Launch
   của app phải có cờ `detector` (mặc định `true`) để tắt được khi detector đã chạy ở
   đường khác — xem `rx150_pick_place/launch/tube_rack.launch.py`.

## Luật không được phá

* App **không** import từ app khác. Logic dùng chung đi lên `rx150_modules`.
* App **không** phụ thuộc vào một bộ điều khiển cụ thể: nói chuyện với MoveIt
  (`move_action` / `FollowJointTrajectory`), không nói thẳng với `fuzzy_node`.
* Mọi toạ độ hình học vào `config/*.yaml`, và phải qua được `rx150_reach_check.py`
  **trước** khi cấp điện cho robot.
