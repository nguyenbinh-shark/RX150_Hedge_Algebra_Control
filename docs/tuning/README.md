# docs/tuning — kết quả đo và hiệu chuẩn

Thư mục này giữ **kết luận đã viết thành văn**. Số liệu thô của từng lần chạy nằm ở
`tuning_runs/<nhãn>_<ts>/`, và **CSV thô không được commit** (xem `.gitignore`) — chúng ở
lại máy đo, còn `meta.json` / `*.png` / `summary.txt` thì theo kho.

| Tài liệu | Trả lời câu gì |
| :--- | :--- |
| [do_chinh_xac_camera_robot.md](do_chinh_xac_camera_robot.md) | Toạ độ camera báo về có trùng chỗ tay gắp thật tới không; phần lệch có phải offset hằng không |
| [hieu_chuan_tag_va_camera.md](hieu_chuan_tag_va_camera.md) | Quy trình hiệu chuẩn **A → B → C** (tag trên tay gắp → camera → giá) và khi nào phải chạy lại |

## Sinh lại đồ thị

Ảnh `*.png` ở đây **không commit** vì sinh lại được hoàn toàn bằng tính toán, không cần
robot cũng không cần dữ liệu đo:

```bash
source ~/RX150_Hedge_Algebra_Control/install/setup.bash
cd ~/RX150_Hedge_Algebra_Control

# mặt điều khiển fuzzy vs HAC + quỹ đạo so sánh + comparison_data.csv
RX150_PLOT_DIR=docs/tuning ros2 run rx150_motion_common compare_fuzzy_vs_hac.py

# đường cong vận tốc HAC + mặt 4 góc phần tư
RX150_PLOT_DIR=docs/tuning ros2 run rx150_motion_common plot_hac_velocity_analysis.py
```

> `RX150_PLOT_DIR` mặc định là **thư mục hiện hành**. Quên đặt biến này thì ảnh rơi ra
> gốc workspace chứ không báo lỗi.

## Đồ thị của một lần chạy thật

Khác với trên — phần này cần robot và phải ghi trước khi vẽ:

```bash
./rx150.sh record                      # thu, trong lúc ./rx150.sh tubes đang chạy
./rx150.sh plot tuning_runs/<thư mục>  # vẽ lại, không cần ROS
```

Đọc `meta.json` **trước khi tin đồ thị**: `reference: 0` nghĩa là cột `ref_*` rỗng nên
mọi RMS bám đều vô nghĩa — không phải "sai số bằng 0".
