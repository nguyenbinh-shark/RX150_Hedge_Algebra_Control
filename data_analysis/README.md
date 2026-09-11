# data_analysis — ghi CSV và vẽ đồ thị điều khiển

Hạ tầng dùng chung cho cả ba bộ điều khiển (`fuzzy` / `ff` / `hac`): một node ghi
telemetry ra CSV, một script vẽ đồ thị offline, và layout PlotJuggler dựng sẵn.

```
data_analysis/
├── csv_logger.py        node ROS 2: subscribe telemetry → CSV
├── plot_control_csv.py  vẽ đồ thị offline từ CSV (300 DPI)
├── requirements.txt
├── layouts/
│   ├── fuzzy_plotjuggler_layout.xml   4 tab: vị trí / vận tốc / sai số / PWM
│   └── hac_plotjuggler_layout.xml
└── utils/
    ├── csv_loader.py    đọc CSV, tự dò danh sách khớp
    └── plot_styles.py   style chuẩn IEEE/MDPI
```

## Quy ước — thứ giữ cho mọi thứ ghép được với nhau

Bộ điều khiển nào cũng phải phát đúng bộ topic này thì logger và layout mới dùng lại
được mà không sửa dòng nào. `{prefix}` ∈ `fuzzy | ff | hac`.

| Topic | Message | Nội dung |
| :--- | :--- | :--- |
| `/{robot}/{prefix}/reference` | `sensor_msgs/JointState` | `q_ref`, `q̇_ref` |
| `/{robot}/{prefix}/error` | `sensor_msgs/JointState` | `e = q_ref − q` |
| `/{robot}/{prefix}/edot` | `sensor_msgs/JointState` | `ė = q̇_ref − q̇` |
| `/{robot}/{prefix}/effort` | `sensor_msgs/JointState` | PWM `u` + phần bù trọng lực |
| `/{robot}/joint_states` | `sensor_msgs/JointState` | `q`, `q̇` thật từ encoder |

Cột CSV: `timestamp` + mỗi khớp 8 cột `{joint}_` × `pos`, `vel`, `ref_pos`, `ref_vel`,
`err`, `edot`, `pwm`, `grav`. Góc tính bằng **radian**, PWM trong **−1023…1023**.

> `tuning_lib.py` (trong `rx150_motion_common`) ghi CSV **cùng schema này** cho các phiên
> tuning/hiệu chuẩn — file nó sinh ra mở được thẳng bằng `plot_control_csv.py`.

## Ghi dữ liệu

```bash
cd ~/RX150_Hedge_Algebra_Control/data_analysis
python3 csv_logger.py --ros-args -p controller_prefix:=fuzzy -p robot_name:=rx150
```

`Ctrl+C` để dừng; file ra `{prefix}_data_YYYYMMDD_HHMMSS.csv` tại thư mục hiện hành.

Muốn thu **một chu kỳ gắp** (quỹ đạo ee + nhận diện + toạ độ giá) thì dùng đường khác:
`./rx150.sh record` → `tuning_runs/<nhãn>_<ts>/`, vẽ bằng `./rx150.sh plot <thư mục>`.

## Vẽ đồ thị offline

```bash
python3 plot_control_csv.py fuzzy_data_20260813_140000.csv
python3 plot_control_csv.py <csv> --joints waist shoulder     # chỉ 2 khớp
python3 plot_control_csv.py <csv> --start 2.0 --end 10.0      # cắt khoảng thời gian
python3 plot_control_csv.py <csv> --save bao_cao.png          # png / pdf / svg
```

## PlotJuggler (giám sát realtime)

```bash
sudo apt install ros-humble-plotjuggler-ros      # 1 lần
ros2 run plotjuggler plotjuggler -l data_analysis/layouts/fuzzy_plotjuggler_layout.xml
```

> Nạp layout **không** tự bật subscriber: phải vào bảng *Streaming* → chọn
> *ROS2 Topic Subscriber* → **Start** rồi mới tick topic. Layout trống sau khi mở là
> bình thường, không phải lỗi.
