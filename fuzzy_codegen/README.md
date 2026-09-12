# fuzzy_codegen — sinh mã C từ file `.fis`

Luật mờ được thiết kế ở định dạng **`.fis` (MATLAB Fuzzy Inference System)**, rồi dịch
sang **C99 thuần** để nhúng vào node C++. Mã sinh ra không phụ thuộc thư viện nào và
không chứa mã ROS — nhờ vậy test được bằng `gcc` mà không cần robot lẫn ROS.

Tên hàm và tên tệp sinh ra **lấy theo `Name=` trong `[System]`**. Controller `#include
"fuzzy_type1.h"` và gọi `fuzzy_type1_eval(e, ed)`, nên `Name` **phải** là `'fuzzy_type1'` —
đổi nó là gãy build.

## Thiết kế luật mờ

| | Miền | Ý nghĩa |
| :--- | :--- | :--- |
| Đầu vào `e` | [−1, 1] | Sai số vị trí, đã chuẩn hoá qua `Ke` |
| Đầu vào `ed` | [−1, 1] | Đạo hàm sai số, đã chuẩn hoá qua `Ked` |
| Đầu ra `u` | [−1, 1] | Lệnh điều khiển chuẩn hoá, nhân `Ku` thành PWM |

Phương pháp **Mamdani**: AND = min, OR = max, implication = min, aggregation = max,
defuzzification = centroid.

| Controller | MF | Bảng luật |
| :--- | :--- | :--- |
| `rx150_fuzzy_controller` | `e`, `ed`: 5 MF tam giác/hình thang; đầu ra: **7 MF** (NB…PB) | 25 luật, chéo đều |
| `rx150_ff_controller` | `e`, `ed`, đầu ra: 5 MF gauss/zmf/smf | 25 luật, bão hoà sớm ở góc |

Gain `Ke`/`Ked`/`Ku` **không** nằm trong `.fis` — chúng là tham số ROS đọc từ
`config/rx150_fuzzy_gains.yaml`, sửa nóng được bằng `rx150_tuning_gui.py`. Đổi gain thì
**không** phải sinh lại mã; chỉ đổi hình dạng MF hoặc bảng luật mới phải.

## Các tệp

| Tệp | Vai trò |
| :--- | :--- |
| `fuzzy_type1.fis` | Bản chép từ package — `regenerate.sh` ghi đè, **đừng sửa ở đây** |
| `fis2c.py` | Trình dịch `.fis` → `.c`/`.h` |
| `fuzzy_type1.c` / `.h` | Mã C sinh ra — **đừng sửa tay**, sửa `.fis` rồi sinh lại |
| `fuzzy_type1_demo.c` | Demo chạy độc lập bằng `gcc` (cũng do `fis2c.py` sinh) |
| `gen_surface.py` | Quét lưới (e, ed) → `surface.json` |
| `build_surface_html.py` | `surface.json` → `fuzzy_surface.html` tự chứa |

## Sinh lại mã sau khi sửa `.fis`

**Nguồn sự thật là bản `.fis` nằm trong package**, không phải bản ở đây — `regenerate.sh`
chép nó sang thư mục này, dịch, rồi chép `.c`/`.h` ngược trở lại. Sửa thẳng vào
`fuzzy_codegen/fuzzy_type1.fis` sẽ bị lần chạy sau ghi đè.

```bash
bash src/rx150/controllers/rx150_fuzzy_controller/src/fuzzy/regenerate.sh
bash src/rx150/controllers/rx150_ff_controller/src/fuzzy/regenerate.sh
```

Hai controller giữ **hai bản `.fis` riêng** và dùng chung trình dịch này, nên sửa luật
cho `fuzzy` không tự động đổi `ff`. Sau khi sinh lại phải `./tools/build.sh` — mã C được
biên dịch vào node, không nạp lúc chạy.

Kiểm tra mã sinh ra khớp với bản đang commit (không đổi gì thì `git diff` phải trống):

```bash
bash src/rx150/controllers/rx150_fuzzy_controller/src/fuzzy/regenerate.sh
git diff --stat -- src/rx150/controllers/rx150_fuzzy_controller/src/fuzzy/ fuzzy_codegen/
```

### Bẫy thường gặp: đổi `Name=` trong `[System]`

`fis2c.py` đặt tên tệp **và** tên hàm theo `Name=`. Để `Name='fuzzy_type1_abc'` thì mã sinh
ra là `fuzzy_type1_abc.{c,h}` với `fuzzy_type1_abc_eval()`, trong khi `CMakeLists.txt` vẫn
biên dịch `src/fuzzy/fuzzy_type1.c` còn `fuzzy_node.cpp` vẫn gọi `fuzzy_type1_eval()` — lỗi
`undefined reference`. Muốn thử biến thể luật thì **giữ nguyên `Name`**, chỉ đổi MF và
`[Rules]`. Nếu thật sự cần đổi tên, phải sửa kèm `CMakeLists.txt` và `*_node.cpp`.

## Xem mặt điều khiển 3D

Kiểm tra mặt có liên tục và mượt **trước khi** nạp lên robot thật — mặt gãy khúc là dấu
hiệu bảng luật thủng hoặc MF không phủ kín miền:

```bash
cd fuzzy_codegen
python3 gen_surface.py                    # mặc định: fuzzy_type1.fis → surface.json
python3 build_surface_html.py             # surface.json → fuzzy_surface.html
xdg-open fuzzy_surface.html
```

Cả hai script đều nhận đường dẫn khác qua tham số, tiện khi so hai bản thiết kế:

```bash
python3 gen_surface.py ../src/rx150/controllers/rx150_ff_controller/src/fuzzy/fuzzy_type1.fis \
        -o surface_ff.json -N 61
python3 build_surface_html.py surface_ff.json -o surface_ff.html
```

## Test mã C độc lập (không cần ROS, không cần robot)

```bash
cd fuzzy_codegen
gcc -O2 -o /tmp/fuzzy_demo fuzzy_type1.c fuzzy_type1_demo.c -lm
/tmp/fuzzy_demo 0.5 -0.2     # tham số: e ed  (đã chuẩn hoá, miền [-1, 1])
```

Dùng để kiểm nhanh tính đối xứng của luật: `e ed` và `-e -ed` phải cho `u` đối dấu, và
`0 0` phải cho `u = 0`. Với bảng luật hiện tại, `u` bão hoà ở **±0.892** (không phải ±1) —
centroid của MF hình thang ở hai biên không bao giờ chạm mép miền; bù lại bằng `Ku`.
