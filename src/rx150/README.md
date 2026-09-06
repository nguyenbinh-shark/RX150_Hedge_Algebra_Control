# src/rx150 — code của dự án RX150

Mọi package do dự án viết đều nằm ở đây, chia theo tầng
[IRROS](https://github.com/Interbotix/interbotix_ros_core#code-structure). Code đi mượn
(Interbotix upstream, MoveIt) nằm riêng ở `src/vendor/` và **không** được commit.

| Thư mục | Tầng IRROS | Vai trò |
| :--- | :--- | :--- |
| [`rx150/`](rx150/) | — | Metapackage, depend toàn bộ bên dưới |
| [`controllers/`](controllers/) | Control | Ba bộ điều khiển khớp chạy song song để so sánh |
| [`rx150_toolbox/`](rx150_toolbox/) | Application Support | Thư viện + hạ tầng + nhận diện, dùng chung |
| [`apps/`](apps/) | Application | Ứng dụng người dùng cuối |

## Luật phụ thuộc

```
apps/  ──depend──▶  rx150_toolbox/  ──depend──▶  vendor (xs_sdk, MoveIt, RealSense)
                          ▲
controllers/ ─────────────┘
```

**Tầng trên gọi tầng dưới, không bao giờ ngược lại.** Cụ thể:

* `rx150_toolbox/*` **không được** import từ `apps/*`. Nếu một app cần dùng lại logic của
  app khác thì logic đó phải đi lên `rx150_modules` trước.
* `controllers/*` không phụ thuộc `apps/*`, và không phụ thuộc lẫn nhau ngoài phần hạ tầng
  chung ở `rx150_motion_common`.
* Ứng dụng mới: xem [`apps/README.md`](apps/README.md).

## Vì sao gom ba controller vào một thư mục

`fuzzy`, `ff` và `hac` là **ba biến thể của cùng một bài toán** (PWM vòng kín 100 Hz cho
RX150), tồn tại song song để so sánh A/B chứ không phải ba tính năng khác nhau. Gom lại
để rõ rằng chọn một trong ba, không phải chạy cả ba.
