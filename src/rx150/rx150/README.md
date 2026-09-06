# rx150 (metapackage)

Metapackage gom toàn bộ package của dự án RX150. Không chứa code, chỉ khai báo phụ thuộc
trong `package.xml` — giống [`interbotix_ros_xsarms`](https://github.com/Interbotix/interbotix_ros_manipulators/tree/main/interbotix_ros_xsarms/interbotix_ros_xsarms)
của upstream.

Công dụng chính:

```bash
# Kéo đúng bộ phụ thuộc hệ thống trong một lần
rosdep install --from-paths src --ignore-src -r -y

# Build tất cả những gì rx150 cần (kể cả vendor), không cần liệt kê tay
./tools/build.sh --packages-up-to rx150
```

Khi thêm package mới vào `src/rx150/`, nhớ thêm một dòng `<exec_depend>` tương ứng ở đây —
nếu không thì hai lệnh trên sẽ bỏ sót nó.
