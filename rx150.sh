#!/usr/bin/env bash
# rx150.sh — Chạy hệ RX150 với đúng tổ hợp cờ tối ưu cho từng mục đích.
#
# Dùng:   ./rx150.sh <chế_độ>
#
#   t1        Robot + MoveIt + camera (HẰNG NGÀY — không point cloud, không MoveIt-RViz)
#   t1-pcl    Như t1 nhưng BẬT point cloud (chỉ khi cần luồng PCL / OctoMap)
#   t1-hac    Như t1 nhưng dùng bộ điều khiển HAC thay cho fuzzy (A/B)
#   t2        Perception thường ngày: nạp TF calib (+ RViz)
#   calib     Phiên HIỆU CHUẨN CAMERA (ArmTag): armtag GUI + RViz (không cần point cloud)
#   rack-calib  HIỆU CHUẨN VỊ TRÍ GIÁ ĐỠ ỐNG NGHIỆM bằng AprilTag -> rack_pose.yaml
#   rack-gui    KIỂM BẰNG MẮT: RViz + ảnh chồng 4 lỗ lên hình camera (không ghi file)
#   rack-watch  Theo dõi giá có bị xê dịch không (đo liên tục, KHÔNG ghi file)
#   rack-pub    Chỉ phát TF tube_rack từ rack_pose.yaml (không cần camera)
#   t2-rack     T2 + theo dõi giá NGAY TRONG RViz của T2 (thay cho rack-gui riêng)
#   record      Thu dữ liệu một lần chạy tube_rack -> tuning_runs/ (CSV + meta)
#   plot        Vẽ đồ thị từ thư mục do 'record' sinh ra
#   tune      Phiên TUNE PCL: pointcloud tuner GUI + RViz (T1 phải là t1-pcl)
#   eetag     Theo dõi AprilTag trên TAY GẮP (30 fps) — nguồn đo ngoài cho bench
#   eetag-hold  Bench camera vs encoder vs lệnh trên bộ pose tĩnh (cần eetag)
#   eetag-watch Như trên nhưng CHỈ GHI, không ra lệnh — dùng khi task đang chạy
#   eetag-tagcal ĐO VỊ TRÍ TAG trên tay gắp (3 lượt + gộp + cài, ~17 phút)
#   eetag-calib HIỆU CHUẨN LẠI TF CAMERA bằng tag tay gắp (39 pose lưới -> refine)
#   eetag-pick  BÀI TEST bám quỹ đạo + tới điểm trên quỹ đạo mô phỏng GẮP
#   tubes     Gắp ống nghiệm (YOLO, Layer 2)
#   tubes-direct  Như tubes nhưng KHÔNG qua move_group (motion_backend:=direct)
#   gesture   Gắp vật được chỉ bằng cử chỉ tay (pick_place + hand_gesture)
#   dry       Gắp ống nghiệm CHẠY THỬ: chỉ IK + log, không cử động
#   dry-fake  Như dry nhưng bơm 1 ống GIẢ — chạy hết chuỗi khi KHÔNG có camera
#   all       T1+T2+YOLO trong MỘT terminal (fuzzy_moveit_perception)
#   test      Bộ test offline perception: 14 test YOLO + hình học hiệu chuẩn giá
#   check     Smoke-test theo tầng: joint_states + action server + TF/detection
#   reach     Bảng tầm với + kiểm config (KHÔNG cần robot)
#   diag      Chụp trạng thái để gửi kèm báo lỗi -> diag_<ts>.tar.gz
#
# Thứ tự bring-up (chi tiết: src/rx150_pick_place/docs/RUNBOOK.md):
#   reach → t1 → t2 → [rack-calib] → check → dry-fake → tubes
#   (rack-calib chỉ chạy lại khi giá bị xê dịch — kết quả nằm trong file.)
#   Hỏng ở bậc nào thì DỪNG ở bậc đó, đừng chạy tiếp.
#
# Nguyên tắc tối ưu:
#   - Point cloud (~295 MB/s) chỉ bật khi thật sự dùng (t1-pcl + tune).
#   - MỘT RViz duy nhất (bên perception, chạy trên GPU NVIDIA); MoveIt-RViz tắt.
#   - Tuner GUI chỉ mở trong phiên tune/calib rồi TẮT — InterbotixRobotNode rò
#     timer 20Hz theo mỗi lần kéo slider, để GUI mở lâu là CPU tăng dần.
#   - MỘT yolo_tube_detector duy nhất: 'all' đã có sẵn nên task phải detector:=false;
#     đường t1→t2→tubes thì không dính (rx150_perception KHÔNG chạy YOLO).
set -e
cd "$(dirname "$0")"
source ./source_all.sh >/dev/null

case "${1:-}" in
  t1)
    echo "⚠️  Tay máy sẽ BẬT TORQUE và tự về HOME khi launch. Dọn chỗ quanh robot."
    exec ros2 launch rx150_fuzzy_controller fuzzy_moveit.launch.py \
        use_camera_static_tf:=false \
        use_moveit_rviz:=false
    ;;
  t1-pcl)
    echo "⚠️  Tay máy sẽ BẬT TORQUE và tự về HOME khi launch. Point cloud BẬT (~295 MB/s)."
    exec ros2 launch rx150_fuzzy_controller fuzzy_moveit.launch.py \
        use_camera_static_tf:=false \
        use_moveit_rviz:=false \
        rs_camera_pointcloud_enable:=true
    ;;
  t1-hac)
    # Bộ điều khiển thay thế: PD tuyến tính (mặt HAC a/b/c) + Ruckig + bù trọng lực.
    # Cùng bridge, cùng action /rx150/arm_controller/follow_joint_trajectory, cùng
    # gripper bridge ⇒ tầng task (tubes/gesture) chạy y hệt, KHÔNG cần đổi cờ gì.
    # Khác fuzzy ở chỗ gain: HAC dùng a/b/c + u_max (u_max vừa là gain vừa là nắp),
    # không có Ku. Bù ma sát có sẵn nhưng mặc định = 0 tới khi chạy rx150_friction_id.py.
    #
    # gravity_model_file: model lượng giác đã hiệu chuẩn trên phần cứng 2026-09-09
    # (rx150_gravity_id.py identify, 132 pose). Thay đường Pinocchio+Gff, vốn lệch
    # hệ thống -53 PWM ở elbow. Bỏ arg này => quay về Pinocchio để so A/B.
    echo "⚠️  Tay máy sẽ BẬT TORQUE và tự về HOME khi launch. Dọn chỗ quanh robot."
    exec ros2 launch rx150_hac_controller hac_moveit.launch.py \
        use_camera_static_tf:=false \
        use_moveit_rviz:=false \
        gravity_model_file:=rx150_gravity_model.yaml
    ;;
  t2)
    exec ros2 launch rx150_perception rx150_perception.launch.py use_rviz:=true
    ;;
  calib)
    echo "Hiệu chuẩn: đưa AprilTag vào khung hình, chỉnh Snapshots=10, bấm Snap Pose."
    echo "Xong thì Ctrl+C và chạy lại './rx150.sh t2' (đừng để GUI mở lâu)."
    exec ros2 launch rx150_perception rx150_perception.launch.py \
        use_armtag_tuner_gui:=true \
        use_rviz:=true
    ;;
  rack-calib|rackcalib)
    # Hiệu chuẩn VỊ TRÍ GIÁ, cùng khuôn với calib camera nhưng khác đối tượng:
    #   calib      camera: armtag     → static_transforms.yaml → static_trans_pub
    #   rack-calib GIÁ:    rack_calib → rack_pose.yaml         → tube_rack_node
    # Cần T1 (robot + camera) và T2 (TF world<->camera) đang chạy: phép đo này
    # nằm TRÊN NỀN TF camera — calib camera sai thì đây chỉ chép lại cái sai đó.
    shift
    echo "Tag của GIÁ phải nằm trong khung hình camera và không bị che."
    echo "Thêm rviz:=true để vừa snap vừa soi bằng mắt; hoặc chạy './rx150.sh rack-gui'."
    echo "Xong: ./rx150.sh reach để kiểm 4 lỗ có với tới được không."
    exec ros2 launch rx150_perception rack_calib.launch.py mode:=snap "$@"
    ;;
  rack-gui|rackgui)
    # KIỂM BẰNG MẮT trước khi tin vào số. Không ghi đè rack_pose.yaml.
    #   khung 3D  — marker 4 lỗ + viền giá + trục lỗ, so với model robot
    #   khung ảnh — 4 lỗ chiếu NGƯỢC lên ảnh camera: vòng tròn trùng lỗ thật = ĐÚNG.
    #               Đây là phép kiểm đi qua CẢ TF hiệu chuẩn camera lẫn pose tag,
    #               nên nó bắt được cả lỗi calib camera lẫn lỗi hình học khai sai.
    # Lục = đang đo. Cam = đã lưu trong file (chỉ hiện khi đã snap ít nhất một lần).
    shift
    echo "Lục = nghiệm ĐANG ĐO · Cam = nghiệm đã lưu trong rack_pose.yaml."
    echo "Nhìn khung ảnh: 4 vòng tròn phải nằm ĐÚNG trên 4 miệng lỗ thật."
    exec ros2 launch rx150_perception rack_calib.launch.py mode:=watch rviz:=true "$@"
    ;;
  rack-watch|rackwatch)
    # Chỉ đo và in độ lệch so với rack_pose.yaml — KHÔNG ghi đè file.
    shift
    exec ros2 launch rx150_perception rack_calib.launch.py mode:=watch "$@"
    ;;
  rack-pub|rackpub)
    # Phát TF tĩnh rx150/base_link -> tube_rack (+ tube_rack/slot0..3) để soi
    # trong RViz. KHÔNG cần camera. tube_rack_node KHÔNG dùng TF này (nó đọc
    # thẳng file), nên chạy hay không chạy đều không đổi hành vi gắp.
    shift
    exec ros2 launch rx150_perception rack_calib.launch.py mode:=publish "$@"
    ;;
  tune)
    echo "Tune PCL: T1 phải đang chạy './rx150.sh t1-pcl' (cần point cloud)."
    echo "Xong thì bấm Save Config, Ctrl+C và quay về './rx150.sh t2'."
    exec ros2 launch rx150_perception rx150_perception.launch.py \
        use_pointcloud_tuner_gui:=true \
        use_rviz:=true
    ;;
  eetag)
    # Detector AprilTag LIÊN TỤC cho tag dán trên tay gắp -> /ee_tag/tag_detections.
    # Không bật driver camera (T1 đang giữ D435i); chỉ đọc ảnh color có sẵn.
    # Cần: T1 (robot+camera) và T2 (TF hiệu chuẩn world<->camera) đang chạy.
    echo "Tag phải dán trên tay gắp và NHÌN THẤY được từ camera."
    echo "Kiểm tra: ros2 topic hz /ee_tag/tag_detections  (0 Hz = không thấy tag)."
    exec ros2 launch rx150_perception ee_tag.launch.py
    ;;
  eetag-hold)
    # So vị trí tay gắp: lệnh (FK setpoint) vs encoder (TF) vs camera (AprilTag).
    # Robot SẼ CỬ ĐỘNG qua bộ pose tĩnh rồi về sleep. Kết quả vào tuning_runs/.
    shift
    echo "⚠️  Tay máy sẽ đi qua các pose đo rồi về SLEEP. Dọn chỗ quanh robot."
    exec ros2 run rx150_motion_common rx150_ee_tag_bench.py hold "$@"
    ;;
  eetag-watch)
    # Chỉ quan sát: không publish setpoint, an toàn khi tubes/gesture đang chạy.
    shift
    exec ros2 run rx150_motion_common rx150_ee_tag_bench.py watch "$@"
    ;;
  eetag-tagcal|tagcal)
    # ĐO VỊ TRÍ TAG TRÊN TAY GẮP -> ee_tag_offset.yaml (nguồn sự thật duy nhất).
    #
    # Vì sao không lấy số trong URDF: `ar_tag_link` khai vị trí gá AR tag CHÍNH
    # HÃNG của Interbotix. Gá tự thiết kế thì con số đó sai, và Snap Pose sai
    # đúng bằng chừng ấy — phần xoay đi 1:1. Đo 2026-09-11: lệch 4.82 mm / 2.09°.
    #
    # Bộ pose `tagcal` QUÉT wrist_rotate ±1.2 rad, khác mọi bộ khác: t_X chỉ tách
    # được khỏi tịnh tiến camera nhờ R_ee THAY ĐỔI giữa các pose. Bộ nào để
    # wrist_rotate ≡ 0 thì R_ee·t_X gần như hằng và lẫn hết vào t_camera.
    #
    # BA LƯỢT, không phải một: bộ pose có 20 pose mà kiểm tra chéo chia đôi THEO
    # POSE, nên một lượt chỉ còn ~9 pose mỗi nửa. Đo 2026-09-11: lượt đơn cho hai
    # nửa lệch 4.2-5.5 mm (DỮ LIỆU CHƯA ĐỦ), ba lượt gộp cho 0.8 mm (ĐÁNG TIN).
    # Ba lượt cố tình khác động lực học (bậc thang / trườn chậm hai chiều) để sai
    # số quán tính - ma sát không cùng dấu ở cả ba.
    #
    # CHẠY LẠI KHI: in/lắp lại gá, dán lại tag, đổi tag, đổi độ phân giải camera.
    shift
    stamp=$(date +%Y%m%d_%H%M%S)
    base="tuning_runs/tagcal_$stamp"
    echo "⚠️  BA lượt đo × ~5 phút (tổng ~17 phút). Tay máy đi 20 pose mỗi lượt rồi về SLEEP."
    echo "    Cần: T1 + T2 + './rx150.sh eetag' đang chạy. Dọn chỗ quanh robot."
    echo "    Kết quả gộp sẽ TỰ CÀI vào ee_tag_offset.yaml (bản cũ được sao lưu)."
    echo "    Xong nhớ khởi động lại T2."
    rc=0; i=0
    for flags in "" "--creep-vel 0.15 --creep-sign 1" "--creep-vel 0.15 --creep-sign=-1"; do
      i=$((i + 1))
      echo; echo "════════ lượt $i/3 ${flags:-(bậc thang)} ════════"
      ros2 run rx150_motion_common rx150_ee_tag_bench.py hold \
          --pose-set tagcal --dwell 2.5 --settle 2.5 --label tagcal \
          --out "$base/pass$i" --no-plot $flags "$@" || rc=1
    done
    if [ $rc -ne 0 ]; then
      echo "❌ Có lượt đo hỏng — KHÔNG gộp. Xem nguyên nhân bên trên."
      exit 1
    fi
    echo; echo "════════ gộp 3 lượt + cài ════════"
    exec ros2 run rx150_motion_common rx150_ee_tag_bench.py tagoffset \
        "$base/pass1/tagcal_hold.csv" \
        "$base/pass2/tagcal_hold.csv" \
        "$base/pass3/tagcal_hold.csv" --install
    ;;
  eetag-calib|eetagcalib)
    # HIỆU CHUẨN LẠI TF CAMERA — thay cho một lần Snap Pose của armtag.
    #   armtag:  MỘT tư thế  -> 6 ẩn giải từ 1 quan sát, lệch vài độ là thường
    #   cái này: 39 tư thế   -> giải bình phương tối thiểu, có kiểm tra chéo
    # t_X (offset gắn tag) CỐ ĐỊNH từ ee_tag_offset.yaml nên tịnh tiến camera
    # không đổi chác với offset tag — nghiệm ra là TUYỆT ĐỐI, không phải một
    # cặp (camera, tag) tuỳ ý cùng khớp dữ liệu.
    #
    # --keepout-rack: bỏ pose rơi trúng hộp vật cản của giá (rack_pose.yaml).
    # Giá xê dịch mà chưa snap lại thì vùng cấm sai chỗ ⇒ snap giá TRƯỚC.
    #
    # Xong: chép tuning_runs/<run>/static_transforms_refined.yaml đè lên
    # rx150_perception/config/static_transforms.yaml, khởi động lại T2, rồi
    # PHẢI snap lại giá (rack_pose nằm TRÊN NỀN TF vừa đổi).
    shift
    echo "⚠️  Tay máy sẽ đi qua ~39 pose lưới rồi về SLEEP (~5 phút). Dọn chỗ quanh robot."
    echo "    Cần: T1 (t1/t1-hac) + T2 + './rx150.sh eetag' đang chạy."
    exec ros2 run rx150_motion_common rx150_ee_tag_bench.py hold \
        --pose-set grid --grid-radii=0.16,0.24,0.31 --grid-heights=0.12,0.20,0.28 \
        --grid-waists-deg=-40,-15,15,40,60 --keepout-rack \
        --tag-offset ee_tag_offset.yaml --dwell 3.5 --settle 2.5 \
        --label calib "$@"
    ;;
  eetag-pick|eetagpick)
    # BÀI TEST ĐỘ CHÍNH XÁC trên quỹ đạo mô phỏng gắp ống nghiệm: treo trên lỗ
    # -> hạ xuống -> dừng -> nhấc -> sang lỗ kế, tốc độ lấy đúng của pick_place
    # (0.05 m/s đi ngang, 0.025 m/s lên xuống).
    #
    # Điểm thấp nhất DỪNG CÁCH miệng lỗ --clearance (mặc định 15 mm): bài test
    # không bao giờ chạm giá, kể cả khi rack_pose lệch.
    #
    # Báo cáo tách ba: (1) tới điểm, (2) bám quỹ đạo có tách TRỄ khỏi lệch hằng,
    # (3) kiểm chéo hai tag trong cùng khung hình. Muốn có (3) thì detector tag
    # giá phải chạy: ros2 launch rx150_perception rack_calib.launch.py mode:=watch
    shift
    echo "⚠️  Tay máy sẽ chạy quỹ đạo mô phỏng gắp TRÊN GIÁ rồi về SLEEP."
    echo "    Cần: T1 + T2 + eetag; nên có cả detector tag giá cho phép kiểm chéo."
    exec ros2 run rx150_motion_common rx150_ee_tag_bench.py pick \
        --tag-offset ee_tag_offset.yaml --label pick "$@"
    ;;
  tubes)
    exec ros2 launch rx150_pick_place tube_rack.launch.py
    ;;
  tubes-direct)
    echo "⚠️  motion_backend=direct: KHÔNG có tránh vật cản (không qua move_group)."
    echo "    An toàn nằm ở waypoint. Mỗi thao tác planning scene sẽ treo ~3 s rồi WARN"
    echo "    nếu move_group không chạy — đó là bình thường, xem RUNBOOK B5b."
    exec ros2 launch rx150_pick_place tube_rack.launch.py motion_backend:=direct
    ;;
  gesture)
    exec ros2 launch rx150_pick_place pick_place.launch.py
    ;;
  dry)
    exec ros2 launch rx150_pick_place tube_rack.launch.py dry_run:=true
    ;;
  dry-fake)
    # detector:=false + ống giả ⇒ chạy hết state machine mà không cần camera lẫn
    # động cơ. Đây là bậc B4 của RUNBOOK — chỗ bắt lỗi hình học rẻ nhất.
    exec ros2 launch rx150_pick_place tube_rack.launch.py \
        detector:=false dry_run:=true \
        fake_tubes:='[{"x":0.20,"y":-0.15,"z":0.03,"yaw":0.0,"class":"pink"}]'
    ;;
  all)
    # Một terminal: fuzzy_moveit + perception + YOLO. Point cloud tắt (295 MB/s,
    # thuần hiển thị). Task chạy ở terminal khác và PHẢI có detector:=false —
    # launch này đã có yolo_tube_detector rồi, hai node trùng tên là hỏng.
    echo "⚠️  Tay máy sẽ BẬT TORQUE khi launch. Dọn chỗ quanh robot."
    echo "    Task ở terminal khác PHẢI thêm detector:=false (đã có YOLO ở đây)."
    exec ros2 launch rx150_fuzzy_controller fuzzy_moveit_perception.launch.py \
        rs_camera_pointcloud_enable:=false
    ;;
  test)
    # Không exec: chạy cả hai bộ rồi tổng kết. Cả hai đều offline.
    rc=0
    ros2 run rx150_perception test_yolo_tube_detector.py || rc=1
    echo
    echo "════════ hình học hiệu chuẩn giá ════════"
    ros2 run rx150_perception test_rack_calib.py || rc=1
    exit $rc
    ;;
  check)
    # Không exec: chạy cả ba rồi tổng kết. Không bài nào phát lệnh tới robot.
    rc=0
    for t in hardware/joint_states_test.py \
             moveit/action_servers_test.py \
             perception/tf_and_detection_test.py; do
      echo
      echo "════════ $t ════════"
      python3 module_tests/run_test.py "$t" || rc=1
    done
    echo
    [ $rc -eq 0 ] && echo "✅ Tất cả smoke-test GO." \
                  || echo "❌ Có smoke-test NO-GO — xem nguyên nhân gốc bên trên."
    exit $rc
    ;;
  reach)
    # Không cần robot. Chạy trước MỌI lần đo lại vị trí giá/bàn.
    # rx150_reach_check trả 1 khi có điểm ngoài tầm — đó là KẾT QUẢ, không phải
    # lỗi script, nên phải chặn `set -e` để còn chạy hết các config.
    rc=0
    ros2 run rx150_pick_place rx150_reach_check.py || rc=1
    share="$(ros2 pkg prefix rx150_pick_place)/share/rx150_pick_place/config"
    # --no-envelope: bảng bao hình đã in ở trên, không lặp lại 3 lần.
    for f in tube_rack_params pick_place_params; do
      echo
      echo "════════ $f.yaml ════════"
      ros2 run rx150_pick_place rx150_reach_check.py \
        --config "$share/$f.yaml" --no-envelope || rc=1
    done
    echo
    echo "════════ tư thế trung chuyển (home_xyz_pitch) ════════"
    echo "IK điểm này FAIL ⇒ resolve_home() âm thầm quay lại home duỗi thẳng"
    echo "[0,0,0,0,0] (FK = 0.359,0,0.255) — quét ngang QUA giá ở x=0.26."
    ros2 run rx150_pick_place rx150_reach_check.py \
      --point 0.18 0 0.18 --pitch 0 --no-envelope || rc=1
    echo
    [ $rc -eq 0 ] && echo "✅ Mọi điểm trong config đều với tới." \
                  || echo "❌ Có điểm NGOÀI tầm với — sửa config trước khi cấp điện."
    exit $rc
    ;;
  t2-rack|t2rack)
    # T2 + rack_calib mode=watch trong CÙNG một RViz. Thay cho việc mở thêm
    # './rx150.sh rack-gui' ở terminal khác — hai RViz cùng lúc là thừa, và
    # rack-gui dựng node 'rack_tag' trùng tên với bản chạy ở đây.
    #
    # watch CHỈ ĐO, không ghi rack_pose.yaml. Muốn ghi thì vẫn phải 'rack-calib'.
    # Trong RViz bật hai Display: RackMarkers (4 lỗ trong 3D) và RackOverlay
    # (4 vòng tròn chiếu ngược lên ảnh camera — vòng trùng miệng lỗ thật = ĐÚNG).
    shift
    echo "Lục = nghiệm ĐANG ĐO · Cam = nghiệm đã lưu trong rack_pose.yaml."
    echo "RViz: xem RackOverlay — 4 vòng tròn phải nằm ĐÚNG trên 4 miệng lỗ thật."
    exec ros2 launch rx150_perception rx150_perception.launch.py \
        use_rviz:=true use_rack_watch:=true "$@"
    ;;
  record)
    # Thu MỘT lần chạy để vẽ đồ thị. Không ra lệnh gì cho robot — an toàn chạy
    # song song với tubes. Muốn có đường 'tag' (camera nhìn tay gắp) thì phải
    # bật './rx150.sh eetag' ở terminal khác trước; không có thì thêm --no-tag.
    shift
    exec python3 ./tools/record_tube_run.py "$@"
    ;;
  plot)
    # Vẽ offline từ thư mục record đã ghi. Không cần ROS đang chạy.
    shift
    if [ $# -eq 0 ]; then
      echo "Dùng: ./rx150.sh plot tuning_runs/<thư mục record>"
      echo "Gần nhất:"; ls -dt tuning_runs/*/ 2>/dev/null | head -5
      exit 1
    fi
    exec python3 ./tools/plot_tube_run.py "$@"
    ;;
  diag)
    shift
    exec ./tools/collect_diag.sh "$@"
    ;;
  *)
    # In toàn bộ khối chú thích đầu file (dòng 2 -> dòng không-phải-chú-thích
    # đầu tiên). Bản cũ dùng dải cứng '2,31p' nên lệch mỗi lần chèn thêm chế độ:
    # nó đang cắt giữa mục "Nguyên tắc tối ưu", giấu mất cảnh báo trùng
    # yolo_tube_detector. awk tự bám nên không phải sửa lại nữa.
    awk 'NR == 1 { next } /^#/ { sub(/^# ?/, ""); print; next } { exit }' "$0"
    exit 1
    ;;
esac
