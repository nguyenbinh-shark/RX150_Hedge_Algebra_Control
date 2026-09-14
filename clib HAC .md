1. Dọn dẹp / kill toàn bộ tiến trình cũ

pkill -f 'ros2|rviz|move_group|hac_node|fuzzy_node|plotjuggler' || true

2. Terminal 1: Chạy MoveIt + RViz (kết nối cánh tay RX150)
   
cd ~/RX150_500hz
source ./source_all.sh
ros2 launch rx150_hac_controller hac_moveit.launch.py \
    use_camera:=false \
    use_moveit_rviz:=true \
    gravity_model_file:=rx150_gravity_model.yaml

3. Terminal 2: Mở PlotJuggler kèm layout HAC

cd ~/RX150_500hz
source ./source_all.sh
ros2 run plotjuggler plotjuggler --layout data_analysis/layouts/hac_plotjuggler_layout.xml

4. Terminal 3 (Tùy chọn): Thay đổi nhanh hệ số 

ros2 param set /rx150/hac_node a 0.3
ros2 param set /rx150/hac_node b 12.0
ros2 param set /rx150/hac_node c 1200.0


T1 
T2
Bật point cloud:
ros2 param set /camera/camera pointcloud.enable true

1. ./rx150.sh calib: snap nhanh, chụp một tư thế
./rx150.sh calib

/home/hust/.gemini/antigravity/brain/5431718a-50f3-4f63-99c6-b611097e18da/bao_cao_hieu_chuan_ga_tag_rx150.md
