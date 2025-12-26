# Greenhouse SLAM - Hướng Dẫn Chi Tiết

Tài liệu này hướng dẫn chạy từng chức năng cụ thể của hệ thống Robot Pepper trong nhà kính.

---

## 🚀 1. Chạy Hệ Thống Hoàn Chỉnh (Full System)
Đây là chế độ demo chính, bao gồm: Mô phỏng Gazebo + Navigation + HRI (Voice) + SMACH Core.
```bash
roslaunch greenhouse_sim greenhouse_demo.launch
```

---

## 🗺️ 2. Chức Năng Vẽ Bản Đồ (SLAM Mapping)
Sử dụng khi bạn muốn tạo bản đồ mới cho nhà kính (hoặc môi trường mới).
```bash
roslaunch greenhouse_sim greenhouse_gmapping.launch
```
*   Dùng bàn phím để điều khiển robot chạy khắp phòng:
    ```bash
    rosrun teleop_twist_keyboard teleop_twist_keyboard.py
    ```
*   Lưu bản đồ sau khi vẽ xong:
    ```bash
    rosrun map_server map_saver -f ~/pepper_ws/src/greenhouse_sim/maps/my_greenhouse
    ```

---

## 🧭 3. Chạy Navigation Độc Lập
Nếu bạn chỉ muốn test khả năng di chuyển (Move Base) mà không cần Logic HRI hay Tablet.
```bash
roslaunch greenhouse_sim greenhouse_navigation.launch
```
*   Mở Rviz, dùng công cụ **"2D Nav Goal"** để chấm điểm cho robot chạy thử.

---

## 🛠️ 4. Debug & Test Từng Module
Hướng dẫn kiểm tra riêng lẻ từng thành phần để sửa lỗi.

### A. Test Giao Diện Tablet (Tablet UI)
Chạy riêng màn hình hiển thị để kiểm tra load ảnh/text.
```bash
rosrun greenhouse_sim tablet_ui.py
```
*   Gửi lệnh test qua terminal khác:
    ```bash
    rostopic pub /pepper/tablet/command std_msgs/String "data: 'SHOW_IMAGE:rose'" --once
    ```

### B. Test Nhận Diện Người (Leg Detection)
Kiểm tra xem Laser có phát hiện được chân người hay không.
```bash
roslaunch greenhouse_sim leg_tracking.launch
```
*   Mở Rviz, add topic `/visualization_marker` để xem các chấm tròn quanh chân người.

### C. Script Test Thủ Công (Manual Script)
Script python giúp bạn giả lập các lệnh Voice/Nav mà không cần nói thật.
```bash
rosrun greenhouse_sim manual_ros_test.py
```
*   Chọn các phím chức năng (1: Đi đến cây, 2: Dừng khẩn cấp, ...).

---

## 📦 Cài Đặt (Nhắc Lại)
```bash
# 1. Cài dependencies
sudo apt-get install ros-noetic-navigation ros-noetic-teb-local-planner \
                     ros-noetic-smach-ros ros-noetic-gmapping \
                     python3-tk

# 2. Cài thư viện Python
pip3 install gTTS pygame Pillow

# 3. Build workspace
cd ~/pepper_ws
catkin_make
source devel/setup.bash
```
