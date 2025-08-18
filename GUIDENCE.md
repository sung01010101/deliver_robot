# Cartographer 建圖與 Nav2 導航操作指南

## 目錄
- [建圖流程 (Cartographer)](#建圖流程-cartographer)
- [導航流程 (Nav2)](#導航流程-nav2)

---

## 建圖流程 (Cartographer)

### Step 1. 啟動樹梅派上的 Lidar 及 ESP32

**Terminal 1**
```bash
ssh ubuntu@<raspberry_ip>
~/sllidar_ws/start_lidar.sh
```

**Terminal 2**
```bash
ssh ubuntu@<raspberry_ip>
ros2 launch deliver_robot esp32_serial.launch.py
```

### Step 2. 開啟建圖並遙控

**Terminal 3**
```bash
ros2 launch deliver_robot cartographer_lidar_only.launch.py
```

**Terminal 4**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/cmd_vel
```

### Step 3. 儲存地圖檔案（建圖完成後執行）

**Terminal 5**
```bash
~/map_ws/save_map.sh
```

---

## 導航流程 (Nav2)

### Step 1. 啟動樹梅派上的 Lidar、ESP32 及 IMU

**Terminal 1**
```bash
ssh ubuntu@<raspberry_ip>
~/sllidar_ws/start_lidar.sh
```

**Terminal 2**
```bash
ssh ubuntu@<raspberry_ip>
ros2 launch deliver_robot esp32_serial.launch.py

ros2 launch deliver_robot esp32_scog.launch.py  # Alternate: Launch Slip Ratio Compensation Odometry
```

**Terminal 3**
```bash
ssh ubuntu@<raspberry_ip>
ros2 launch wit_ros2_imu rviz_and_imu.launch.py
```

### Step 2. 開啟定位及導航

**Terminal 4**
```bash
ros2 launch deliver_robot localization.launch.py map:=/home/sung/map_ws/maps/<map_name>.yaml
```

**Terminal 5**
```bash
ros2 launch deliver_robot navigation.launch.py
```

**Terminal 6**
```bash
ros2 launch deliver_robot rviz.launch.py
```

### Step 3. 使用 Rviz 進行導航

- **估計機器人位置**：使用 `2D pose estimate`
- **標記導航目標點**：使用 `Nav 2 Goal`

---
