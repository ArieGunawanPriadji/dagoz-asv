# ASV Software Stack — Tim Dagozilla — KKI 2026

Repository software untuk kapal Autonomous Surface Vessel (ASV) kategori
Prototype KKI 2026. Stack: **ROS2 Jazzy** + **Pixhawk (ArduRover)** via **MAVROS**.
Seluruh node dituliskan dalam **C++** (ament_cmake); hanya launch file yang tetap
Python karena itu format standar `launch_ros` di ROS2.

## Arsitektur Singkat

```
Pixhawk (ArduRover firmware)
        │  MAVLink
        ▼
   MAVROS (driver resmi PX4/ArduPilot <-> ROS2)
        │
        ▼
 asv_mavros_bridge   ← baca state/GPS Pixhawk, service ganti mode
        │
        ▼
 asv_mission (state machine misi)
        │  publish active_waypoint
        ▼
 asv_guidance_control (waypoint follower / PID / pure pursuit)
        │  publish cmd_vel
        ▼
   MAVROS setpoint  →  Pixhawk (mode GUIDED)

 asv_perception (kamera/lidar) ──► obstacles ──► dipakai guidance/mission
 asv_navigation  (opsional: EKF lokal jika GPS Pixhawk kurang presisi)
```

## Struktur Package

| Package | Bahasa | Fungsi |
|---|---|---|
| `asv_msgs` | interfaces (msg/srv) | Definisi message & service kustom (Waypoint, MissionStatus, dst) |
| `asv_mavros_bridge` | C++ | Jembatan node ROS2 ↔ Pixhawk lewat MAVROS |
| `asv_guidance_control` | C++ | Guidance & control: waypoint follower, PID/pure pursuit |
| `asv_navigation` | C++ (belum ada node) | State estimation tambahan (opsional, mis. EKF lokal) |
| `asv_perception` | C++ | Stub deteksi obstacle generik (lidar/sensor non-vision) |
| `asv_perception_vision` | **C++** | Deteksi buoy/obstacle via OpenCV, jalan di namespace `dagozilla` |
| `asv_mission` | C++ | State machine misi tingkat tinggi (IDLE/RUNNING/COMPLETED/ABORTED) |
| `asv_bringup` | launch/config only | Launch file utama + file konfigurasi terpusat |
| `asv_simulation` | launch/config only | Konfigurasi simulasi Gazebo + ArduPilot SITL untuk testing tanpa kapal fisik |

## Cara Pakai

```bash
# Build
cd asv_ws
colcon build --symlink-install
source install/setup.bash

# Jalankan (kapal asli, sesuaikan port Pixhawk)
ros2 launch asv_bringup asv_bringup.launch.py fcu_url:=/dev/ttyACM0:115200

# Mulai misi
ros2 service call /asv/start_mission std_srvs/srv/Trigger

# Jalankan vision detector (namespace dagozilla)
ros2 launch asv_perception_vision vision.launch.py
# Topic yang muncul: /dagozilla/camera/image_raw (subscribe), /dagozilla/obstacles (publish)
```

## Alur Kerja Tim (Rekomendasi)

1. **Branch per fitur**: `feat/guidance-pid`, `feat/obstacle-yolo`, dst. Jangan langsung push ke `main`.
2. **Testing di simulasi dulu** (`asv_simulation`, ArduPilot SITL) sebelum diuji di kapal fisik/kolam.
3. **Satu orang pegang `asv_mavros_bridge` & `asv_mission`** (interface kritis), lainnya bisa paralel di `guidance_control` / `perception`.
4. **Isi `docs/`** dengan wiring diagram, daftar topic/service, dan catatan kalibrasi kompas/GPS — penting banget buat laporan teknis & saat ditanya juri.
5. Pasang **pre-commit / CI** (`.github/workflows/ci.yml`) supaya build error ketauan sebelum H-1 lomba.

## Rulebook Reference

Sesuaikan task (navigation channel, obstacle avoidance, docking, dst) dan
scoring dengan **Buku Panduan KKI 2026** kategori ASV dari BELMAWA/Puspresnas.
Simpan salinan PDF-nya di `docs/rulebook/`.
