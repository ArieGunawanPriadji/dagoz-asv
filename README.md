# ASV Software Stack — Tim Dagozilla — KKI 2026

Repository software untuk kapal Autonomous Surface Vessel (ASV) kategori
Prototype KKI 2026. Stack: **ROS2 Jazzy** + **Pixhawk (ArduRover)** via **MAVROS**.

## Arsitektur Singkat

```
Pixhawk (ArduRover firmware)
        │  MAVLink
        ▼
   MAVROS (driver resmi PX4/ArduPilot <-> ROS2)
        │
        ▼
 mavros_bridge   ← baca state/GPS Pixhawk, service ganti mode
        │
        ▼
 mission (state machine misi)
        │  publish active_waypoint
        ▼
 guidance_control (waypoint follower / PID / pure pursuit)
        │  publish cmd_vel
        ▼
   MAVROS setpoint  →  Pixhawk (mode GUIDED)

 vision (kamera/lidar) ──► obstacles ──► dipakai guidance/mission
 navigation  (opsional: EKF lokal jika GPS Pixhawk kurang presisi)
```

## Struktur Package

| Package | Bahasa | Fungsi |
|---|---|---|
| `msgs` | interfaces (msg/srv) | Definisi message & service kustom (Waypoint, MissionStatus, dst) |
| `mavros_bridge` | C++ | Jembatan node ROS2 ↔ Pixhawk lewat MAVROS |
| `guidance_control` | C++ | Guidance & control: waypoint follower, PID/pure pursuit |
| `navigation` | C++ (belum ada node) | State estimation tambahan (opsional, mis. EKF lokal) |
| `vision` | C++ | Stub deteksi obstacle generik (lidar/sensor non-vision) |
| `perception` | **C++** | Deteksi buoy/obstacle via OpenCV, jalan di namespace `dagozilla` |
| `mission` | C++ | State machine misi ASV KKI 2026: buoy, imaging, docking, timeout, penalti |
| `bringup` | launch/config only | Launch file utama + file konfigurasi terpusat |
| `simulation` | launch/config only | Konfigurasi simulasi Gazebo + ArduPilot SITL untuk testing tanpa kapal fisik |

## Cara Pakai

```bash
# Build
cd dagozilla_ws
colcon build --symlink-install
source install/setup.bash

# Jalankan (kapal asli, sesuaikan port Pixhawk)
ros2 launch bringup bringup.launch.py fcu_url:=/dev/ttyACM0:115200

# Mulai misi
ros2 service call /asv/start_mission std_srvs/srv/Trigger

# Event misi sesuai panduan KKI 2026
ros2 service call /asv/complete_buoy_pair std_srvs/srv/Trigger
ros2 service call /asv/mark_surface_image std_srvs/srv/Trigger
ros2 service call /asv/mark_underwater_image std_srvs/srv/Trigger
ros2 service call /asv/complete_docking std_srvs/srv/Trigger
ros2 service call /asv/record_penalty std_srvs/srv/Trigger

# Jalankan vision detector (namespace dagozilla)
ros2 launch perception vision.launch.py
# Topic yang muncul: /dagozilla/camera/image_raw (subscribe), /dagozilla/obstacles (publish)
# Class deteksi: RED_BUOY, GREEN_BUOY, BLUE_DOCKING_BUOY
```

## Alur Kerja Tim (Rekomendasi)

1. **Branch per fitur**: `feat/guidance-pid`, `feat/obstacle-yolo`, dst. Jangan langsung push ke `main`.
2. **Testing di simulasi dulu** (`simulation`, ArduPilot SITL) sebelum diuji di kapal fisik/kolam.
3. **Satu orang pegang `mavros_bridge` & `mission`** (interface kritis), lainnya bisa paralel di `guidance_control` / `perception`.
4. **Isi `docs/`** dengan wiring diagram, daftar topic/service, dan catatan kalibrasi kompas/GPS — penting banget buat laporan teknis & saat ditanya juri.
5. Pasang **pre-commit / CI** (`.github/workflows/ci.yml`) supaya build error ketauan sebelum H-1 lomba.

## Rulebook Reference

Sesuaikan task (navigation channel, obstacle avoidance, docking, dst) dan
scoring dengan **Buku Panduan KKI 2026** kategori ASV dari BELMAWA/Puspresnas.
Simpan salinan PDF-nya di `docs/rulebook/`.
