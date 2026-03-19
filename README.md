# burkut-sim

Değişken Kanat Alanlı (Morphing) İHA simülasyon workspace'i.
**Stack:** ROS 2 Humble + PX4 v1.17 + Gazebo Harmonic

## Sistem Gereksinimleri

| Bileşen | Versiyon |
|---|---|
| Ubuntu | 22.04 LTS |
| ROS 2 | Humble Desktop |
| Gazebo | Harmonic (gz-sim8) |
| PX4-Autopilot | ~/PX4-Autopilot (px4_sitl_default derlenmiş) |
| px4_msgs / px4_ros_com | ~/ros2_ws/ altında derlenmiş |

## Kurulum

```bash
git clone https://github.com/<ORG>/burkut-sim.git
cd burkut-sim
source tools/activate.sh
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

## Paket Yapısı

| Paket | Tür | Açıklama |
|---|---|---|
| `burkut_msgs` | cmake | Custom mesaj ve servis tanımları |
| `burkut_description` | cmake | UAV SDF/URDF modeli (burkut_plane) |
| `burkut_worlds` | cmake | Gazebo world dosyaları (direkli parkur) |
| `burkut_bringup` | cmake | Launch dosyaları |
| `burkut_perception` | python | Mock stereo → `/burkut/obstacles` publisher |
| `burkut_planning` | python | Rota planlama ve engel kaçınma |
| `burkut_control` | python | PX4 offboard kontrol arayüzü |

## Araçlar

```bash
source tools/activate.sh      # Ortamı aktive et
./tools/run_sim.sh             # Simülasyonu tmux'ta başlat (4 panel)
./tools/check_health.sh        # Sistem sağlığını kontrol et
./tools/dump_runtime.sh        # Anlık sistem snapshot'ı al (runs/ altına)
```

## Simülasyon Başlatma (Manuel)

**Terminal 1 — Gazebo + ROS 2:**
```bash
source tools/activate.sh
ros2 launch burkut_bringup sim.launch.py
```

**Terminal 2 — uXRCE-DDS Agent (PX4 ↔ ROS 2 köprüsü):**
```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 3 — PX4 SITL:**
```bash
cd ~/PX4-Autopilot
export PX4_GZ_STANDALONE=1
export PX4_SYS_AUTOSTART=4008
export PX4_GZ_WORLD=burkut_poles
export PX4_GZ_MODEL_NAME=burkut_plane
./build/px4_sitl_default/bin/px4
```

## Branch Stratejisi

| Branch | Açıklama |
|---|---|
| `main` | Korumalı, sadece PR ile merge |
| `dev` | Geliştirme entegrasyonu |
| `feat/*` | Özellik dalları |


 Ahmet Buğra Kallat 
