# burkut-sim

Değişken Kanat Alanlı (Morphing) İHA simülasyon workspace'i.
**Stack:** ROS 2 Humble + PX4 v1.17 + Gazebo Harmonic

## Sistem Gereksinimleri

| Bileşen | Versiyon |
|---|---|
| Ubuntu | 22.04 LTS |
| ROS 2 | Humble Desktop |
| Gazebo | Harmonic (gz-sim8) |
| PX4-Autopilot | `~/PX4-Autopilot` (`px4_sitl_default` derlenmiş) |
| px4_msgs | `~/ros2_ws/` altında derlenmiş |

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
| `burkut_description` | cmake | UAV SDF modeli (`advanced_plane`) |
| `burkut_worlds` | cmake | Gazebo world dosyaları (`test` world) |
| `burkut_bringup` | cmake | Launch dosyaları ve GUI |
| `burkut_perception` | python | Mock stereo → `/burkut/obstacles` publisher |
| `burkut_planning` | python | Rota planlama ve engel kaçınma |
| `burkut_control` | python | PX4 offboard kontrol arayüzü |

## Araçlar

```bash
source tools/activate.sh      # Ortamı aktive et
./tools/run_sim.sh             # Simülasyonu tmux'ta başlat (4 panel)
./tools/run_sim.sh stop        # Tüm simülasyon bileşenlerini kapat
./tools/check_health.sh        # Sistem sağlığını kontrol et
./tools/dump_runtime.sh        # Anlık sistem snapshot'ı al (runs/ altına)
```

## Simülasyon Başlatma

### Otomatik (Önerilen) — `run_sim.sh`

```bash
./tools/run_sim.sh        # Tüm bileşenleri tmux'ta başlat
./tools/run_sim.sh stop   # Her şeyi durdur ve temizle
```

Script, `burkut_sim` adında bir tmux session açar ve 3 window'a bölünür:

#### Window 0 — Core (4 panel, tiled layout)

```
┌─────────────────────┬─────────────────────┐
│  0.0  Core          │  0.1  DDS           │
│  Gazebo + ROS 2     │  MicroXRCEAgent     │
│  main.launch.xml    │  udp4 -p 8888       │
├─────────────────────┼─────────────────────┤
│  0.2  PX4 SITL      │  0.3  Shell         │
│  (8sn gecikmeyle    │  activate.sh ile    │
│  başlar)            │  interaktif bash    │
└─────────────────────┴─────────────────────┘
```

| Panel | İçerik | Detay |
|---|---|---|
| `0.0` Core | Gazebo + ROS 2 | `ros2 launch burkut_bringup main.launch.xml` |
| `0.1` DDS | uXRCE-DDS Agent | PX4 ↔ ROS 2 köprüsü, UDP 8888 |
| `0.2` PX4 | PX4 SITL | 8 sn bekledikten sonra başlar; `PX4_GZ_STANDALONE=1`, `AUTOSTART=4008`, world=`test`, model=`advanced_plane` |
| `0.3` Shell | İnteraktif bash | `activate.sh` source edilmiş, repo kökünde açılır |

#### Window 1 — Health

`check_health.sh`'i 5 saniyede bir döngüyle çalıştırır. Bileşenlerin ayakta olup olmadığını sürekli izler.

#### Window 2 — QGC

`~/QGroundControl.AppImage`'ı 8 saniye bekledikten sonra başlatır. Farklı bir yol kullanmak için:

```bash
QGC_PATH=/path/to/QGroundControl.AppImage ./tools/run_sim.sh
```

#### Loglar

Her çalıştırmada `runs/YYYY-MM-DD-HH-MM-SS/` altına otomatik log dosyaları oluşturulur:

```
runs/
└── 2026-03-29-14-30-00/
    ├── core.log
    ├── dds.log
    ├── px4.log
    ├── shell.log
    ├── health.log
    └── qgc.log
```

#### tmux Kısayolları

| Kısayol | Açıklama |
|---|---|
| `Ctrl+B` → `0/1/2` | Window'lar arası geçiş |
| `Ctrl+B` → yön tuşları | Paneller arası geçiş |
| `Ctrl+B` → `d` | Session'dan çık (arka planda çalışmaya devam eder) |
| `tmux attach -t burkut_sim` | Tekrar bağlan |

---

### Manuel Başlatma

**Terminal 1 — Gazebo + ROS 2:**
```bash
source tools/activate.sh
ros2 launch burkut_bringup main.launch.xml
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
export PX4_GZ_WORLD=test
export PX4_GZ_MODEL_NAME=advanced_plane
./build/px4_sitl_default/bin/px4
```

## Branch Stratejisi

| Branch | Açıklama |
|---|---|
| `main` | Korumalı, sadece PR ile merge |
| `dev` | Geliştirme entegrasyonu |
| `feat/*` | Özellik dalları |


Ahmet Buğra Kallat
