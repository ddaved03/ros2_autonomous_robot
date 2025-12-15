# ROS2 autonóm robot – akadályészlelés és egyszerű akadálykerülés (AJR nagy féléves)

## Rövid leírás
A projekt célja egy ROS 2 Humble alapú demonstráció elkészítése, ahol egy (szimulált) LiDAR szenzoradat (/scan) alapján a rendszer döntést hoz, és sebességparancsot publikál (/cmd_vel).
A megoldás több node-ból áll, ROS2 topic kommunikációval, és launch fájllal egy parancsból indítható.

## Funkciók
- `/scan` LaserScan feldolgozás (minimális távolság meghatározása)
- Egyszerű vezérlő logika:
  - ha akadály közel van → fordulás
  - ha szabad a tér → előrehaladás
- `/cmd_vel` kiadása és monitorozása
- Egy parancsos indítás `ros2 launch` segítségével

## Rendszerfelépítés (node-ok és topic-ok)
**Node-ok:**
- `fake_scan_publisher` – teszt LaserScan adatok publikálása `/scan`-ra
- `controller_node` – döntési logika `/scan` alapján, publikál `/cmd_vel`-re
- `cmd_vel_monitor` – `/cmd_vel` monitorozása és logolása
- `sensor_node` – `/scan` minimális távolság logolása (külön futtatható)

**Topic-ok:**
- `/scan` (`sensor_msgs/msg/LaserScan`)
- `/cmd_vel` (`geometry_msgs/msg/Twist`)

Egyszerű adatfolyam:
`fake_scan_publisher` → `/scan` → `controller_node` → `/cmd_vel` → `cmd_vel_monitor`

## Követelmények
- Ubuntu 22.04 / WSL2
- ROS 2 Humble
- colcon

## Build
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select autonomous_robot
source install/setup.bash

