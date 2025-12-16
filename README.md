# ROS 2 alapú autonóm robot akadálykerüléssel

Ez a projekt egy ROS 2 Humble alapú autonóm mobilrobot vezérlőt valósít meg, 
amely lézerszkenner (LaserScan) adatok alapján képes egyszerű akadálykerülésre.
A megoldás TurtleBot3 roboton, Gazebo szimulációban került megvalósításra.

A projekt célja az autonóm járművek és robotok programozásához szükséges alapvető
ROS 2 kommunikációs mechanizmusok (publisher, subscriber, launch fájlok) gyakorlása.

---

## Funkcionalitás

- TurtleBot3 robot szimuláció Gazebo környezetben (headless módban)
- LaserScan adatok feldolgozása (`/scan`)
- Egyszerű döntési logika:
  - akadály esetén fordulás
  - szabad térben előrehaladás
- Sebességparancsok publikálása `/cmd_vel` topicra
- Részletes konzolos logolás a döntésekről

---

## Követelmények

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo (ROS 2 integrációval)
- turtlebot3 csomagok

Szükséges csomagok telepítése:
```bash
sudo apt update
sudo apt install ros-humble-turtlebot3* ros-humble-gazebo-ros-pkgs

