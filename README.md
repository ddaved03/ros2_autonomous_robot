# Autonomous Robot – ROS 2 + Gazebo + Foxglove Monitoring

Ez a projekt egy **ROS 2 Humble** alapú autonóm robot rendszert valósít meg
**TurtleBot3** szimulációval.  
A robot **LaserScan** szenzoradatok alapján érzékeli az akadályokat,
kiszámolja a legközelebbi akadály távolságát, ütközésveszélyt jelez,
és **Foxglove Studio** segítségével grafikus felületen monitorozható.

A projekt az  
**Autonóm járművek és robotok programozása** tantárgy  
**nagy féléves beadandójaként** készült.

---

## Funkciók

- TurtleBot3 szimuláció Gazebo-ban (headless mód)
- LaserScan feldolgozás (`/scan`)
- Minimum akadálytávolság számítás (`/min_distance`)
- Ütközésveszély jelzés (`/collision_warning`)
- Egyszerű autonóm vezérlés (`/cmd_vel`)
- Statikus TF kapcsolat (`base_footprint → base_scan`)
- Foxglove Studio vizualizáció:
  - LaserScan 3D nézet
  - Sebesség grafikonok
  - Biztonsági állapot indikátor

---

## Követelmények

### Operációs rendszer
- Ubuntu 22.04  
  **vagy**
- Windows + WSL2 (Ubuntu 22.04)

### Szoftverek
- ROS 2 **Humble**
- Gazebo Classic
- TurtleBot3 csomagok
- Foxglove Studio

---

## Szükséges csomagok telepítése

```
sudo apt update
sudo apt install ros-humble-turtlebot3* ros-humble-gazebo-ros-pkgs
```

###TurtleBot3 modell beállítása:

```
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc

```

###Workspace build

```
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build

```

##Program futtatása – lépésről lépésre
###A rendszer KÉT terminált igényel.
###Mindkét terminált külön kell megnyitni.

##Terminál 1 – Robot, szimuláció és vezérlés
Ebben a terminálban indul el a teljes robot rendszer.

1. Környezet betöltése

```
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
source install/setup.bash
```

2. Teljes rendszer indítása (egyetlen parancs)

```
ros2 launch autonomous_robot tb3_headless.launch.py
```

Ez a launch fájl automatikusan elindítja:

- Gazebo szimulációt (headless)
- TurtleBot3 modellt
- robot_state_publisher node-ot
- statikus TF kapcsolatot (base_footprint → base_scan)
- autonóm vezérlőt (controller_node)
- biztonsági réteget (min_distance_node)
- /cmd_vel monitor node-ot

3. Ellenőrzés

```
ros2 topic list | grep -E "scan|min_distance|collision|cmd_vel"
```

Elvárt topicok:

- /scan
- /cmd_vel
- /min_distance
- /collision_warning


##Terminál 2 – Foxglove Bridge
Ez a terminál a Foxglove Studio kapcsolathoz szükséges.

1. Környezet betöltése

```
source /opt/ros/humble/setup.bash
```

2. Foxglove Bridge indítása

```
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```
WebSocket cím:

```
ws://localhost:8765
```

Ellenőrzés:

```
ss -lntp | grep 8765
```

##Foxglove Studio használata
1. Indítsd el a Foxglove Studio-t
2. Connect → Foxglove WebSocket
3. Add meg a címet:

```
ws://localhost:8765
```

##Ajánlott panelek
###3D panel

- Fixed frame: odom vagy base_footprint
- Bekapcsolva: /scan

###Plot panel

- /cmd_vel.linear.x
- /cmd_vel.angular.z
- /min_distance.data

##Indicator panel

- /collision_warning.data
- true → piros (STOP)
- false → zöld (OK)

##Működési logika

- A LaserScan adatból kiszűrésre kerülnek:
	- NaN értékek
	- végtelen (inf) értékek
	- range_min / range_max kívüli mérések
- Meghatározásra kerül a legkisebb érvényes távolság
- Ha min_distance < stop_distance:
	- collision_warning = true
	- a robot megáll és fordul
- Egyébként:
	- a robot előre halad

##Gyakori hibák
###LaserScan nem látszik Foxglove-ban

```
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint base_scan
```

###Nem jelenik meg a min_distance

```
ros2 topic echo /min_distance --once
```
