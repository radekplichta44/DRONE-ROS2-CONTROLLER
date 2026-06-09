#!/bin/bash

# 1. Wczytanie środowisk ROS 2
source /opt/ros/jazzy/setup.bash
source /home/subaruu/ros2_ws/install/setup.bash
source /home/subaruu/lidar_slam_ws/install/setup.bash

# 2. Czysty start (ubicie ewentualnych starych procesów i duchów)
pkill -9 -f joy_node
pkill -9 -f teleop_node
pkill -9 -f pad_lasery.py
pkill -9 -f pilot.py
pkill -9 -f ros_robot_controller
sleep 2

# 3. Uruchomienie całego ekosystemu w tle
# Główny panel WWW i kinematyka silników
python3 /home/subaruu/pilot.py &

# Obsługa pada Bluetooth (strefa nieczułości 10% na naturalne wychylenia gałek)
ros2 run joy joy_node --ros-args -p deadzone:=0.1 &

# Tłumacz pada na wektory ruchu dla kół Mecanum 
ros2 run teleop_twist_joy teleop_node --ros-args -p require_enable_button:=false -p axis_linear.x:=1 -p axis_linear.y:=0 -p axis_angular.yaw:=3 &



# Czekanie na zakończenie procesów (utrzymuje skrypt przy życiu w systemie)
wait