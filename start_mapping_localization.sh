#!/bin/bash

WS_PATH="$HOME/lidar_slam_ws"

source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
source "$WS_PATH/install/setup.bash"

echo "Synchronizacja zegara..."
# Dodano ignorowanie błędów, jeśli malinka nie ma sprzętowego modułu RTC (Hardware Clock)
sudo hwclock -s 2>/dev/null || true 

MAPS_DIR="$WS_PATH/maps"

echo "========================================="
echo " WYBIERZ MAPĘ DO LOKALIZACJI"
echo "========================================="
echo -e "Dostępne mapy:"
ls -d "$MAPS_DIR"/*/ 2>/dev/null | xargs -n 1 basename
echo "-----------------------------------------"
read -p "Podaj nazwę mapy: " map_name

MAP_PATH="$MAPS_DIR/$map_name/$map_name"

if [ ! -f "${MAP_PATH}.posegraph" ]; then
    echo "BŁĄD Nie znaleziono plików mapy w folderze $MAPS_DIR/$map_name/"
    exit 1
fi

echo "Wczytywanie mapy z folderu: $map_name"

echo "Uruchamiam LiDAR..."
ros2 launch sllidar_ros2 sllidar_a2m12_launch.py serial_port:=/dev/ttyUSB0 frame_id:=laser &

echo "Uruchamiam Odometrię Laserową (rf2o)..."
ros2 run rf2o_laser_odometry rf2o_laser_odometry_node \
  --ros-args \
  -p laser_scan_topic:=/scan \
  -p odom_topic:=/odom \
  -p publish_tf:=true \
  -p base_frame_id:=base_link \
  -p odom_frame_id:=odom &

echo "Uruchamiam Model Robota (URDF)..."
ros2 run robot_state_publisher robot_state_publisher "$WS_PATH/moj_robot.urdf" &

echo "Uruchamiam SLAM Toolbox w trybie LOKALIZACJI..."
ros2 run slam_toolbox localization_slam_toolbox_node --ros-args \
    --params-file "$WS_PATH/mapper_params_map_loc.yaml" \
  -p use_sim_time:=false \
  -p map_file_name:=$MAP_PATH &

echo "Uruchamiam most Foxglove (zamiast RViz2)..."
ros2 run foxglove_bridge foxglove_bridge &

# --- PĘTLA STEROWANIA Z KLAWIATURY ---

echo "================================================================"
echo " SYSTEM LOKALIZACJI I MAPOWANIA DZIAŁA."
echo " [ENTER] - Zapisz mape"
echo " [ESC]   - Zamknij program i wyłącz lidar"
echo "================================================================"

while true; do
    read -rsn1 key
    # ENTER 
    if [[ "$key" == "" ]]; then
        echo -e "\n Uruchamiam zapis..."
        
        "$WS_PATH/zapisz_mape.sh"
        
        echo -e "\n================================================================"
        echo " [ENTER] - Zapisz postęp mapy ponownie"
        echo " [ESC]   - Zamknij program i wyłącz lidar"
        echo "================================================================"
    # ESC
    elif [[ "$key" == $'\e' ]]; then
        echo -e "\n Rozpoczynam wyłączanie..."
        break 
    fi
done

echo "Zamykanie procesów..."
pkill -INT -f "sllidar_node"
pkill -INT -f "sllidar_a2m12_launch"
sleep 3
kill -INT $(jobs -p) 2>/dev/null
sleep 2
pkill -9 -f "rviz2"
pkill -9 -f "slam_toolbox"

echo "Programy wyłączone."
exit 0