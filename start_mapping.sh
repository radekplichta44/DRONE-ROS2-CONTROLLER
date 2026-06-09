#!/bin/bash

# --- KONFIGURACJA ŚCIEŻEK ---
WS_PATH="$HOME/lidar_slam_ws"
MAPS_DIR="$WS_PATH/maps"
YAML_PARAMS="$WS_PATH/mapper_params_mapping.yaml"
URDF_MODEL="$WS_PATH/moj_robot.urdf"

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
source $WS_PATH/install/setup.bash
mkdir -p $MAPS_DIR

clear
echo "================================================================"
echo "        ROBOT CONTROL CENTER - SLAM TOOLBOX (JAZZY)"
echo "================================================================"
echo " 1) NOWA MAPA (Start od zera)"
echo " 2) KONTYNUUJ MAPOWANIE (Wczytaj istniejącą sesję)"
echo " 3) TYLKO LOKALIZACJA (Wczytaj mapę bez jej zmieniania)"
echo " q) WYJŚCIE"
echo "----------------------------------------------------------------"
read -p "Wybierz opcję: " CHOICE

if [[ "$CHOICE" == "q" ]]; then exit 0; fi

read -p "Podaj nazwę mapy/sesji: " MAP_NAME
TEMP_FILE="/tmp/current_slam_map.txt"
echo "$MAP_NAME" > "$TEMP_FILE"

# Logika ładowania mapy
LOAD_PARAMS=""
if [[ "$CHOICE" == "2" || "$CHOICE" == "3" ]]; then
    FULL_PATH="$MAPS_DIR/$MAP_NAME/$MAP_NAME"
    if [ -f "$FULL_PATH.data" ]; then
        echo "✓ Znaleziono pliki sesji: $MAP_NAME"
        LOAD_PARAMS="-p map_file_name:=$FULL_PATH -p map_start_at_dock:=true"
        if [[ "$CHOICE" == "3" ]]; then
            LOAD_PARAMS="$LOAD_PARAMS -p mode:=localization"
        fi
    else
        echo " BŁĄD: Nie znaleziono pliku $FULL_PATH.data"
        exit 1
    fi
fi

echo "Synchronizacja zegara..."
sudo hwclock -s 2>/dev/null || true 

echo "Uruchamiam LiDAR..."
ros2 run sllidar_ros2 sllidar_node --ros-args -p serial_port:=/dev/ttyUSB0 -p serial_baudrate:=256000 -p frame_id:=laser -p angle_compensate:=true -p scan_mode:=Sensitivity &
sleep 2

echo "Uruchamiam Odometrię Laserową (rf2o)..."
ros2 run rf2o_laser_odometry rf2o_laser_odometry_node \
  --ros-args \
  -p laser_scan_topic:=/scan \
  -p odom_topic:=/odom \
  -p publish_tf:=true \
  -p base_frame_id:=base_link \
  -p odom_frame_id:=odom &

echo "Uruchamiam URDF..."
ros2 run robot_state_publisher robot_state_publisher $URDF_MODEL &

echo "Uruchamiam SLAM Toolbox..."
# Wczytujemy SLAM z opcjonalnymi parametrami ładowania mapy
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  --params-file $YAML_PARAMS \
  -p use_sim_time:=false $LOAD_PARAMS &

sleep 4
echo "Aktywacja węzła SLAM..."
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate

echo "Uruchamiam Foxglove..."
ros2 run foxglove_bridge foxglove_bridge &

echo "================================================================"
echo " SESJA: $MAP_NAME DZIAŁA."
echo " [ENTER] - Zapisz postęp (wywołuje zapisz_mape.sh)"
echo " [ESC]   - Zamknij system"
echo "================================================================"

while true; do
    read -rsn1 key
    if [[ "$key" == "" ]]; then
        echo -e "\n--- Wywołuję zapis dla: $MAP_NAME ---"
        $WS_PATH/zapisz_mape.sh
    elif [[ "$key" == $'\e' ]]; then
        break 
    fi
done

echo "Wyłączanie..."
pkill -INT -f "sllidar_node"
pkill -INT -f "foxglove_bridge"
pkill -9 -f "slam_toolbox"
kill -INT $(jobs -p) 2>/dev/null
exit 0