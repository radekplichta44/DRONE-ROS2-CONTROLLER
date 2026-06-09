#!/bin/bash

source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash" 2>/dev/null || true
source "$HOME/lidar_slam_ws/install/setup.bash" 2>/dev/null || true

MAPS_DIR="$HOME/lidar_slam_ws/maps"
TEMP_FILE="/tmp/current_slam_map.txt"

echo "Podaj nazwe mapy (bez spacji, np. pokoj_1):"
read -r map_name
MAP_NAME=$(echo "$map_name" | tr -d ' ' | tr -dc '[:alnum:]_.-')

if [ -z "$MAP_NAME" ]; then
    echo "Nie podano poprawnej nazwy. Anulowano."
    exit 1
fi

echo "$MAP_NAME" > "$TEMP_FILE"

MAP_DIR_PATH="$MAPS_DIR/$MAP_NAME"
mkdir -p "$MAP_DIR_PATH"
FULL_MAP_PATH="$MAP_DIR_PATH/$MAP_NAME"

echo "=== ROZPOCZYNAM ZAPIS KOMPLETU DANYCH ==="

# KROK 1: Zapis obrazu .pgm i .yaml 
echo "1. Generowanie obrazu PGM i pliku YAML..."

WAIT_TIMEOUT=15
WAITED=0
while ! ros2 service list 2>/dev/null | grep -q '/slam_toolbox/save_map' && [ "$WAITED" -lt "$WAIT_TIMEOUT" ]; do
    sleep 1
    WAITED=$((WAITED+1))
done
if ! ros2 service list 2>/dev/null | grep -q '/slam_toolbox/save_map'; then
    echo "ERROR: service /slam_toolbox/save_map not available. Upewnij sie, ze SLAM Toolbox dziala."
    exit 2
fi

ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '$FULL_MAP_PATH'}}"

# Give SLAM Toolbox a moment to write files
sleep 1

# KROK 1b: Konwersja PGM na PNG 
echo "1b. Konwersja PGM na PNG..."
if [ -f "$FULL_MAP_PATH.pgm" ]; then
    if command -v convert >/dev/null 2>&1; then
        convert "$FULL_MAP_PATH.pgm" -quality 100 "$FULL_MAP_PATH.png"
        echo " Konwersja udana: $FULL_MAP_PATH.png"
    else
        echo " Brak programu 'convert'. Zainstaluj go: sudo apt install imagemagick"
    fi
else
    echo " Plik PGM nie został wygenerowany przez SLAM Toolbox!"
fi

# KROK 2: Zapis sesji .data i .posegraph
echo "2. Zapisywanie sesji SLAM (.data, .posegraph)..."

WAITED=0
while ! ros2 service list 2>/dev/null | grep -q '/slam_toolbox/serialize_map' && [ "$WAITED" -lt "$WAIT_TIMEOUT" ]; do
    sleep 1
    WAITED=$((WAITED+1))
done
if ! ros2 service list 2>/dev/null | grep -q '/slam_toolbox/serialize_map'; then
    echo "ERROR: service /slam_toolbox/serialize_map not available. Upewnij sie, ze SLAM Toolbox dziala."
    exit 3
fi

ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$FULL_MAP_PATH'}"

sleep 1

echo ""
echo "=== ZAPIS ZAKOŃCZONY W $MAP_DIR_PATH ==="
echo "Znalezione pliki:"
ls -lh "$MAP_DIR_PATH"

if [ ! -f "$FULL_MAP_PATH.pgm" ]; then
    echo " Brak pliku PGM: $FULL_MAP_PATH.pgm"
fi
if [ ! -f "$FULL_MAP_PATH.yaml" ]; then
    echo " Brak pliku YAML: $FULL_MAP_PATH.yaml"
fi
if [ ! -f "$FULL_MAP_PATH.posegraph" ] && [ ! -f "$FULL_MAP_PATH.data" ]; then
    echo " Brak plikow sesji (.posegraph lub .data) dla: $FULL_MAP_PATH"
fi