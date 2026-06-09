from pathlib import Path
import ctypes
import sys
import threading
import time
import socket
import subprocess
import os
import signal
from datetime import datetime
from flask import Flask, jsonify, request

import evdev
from evdev import ecodes, ff

# --- KONFIGURACJA SCIEZEK ---
SCRIPT_DIR = Path(__file__).resolve().parent
ROS_PATHS = [
    Path('/opt/ros/jazzy/lib/python3.12/site-packages'),
    Path.home() / 'ros2_ws/install/ros_robot_controller_msgs/lib/python3.12/site-packages',
    Path.home() / 'ros2_ws/install/ros_robot_controller/lib/python3.12/site-packages',
]
for path in ROS_PATHS:
    if path.exists() and str(path) not in sys.path:
        sys.path.insert(0, str(path))

ROS_LIB_DIR = Path.home() / 'ros2_ws/install/ros_robot_controller_msgs/lib'
if ROS_LIB_DIR.exists():
    for lib_path in sorted(ROS_LIB_DIR.glob('*.so')):
        try: ctypes.CDLL(str(lib_path), mode=ctypes.RTLD_GLOBAL)
        except Exception: pass

import rclpy
from ros_robot_controller_msgs.msg import MotorsState, MotorState
from geometry_msgs.msg import Twist  
from std_msgs.msg import String      
from sensor_msgs.msg import Joy      

# --- INICJALIZACJA ROS 2 ---
if not rclpy.ok(): rclpy.init()
node = rclpy.create_node('subaru_web_controller')
pub = node.create_publisher(MotorsState, '/ros_robot_controller/set_motor', 10)
web_pub = node.create_publisher(Twist, '/cmd_vel', 10)

app = Flask(__name__)

# --- ZARZADZANIE PROCESAMI SLAM ---
slam_process_group = None
system_status = {"lidar": False, "slam": False}
slam_was_ready = False  

def kill_slam_processes():
    global slam_process_group
    subprocess.run("pkill -INT -f sllidar", shell=True)
    time.sleep(1.5)
    if slam_process_group:
        try: os.killpg(slam_process_group, signal.SIGTERM)
        except: pass
        slam_process_group = None
    subprocess.run("pkill -9 -f slam_toolbox; pkill -9 -f foxglove_bridge; pkill -9 -f robot_state_publisher; pkill -9 -f static_transform", shell=True)

def generate_bash_script(mode, map_name):
    script = f"""#!/bin/bash
    source /opt/ros/jazzy/setup.bash
    source $HOME/ros2_ws/install/setup.bash
    source $HOME/lidar_slam_ws/install/setup.bash
    sudo hwclock -s 2>/dev/null || true

    echo "1. LiDAR..."
    ros2 run sllidar_ros2 sllidar_node --ros-args -p serial_port:=/dev/ttyUSB0 -p serial_baudrate:=256000 -p frame_id:=laser -p angle_compensate:=true -p scan_mode:=Sensitivity &
    sleep 2
    
    echo "2. URDF i Pancerna Odometria..."
    ros2 run robot_state_publisher robot_state_publisher $HOME/lidar_slam_ws/moj_robot.urdf &
    ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 --frame-id odom --child-frame-id base_link &
    ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 --frame-id base_link --child-frame-id base_footprint &
    """

    if mode == 'mapping':
        script += """
        echo "3. SLAM Mapowanie od zera..."
        ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file $HOME/lidar_slam_ws/mapper_params_mapping.yaml -p use_sim_time:=false &
        """
    elif mode == 'continue':
        map_path = f"$HOME/lidar_slam_ws/maps/{map_name}/{map_name}"
        script += f"""
        echo "3. SLAM Kontynuacja mapowania..."
        ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file $HOME/lidar_slam_ws/mapper_params_mapping.yaml -p use_sim_time:=false -p map_file_name:={map_path} -p map_start_at_dock:=true &
        """

    script += """
    sleep 4
    ros2 lifecycle set /slam_toolbox configure
    ros2 lifecycle set /slam_toolbox activate
    echo "4. Foxglove..."
    ros2 run foxglove_bridge foxglove_bridge &
    wait
    """
    with open("/tmp/run_slam_web.sh", "w") as f: f.write(script)
    os.chmod("/tmp/run_slam_web.sh", 0o755)

# --- SYSTEM WIBRACJI ---
def wibruj_padem(wzorzec="zapis"):
    def wibruj_watek():
        try:
            pad_device = None
            for path in evdev.list_devices():
                dev = evdev.InputDevice(path)
                if ecodes.EV_FF in dev.capabilities():
                    pad_device = dev
                    break
            
            if not pad_device:
                return

            rumble = ff.Rumble(strong_magnitude=0xFFFF, weak_magnitude=0xFFFF)

            def zagraj(czas_sekundy):
                effect = ff.Effect(
                    ecodes.FF_RUMBLE, -1, 0,
                    ff.Trigger(0, 0),
                    ff.Replay(int(czas_sekundy * 1000), 0),
                    ff.EffectType(ff_rumble_effect=rumble)
                )
                effect_id = pad_device.upload_effect(effect)
                pad_device.write(ecodes.EV_FF, effect_id, 1)
                time.sleep(czas_sekundy)
                pad_device.erase_effect(effect_id)

            if wzorzec == "start": zagraj(0.3)
            elif wzorzec == "slam": zagraj(0.2); time.sleep(0.15); zagraj(0.2)
            else: zagraj(0.7)
                
        except Exception as e: pass
            
    threading.Thread(target=wibruj_watek).start()

def status_checker():
    global slam_was_ready
    while True:
        try:
            out = subprocess.check_output("source /opt/ros/jazzy/setup.bash && ros2 topic list", shell=True, executable='/bin/bash').decode()
            system_status['lidar'] = '/scan' in out
            is_slam_active = '/map' in out
            system_status['slam'] = is_slam_active
            
            if is_slam_active and not slam_was_ready:
                node.get_logger().info('SLAM GOTOWY! Mozna otwierac Foxglove.')
                wibruj_padem(wzorzec="slam")
                
            slam_was_ready = is_slam_active
        except: pass
        time.sleep(2)

# --- ZAPIS MAPY ---
def execute_save(map_name):
    node.get_logger().info(f'System zapisu: Zapisuje mape jako {map_name}...')
    map_dir = f"{Path.home()}/lidar_slam_ws/maps/{map_name}"
    os.makedirs(map_dir, exist_ok=True)
    full_path = f"{map_dir}/{map_name}"
    
    setup_cmd = f"source /opt/ros/jazzy/setup.bash && source {Path.home()}/ros2_ws/install/setup.bash 2>/dev/null || true && source {Path.home()}/lidar_slam_ws/install/setup.bash 2>/dev/null || true"
    cmd_save = f"{setup_cmd} && ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \"{{name: {{data: '{full_path}'}}}}\""
    cmd_serialize = f"{setup_cmd} && ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \"{{filename: '{full_path}'}}\""
    
    subprocess.run(cmd_serialize, shell=True, executable='/bin/bash')
    subprocess.run(cmd_save, shell=True, executable='/bin/bash')
    
    node.get_logger().info(f'ZAPISANO MAPE: {map_name}')
    wibruj_padem(wzorzec="zapis")

def foxglove_cmd_callback(msg):
    if msg.data.startswith("save:"):
        map_name = msg.data.split("save:")[1].strip()
        threading.Thread(target=execute_save, args=(map_name,)).start()

node.create_subscription(String, '/mc900/commands', foxglove_cmd_callback, 10)

# --- STEROWANIE SILNIKAMI ---
state_lock = threading.Lock()
target_speeds = [0.0]*4
current_speeds = [0.0]*4
speed_mult_linear = 2.0
speed_mult_strafe = 1.6
speed_mult_angular = 1.3
last_dpad_ud = 0.0
last_dpad_lr = 0.0
last_btn_triangle = 0 
pad_connected = False 

def joy_callback(msg):
    global speed_mult_linear, speed_mult_strafe, speed_mult_angular
    global last_dpad_ud, last_dpad_lr, last_btn_triangle
    global slam_process_group, pad_connected 
    
    if not pad_connected:
        if pub.get_subscription_count() > 0:
            node.get_logger().info('SYSTEM GOTOWY! Masz 100% kontroli nad robotem (Z PADA).')
            wibruj_padem(wzorzec="start")
            pad_connected = True
        else:
            return

    if len(msg.axes) >= 8 and len(msg.buttons) >= 4:
        if msg.buttons[1] == 1 and slam_process_group is None:
            kill_slam_processes()
            generate_bash_script('mapping', 'mapa_z_pada')
            p = subprocess.Popen(["/tmp/run_slam_web.sh"], preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            slam_process_group = p.pid

        if msg.buttons[0] == 1 and slam_process_group is not None:
            kill_slam_processes()
            slam_process_group = None

        current_btn_triangle = msg.buttons[2]
        if current_btn_triangle == 1 and last_btn_triangle == 0:
            if slam_process_group is not None:
                map_name = datetime.now().strftime("mapa_%Y-%m-%d_%H-%M-%S")
                threading.Thread(target=execute_save, args=(map_name,)).start()
        last_btn_triangle = current_btn_triangle

        current_lr = msg.axes[6]
        current_ud = msg.axes[7]
        if current_ud > 0.5 and last_dpad_ud <= 0.5:
            speed_mult_linear = round(speed_mult_linear + 0.1, 1)
            speed_mult_strafe = round(speed_mult_strafe + 0.1, 1)
        elif current_ud < -0.5 and last_dpad_ud >= -0.5:
            speed_mult_linear = max(0.1, round(speed_mult_linear - 0.1, 1))
            speed_mult_strafe = max(0.1, round(speed_mult_strafe - 0.1, 1))
            
        if current_lr < -0.5 and last_dpad_lr >= -0.5:
            speed_mult_angular = round(speed_mult_angular + 0.1, 1)
        elif current_lr > 0.5 and last_dpad_lr <= 0.5:
            speed_mult_angular = max(0.1, round(speed_mult_angular - 0.1, 1))

        last_dpad_ud = current_ud
        last_dpad_lr = current_lr

node.create_subscription(Joy, '/joy', joy_callback, 10)

def cmd_vel_callback(msg):
    global speed_mult_linear, speed_mult_strafe, speed_mult_angular
    x = msg.linear.x * speed_mult_linear
    y = msg.linear.y * speed_mult_strafe
    z = msg.angular.z * speed_mult_angular
    with state_lock:
        target_speeds[0] = -x + y + z 
        target_speeds[1] =  x + y + z 
        target_speeds[2] = -x - y + z 
        target_speeds[3] =  x - y + z 
node.create_subscription(Twist, '/cmd_vel', cmd_vel_callback, 10)

def motor_worker():
    alpha, period, deadband = 0.22, 0.02, 0.05
    while True:
        with state_lock:
            for i in range(4):
                current_speeds[i] += alpha * (target_speeds[i] - current_speeds[i])
                if target_speeds[i] == 0 and abs(current_speeds[i]) < deadband: current_speeds[i] = 0.0
            snapshot = tuple(current_speeds)
        
        msg = MotorsState()
        for i, speed in enumerate(snapshot, 1):
            m = MotorState()
            m.id, m.rps = i, float(speed)
            msg.data.append(m)
        pub.publish(msg)
        time.sleep(period)

def launch_ros_controller():
    env = os.environ.copy()
    env['HOME'] = os.path.expanduser('~')
    cmd = 'bash -c "source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 launch ros_robot_controller ros_robot_controller.launch.xml"'
    subprocess.Popen(cmd, shell=True, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

# --- ENDPOINTY API ---
@app.route('/api/status')
def get_status(): return jsonify(system_status)

@app.route('/api/maps')
def get_maps():
    maps_dir = Path.home() / "lidar_slam_ws/maps"
    if not maps_dir.exists(): return jsonify([])
    saved_maps = [d.name for d in maps_dir.iterdir() if d.is_dir()]
    return jsonify(sorted(saved_maps))

@app.route('/api/start', methods=['POST'])
def api_start():
    global slam_process_group
    data = request.json
    kill_slam_processes()
    generate_bash_script(data['mode'], data['map_name'])
    p = subprocess.Popen(["/tmp/run_slam_web.sh"], preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    slam_process_group = p.pid
    return jsonify({"status": "started"})

@app.route('/api/stop', methods=['POST'])
def api_stop():
    kill_slam_processes()
    return jsonify({"status": "stopped"})

@app.route('/api/save', methods=['POST'])
def api_save():
    map_name = request.json['map_name']
    map_dir = f"{Path.home()}/lidar_slam_ws/maps/{map_name}"
    os.makedirs(map_dir, exist_ok=True)
    full_path = f"{map_dir}/{map_name}"
    
    setup_cmd = f"source /opt/ros/jazzy/setup.bash && source {Path.home()}/ros2_ws/install/setup.bash 2>/dev/null || true && source {Path.home()}/lidar_slam_ws/install/setup.bash 2>/dev/null || true"
    cmd_save = f"{setup_cmd} && ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \"{{name: {{data: '{full_path}'}}}}\""
    cmd_serialize = f"{setup_cmd} && ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \"{{filename: '{full_path}'}}\""
    
    try:
        subprocess.run(cmd_serialize, shell=True, executable='/bin/bash', timeout=15.0)
        subprocess.run(cmd_save, shell=True, executable='/bin/bash', timeout=15.0)
    except subprocess.TimeoutExpired:
        return jsonify({"success": False, "message": "BLAD TIMEOUT: Wezel SLAM procesuje dane zbyt dlugo."})
    
    pgm_file = Path(f"{full_path}.pgm")
    yaml_file = Path(f"{full_path}.yaml")
    data_file = Path(f"{full_path}.data")
    posegraph_file = Path(f"{full_path}.posegraph")
    
    all_saved = False
    for _ in range(10): 
        time.sleep(0.5)
        if pgm_file.exists() and yaml_file.exists() and (data_file.exists() or posegraph_file.exists()):
            all_saved = True; break
            
    if all_saved:
        wibruj_padem(wzorzec="zapis")
        return jsonify({"success": True, "path": full_path})
    else: 
        return jsonify({"success": False, "message": "Nie wygenerowano plikow."})

@app.route('/api/drive', methods=['POST'])
def api_drive():
    if pad_connected:
        return jsonify({"status": "blocked", "message": "Sterowanie z telefonu zablokowane. Uzyj fizycznego pada!"})
        
    direction = request.json.get('dir', 'stop')
    msg = Twist()
    
    if direction == 'forward': msg.linear.x = 1.0
    elif direction == 'backward': msg.linear.x = -1.0
    elif direction == 'left': msg.linear.y = 1.0  
    elif direction == 'right': msg.linear.y = -1.0 
    elif direction == 'turn_left': msg.angular.z = 1.0
    elif direction == 'turn_right': msg.angular.z = -1.0
    elif direction == 'stop': pass 
        
    web_pub.publish(msg)
    return jsonify({"status": "driving", "dir": direction})

@app.route('/api/speed', methods=['POST'])
def api_speed():
    global speed_mult_linear, speed_mult_strafe, speed_mult_angular
    if pad_connected:
        return jsonify({"status": "blocked", "message": "Sterowanie z telefonu zablokowane."})
        
    data = request.json
    target = data.get('target', 'linear')
    action = data.get('action', 'increase')
    
    if target == 'linear':
        if action == 'increase':
            speed_mult_linear = min(5.0, round(speed_mult_linear + 0.2, 1))
            speed_mult_strafe = min(5.0, round(speed_mult_strafe + 0.2, 1))
        else:
            speed_mult_linear = max(0.2, round(speed_mult_linear - 0.2, 1))
            speed_mult_strafe = max(0.2, round(speed_mult_strafe - 0.2, 1))
        return jsonify({"status": "speed_changed"})
        
    elif target == 'angular':
        if action == 'increase':
            speed_mult_angular = min(5.0, round(speed_mult_angular + 0.2, 1))
        else:
            speed_mult_angular = max(0.2, round(speed_mult_angular - 0.2, 1))
        return jsonify({"status": "speed_changed"})

# --- INTERFEJS HTML ---
HTML = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
    <title>WEB PAD KONTROLER</title>
    <style>
        body { background: #1a1a1a; color: white; font-family: 'Segoe UI', Arial, sans-serif; text-align: center; margin: 0; padding: 10px; touch-action: manipulation; }
        .panel { background: #2a2a2a; border-radius: 10px; padding: 15px; margin: 10px auto; max-width: 450px; box-shadow: 0 4px 6px rgba(0,0,0,0.3); }
        h3 { margin-top: 0; color: #ccc; font-size: 16px;}
        
        .status-bar { display: flex; justify-content: space-around; font-weight: bold; font-size: 16px; margin-bottom: 15px; }
        .dot { height: 15px; width: 15px; background-color: red; border-radius: 50%; display: inline-block; margin-right: 5px; vertical-align: middle;}
        .dot.active { background-color: #2ecc71; box-shadow: 0 0 10px #2ecc71; }
        
        /* Regulacja predkosci */
        .speed-panel { display: flex; justify-content: space-between; margin-bottom: 20px; font-size: 12px; font-weight: bold; }
        .speed-group { display: flex; flex-direction: column; align-items: center; width: 48%; background: #333; padding: 10px; border-radius: 8px; box-sizing: border-box;}
        .speed-btn-row { display: flex; gap: 10px; margin-top: 8px; }
        .speed-btn { background: #f39c12; border: none; border-radius: 5px; color: white; width: 45px; height: 35px; font-weight: bold; font-size: 18px; cursor: pointer; }
        .speed-btn:active { background: #d68910; }

        /* Uklad jazdy i obrotu */
        .drive-layout { display: flex; justify-content: space-between; align-items: center; margin-bottom: 10px; }
        .dpad { display: grid; grid-template-columns: 60px 60px 60px; grid-template-rows: 60px 60px 60px; gap: 5px; margin: 0 10px; }
        .btn-dir { background: #444; border: none; border-radius: 8px; color: white; font-size: 12px; font-weight: bold; cursor: pointer; display: flex; align-items: center; justify-content: center; user-select: none; }
        .btn-dir:active { background: #777; }
        .up { grid-column: 2; grid-row: 1; }
        .left { grid-column: 1; grid-row: 2; }
        .right { grid-column: 3; grid-row: 2; }
        .down { grid-column: 2; grid-row: 3; }
        
        .btn-rot { background: #2980b9; border: none; border-radius: 8px; color: white; width: 65px; height: 100px; font-size: 12px; font-weight: bold; cursor: pointer; user-select: none;}
        .btn-rot:active { background: #3498db; }

        input[type="text"], select { padding: 12px; border-radius: 5px; border: none; width: 100%; margin-bottom: 10px; font-size: 14px; text-align: center; box-sizing: border-box; }
        .btn-action { padding: 15px; font-size: 14px; border: none; border-radius: 5px; cursor: pointer; margin-bottom: 10px; font-weight: bold; color: white; width: 100%; box-sizing: border-box;}
        .btn-start { background: #27ae60; }
        .btn-continue { background: #2980b9; }
        .btn-save { background: #8e44ad; }
        .btn-kill { background: #c0392b; }
        hr { border: 0; height: 1px; background: #444; margin: 15px 0; }
    </style>
</head>
<body>
    <div class="panel">
        <div class="status-bar">
            <div><span id="dot-lidar" class="dot"></span>LiDAR</div>
            <div><span id="dot-slam" class="dot"></span>MAPA</div>
        </div>

        <h3>STEROWANIE JAZDA I OBROTEM</h3>
        
        <div class="speed-panel">
            <div class="speed-group">
                <span>JAZDA (PRZOD/TYL)</span>
                <div class="speed-btn-row">
                    <button class="speed-btn" onclick="changeSpeed('linear', 'decrease')">-</button>
                    <button class="speed-btn" onclick="changeSpeed('linear', 'increase')">+</button>
                </div>
            </div>
            <div class="speed-group">
                <span>OBROT ROBOTA</span>
                <div class="speed-btn-row">
                    <button class="speed-btn" onclick="changeSpeed('angular', 'decrease')">-</button>
                    <button class="speed-btn" onclick="changeSpeed('angular', 'increase')">+</button>
                </div>
            </div>
        </div>

        <div class="drive-layout">
            <button class="btn-rot" onmousedown="sendDrive('turn_left')" onmouseup="sendDrive('stop')" ontouchstart="sendDrive('turn_left')" ontouchend="sendDrive('stop')">OBROT<br>LEWO</button>
            
            <div class="dpad">
                <button class="btn-dir up" onmousedown="sendDrive('forward')" onmouseup="sendDrive('stop')" ontouchstart="sendDrive('forward')" ontouchend="sendDrive('stop')">PRZOD</button>
                <button class="btn-dir left" onmousedown="sendDrive('left')" onmouseup="sendDrive('stop')" ontouchstart="sendDrive('left')" ontouchend="sendDrive('stop')">LEWO</button>
                <button class="btn-dir right" onmousedown="sendDrive('right')" onmouseup="sendDrive('stop')" ontouchstart="sendDrive('right')" ontouchend="sendDrive('stop')">PRAWO</button>
                <button class="btn-dir down" onmousedown="sendDrive('backward')" onmouseup="sendDrive('stop')" ontouchstart="sendDrive('backward')" ontouchend="sendDrive('stop')">TYL</button>
            </div>

            <button class="btn-rot" onmousedown="sendDrive('turn_right')" onmouseup="sendDrive('stop')" ontouchstart="sendDrive('turn_right')" ontouchend="sendDrive('stop')">OBROT<br>PRAWO</button>
        </div>
    </div>

    <div class="panel">
        <h3>ZARZADZANIE MAPOWANIEM</h3>
        
        <input type="text" id="mapName" placeholder="Wpisz nazwe dla nowej mapy">
        <button class="btn-action btn-start" onclick="startSystem('mapping')">ODPAL LIDAR (NOWA MAPA)</button>
        
        <hr>
        
        <select id="mapSelect"><option value="">-- Wybierz zapisana mape --</option></select>
        <button class="btn-action btn-continue" onclick="startSystem('continue')">ZNAJDZ SIE W MAPIE (KONTYNUUJ)</button>
        
        <hr>
        
        <button class="btn-action btn-save" onclick="saveMap()">ZAPISZ MAPE</button>
        <button class="btn-action btn-kill" onclick="stopSystem()">WYLACZ LIDAR</button>
    </div>

    <script>
        let currentActiveMap = "";

        // --- FUNKCJE JAZDY I PREDKOSCI ---
        function sendDrive(direction) {
            fetch('/api/drive', { method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({dir: direction}) })
            .then(r => r.json())
            .then(d => {
                if (d.status === 'blocked' && direction !== 'stop') {
                    alert(d.message);
                }
            }).catch(() => {});
        }

        function changeSpeed(target, action) {
            fetch('/api/speed', { 
                method: 'POST', 
                headers: {'Content-Type': 'application/json'}, 
                body: JSON.stringify({target: target, action: action}) 
            })
            .then(r => r.json())
            .then(d => {
                if (d.status === 'blocked') alert(d.message);
            }).catch(() => {});
        }

        // --- FUNKCJE SLAM ---
        function loadAvailableMaps() {
            fetch('/api/maps').then(r => r.json()).then(maps => {
                let select = document.getElementById('mapSelect');
                select.innerHTML = '<option value="">-- Wybierz zapisana mape --</option>';
                maps.forEach(m => {
                    let opt = document.createElement('option');
                    opt.value = opt.innerText = m;
                    select.appendChild(opt);
                });
            });
        }

        function startSystem(mode) {
            let name = mode === 'mapping' ? document.getElementById('mapName').value.trim() : document.getElementById('mapSelect').value;
            if(!name) return alert("Wybierz lub wpisz nazwe mapy!");
            currentActiveMap = name;
            fetch('/api/start', { method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({mode: mode, map_name: name}) });
        }

        function stopSystem() { fetch('/api/stop', {method: 'POST'}).then(() => loadAvailableMaps()); }

        function saveMap() {
            if(!currentActiveMap) currentActiveMap = prompt("Podaj nazwe pod jaka zapisac te mape:");
            if(!currentActiveMap) return;
            let btn = document.querySelector('.btn-save');
            let originalText = btn.innerText;
            btn.innerText = "TRWA ZAPISYWANIE...";
            btn.disabled = true;
            fetch('/api/save', { method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({map_name: currentActiveMap}) })
            .then(r => r.json()).then(d => {
                btn.innerText = originalText; btn.disabled = false;
                if (d.success) alert("Zapisano mape!"); else alert("BLAD ZAPISU:\\n" + d.message);
                loadAvailableMaps(); 
            });
        }

        // Status Checker
        setInterval(() => {
            fetch('/api/status').then(r => r.json()).then(status => {
                document.getElementById('dot-lidar').className = status.lidar ? 'dot active' : 'dot';
                document.getElementById('dot-slam').className = status.slam ? 'dot active' : 'dot';
            });
        }, 1500); 
        
        window.onload = loadAvailableMaps;
    </script>
</body>
</html>
"""
@app.route('/')
def index(): return HTML

if __name__ == '__main__':
    subprocess.run("fuser -k 5000/tcp 2>/dev/null", shell=True)
    time.sleep(1) 
    threading.Thread(target=launch_ros_controller, daemon=True).start()
    threading.Thread(target=motor_worker, daemon=True).start()
    threading.Thread(target=status_checker, daemon=True).start()
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
    app.run(host='0.0.0.0', port=5000, threaded=True, use_reloader=False)