#!/bin/bash
# Orchestrates the full C2-Explorer single-drone simulation stack:
# PX4 SITL + Gazebo -> Micro XRCE-DDS Agent -> lidar bridge -> the 3
# c2x_px4_bridge nodes -> the exploration stack, triggers exploration,
# waits for it to finish (or crash), saves the explored map, and opens
# it as an image.
#
# Usage: ./run_exploration.sh [scenario]
#   scenario: empty_room (default) | cubicle_office

SCENARIO="${1:-empty_room}"

ROS2_SETUP=/opt/ros/jazzy/setup.bash
# MicroXRCEAgent e' installato in ~/.local/bin (compilato senza sudo).
export PATH="$HOME/.local/bin:$PATH"
# Percorsi relativi alla posizione di questo script (vedi run_multi_exploration.sh).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Trasporto DDS a memoria condivisa grande, come in run_multi_exploration.sh:
# vedi il commento in fastdds_profilo.xml.
export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/fastdds_profilo.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/fastdds_profilo.xml"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PROJECT_DIR="$(cd "$WS_DIR/.." && pwd)"
WS_SETUP=$WS_DIR/install/setup.bash
PX4_DIR=$PROJECT_DIR/PX4-Autopilot
LOG_DIR=/tmp/c2x_run
PCD_PATH=/tmp/c2x_explored_map.pcd
MAP_PNG=/tmp/c2x_run/explored_map.png
FINISH_TIMEOUT_S=600   # give up waiting for "finished exploration" after this long
WARMUP_S=25            # time to let the stationary drone accumulate an initial
                        # scan before triggering - without this, the very first
                        # frontier check can find nothing yet (drone starts on
                        # the ground, not already hovering) and the FSM
                        # concludes "nothing to explore" and stops immediately
STALE_WARN_S=25        # warn if no new log activity for this long
MAX_TRIGGER_ATTEMPTS=5 # empty_room's frontier detection can flicker between finding a
                        # few and finding none within milliseconds near spawn - a trigger
                        # landing on an empty tick sends the FSM straight to FINISH (a
                        # terminal state; only relaunching exploration_node resets it, just
                        # resending the trigger message does nothing). Retry the whole
                        # launch+warmup+trigger cycle instead of accepting the first miss.

case "$SCENARIO" in
  empty_room)
    PX4_GZ_WORLD=empty_room
    SPAWN_POSE=""              # default spawn baked into the world works fine here
    CONFIG_NAME=single_drone_empty_room.yaml
    ;;
  cubicle_office)
    PX4_GZ_WORLD=cubicle_office
    SPAWN_POSE="-13.5,0,0.1,0,0,0"   # matches the paper's own start position
    CONFIG_NAME=single_drone_cubicle_office.yaml
    ;;
  *)
    echo "Scenario sconosciuto: $SCENARIO (valori validi: empty_room, cubicle_office)"
    exit 1
    ;;
esac

mkdir -p "$LOG_DIR"

# All pkill/rm on /dev/shm below only ever run either before anything is
# launched, or in the EXIT trap after everything has been asked to die -
# never while the stack is live. Cleaning DDS shared-memory segments out
# from under still-running nodes corrupts their discovery state (this bit
# us mid-session: nodes stayed "alive" but stopped receiving messages).

PIDS=()
cleanup() {
  echo ""
  echo "[cleanup] Arresto di tutti i processi..."
  for pid in "${PIDS[@]}"; do
    kill -9 "$pid" 2>/dev/null
  done
  pkill -9 -f "MicroXRCEAgent" 2>/dev/null
  pkill -9 -f "bin/px4" 2>/dev/null
  pkill -9 -f "gz sim" 2>/dev/null
  pkill -9 -f "ros_gz_bridge" 2>/dev/null
  pkill -9 -f "c2x_px4_bridge" 2>/dev/null
  pkill -9 -f "exploration_node" 2>/dev/null
  pkill -9 -f "mtsp_node" 2>/dev/null
  pkill -9 -f "plan_manage/traj_server" 2>/dev/null
}
trap cleanup EXIT INT TERM

# One-line heartbeat: last meaningful FSM/status line, plus how long ago the
# log last changed at all (a rough "is it actually doing something" signal).
print_progress() {
  local last_line log_mtime now stale_s
  last_line=$(grep -E "\[FSM\]:|Triggered|finished exploration|No coverable frontier|CRASH DETECTED" \
    "$LOG_DIR/exploration.log" 2>/dev/null | tail -1)
  log_mtime=$(stat -c %Y "$LOG_DIR/exploration.log" 2>/dev/null || echo 0)
  now=$(date +%s)
  stale_s=$((now - log_mtime))
  if [ -z "$last_line" ]; then
    last_line="(nessuna attivita registrata ancora)"
  fi
  echo "[$(date +%H:%M:%S)] ${last_line}"
  if [ "$stale_s" -ge "$STALE_WARN_S" ]; then
    echo "    ATTENZIONE: nessuna nuova riga di log da ${stale_s}s - potrebbe essersi bloccato"
    echo "    (o e' semplicemente impegnato a ricalcolare la mappa su una scena complessa)."
  fi
}

echo "=== C2-Explorer: avvio simulazione (scenario: $SCENARIO) ==="
read -r -p "Vuoi vedere la GUI 3D di Gazebo? (y/N - no e' piu' leggero per la CPU e per la reattivita' del planner): " SHOW_GUI
SHOW_GUI=${SHOW_GUI:-n}

# --- Pre-flight cleanup: kill any leftover instance and stale DDS shm segments ---
echo "[0/6] Pulizia processi residui..."
pkill -9 -f "MicroXRCEAgent" 2>/dev/null
pkill -9 -f "bin/px4" 2>/dev/null
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "ros_gz_bridge" 2>/dev/null
pkill -9 -f "c2x_px4_bridge" 2>/dev/null
pkill -9 -f "exploration_node" 2>/dev/null
pkill -9 -f "mtsp_node" 2>/dev/null
pkill -9 -f "plan_manage/traj_server" 2>/dev/null
sleep 2
bash -c "source $ROS2_SETUP; ros2 daemon stop" >/dev/null 2>&1
rm -rf /dev/shm/fastrtps_* 2>/dev/null
bash -c "source $ROS2_SETUP; ros2 daemon start" >/dev/null 2>&1
sleep 1

# --- 1. PX4 SITL + Gazebo ---
echo "[1/6] Avvio PX4 SITL + Gazebo (mondo: $PX4_GZ_WORLD)..."
: > "$LOG_DIR/px4.log"
(
  # Niente source di ROS2: su Jazzy mette in testa al PATH il 'gz' ridotto di
  # gz_tools_vendor (senza 'gz sim') e PX4 non troverebbe Gazebo.
  cd "$PX4_DIR" || exit 1
  export PX4_GZ_WORLD
  [ -n "$SPAWN_POSE" ] && export PX4_GZ_MODEL_POSE="$SPAWN_POSE"
  if [[ "$SHOW_GUI" =~ ^[Yy] ]]; then
    make px4_sitl gz_x500_mid360 > "$LOG_DIR/px4.log" 2>&1
  else
    HEADLESS=1 make px4_sitl gz_x500_mid360 > "$LOG_DIR/px4.log" 2>&1
  fi
) &
PIDS+=($!)
for i in $(seq 1 40); do
  grep -q "pxh>" "$LOG_DIR/px4.log" 2>/dev/null && break
  sleep 1
done
if ! grep -q "pxh>" "$LOG_DIR/px4.log" 2>/dev/null; then
  echo "ERRORE: PX4/Gazebo non e' partito entro 40s. Controlla $LOG_DIR/px4.log"
  exit 1
fi
echo "    OK - PX4/Gazebo pronti."

# --- 2. Micro XRCE-DDS Agent ---
echo "[2/6] Avvio Micro XRCE-DDS Agent..."
(MicroXRCEAgent udp4 -p 8888 \
  > "$LOG_DIR/xrce.log" 2>&1) &
PIDS+=($!)
for i in $(seq 1 15); do
  grep -qi "session established" "$LOG_DIR/xrce.log" 2>/dev/null && break
  sleep 1
done
echo "    OK - agente connesso a PX4."

# --- 3. Lidar point cloud bridge ---
echo "[3/6] Avvio bridge point cloud LiDAR..."
(
  source "$ROS2_SETUP"
  ros2 run ros_gz_bridge parameter_bridge \
    "/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked" \
    > "$LOG_DIR/gzbridge.log" 2>&1
) &
PIDS+=($!)
sleep 3
echo "    OK."

# --- 4. The three custom PX4<->ROS2 bridge nodes ---
echo "[4/6] Avvio bridge odometria/point-cloud/comandi..."
(
  source "$ROS2_SETUP"; source "$WS_SETUP"
  ros2 run c2x_px4_bridge odometry_bridge > "$LOG_DIR/odom_bridge.log" 2>&1
) &
PIDS+=($!)
(
  source "$ROS2_SETUP"; source "$WS_SETUP"
  ros2 run c2x_px4_bridge cloud_transform_bridge > "$LOG_DIR/cloud_bridge.log" 2>&1
) &
PIDS+=($!)
(
  source "$ROS2_SETUP"; source "$WS_SETUP"
  ros2 run c2x_px4_bridge position_command_bridge > "$LOG_DIR/poscmd_bridge.log" 2>&1
) &
PIDS+=($!)
sleep 5
echo "    OK."

# --- 5. Exploration stack ---
echo "[5/6] Avvio stack di esplorazione (config: $CONFIG_NAME)..."

EXPL_PID=""
launch_exploration_node() {
  : > "$LOG_DIR/exploration.log"
  (
    source "$ROS2_SETUP"; source "$WS_SETUP"
    ros2 launch exploration_manager single_drone_exploration.launch.py \
      config_name:="$CONFIG_NAME" > "$LOG_DIR/exploration.log" 2>&1
  ) &
  EXPL_PID=$!
  PIDS+=($EXPL_PID)
  for i in $(seq 1 30); do
    grep -q "wait for trigger" "$LOG_DIR/exploration.log" 2>/dev/null && break
    sleep 1
  done
  if ! grep -q "wait for trigger" "$LOG_DIR/exploration.log" 2>/dev/null; then
    echo "ERRORE: lo stack di esplorazione non e' pronto entro 30s. Controlla $LOG_DIR/exploration.log"
    exit 1
  fi
}

# Kill only the exploration_manager launch tree (exploration_node, mtsp_node,
# traj_server) so it can be relaunched to reset the FSM out of FINISH, without
# tearing down PX4/Gazebo/the bridges, which stay up across retries.
stop_exploration_node() {
  [ -n "$EXPL_PID" ] && kill -9 "$EXPL_PID" 2>/dev/null
  pkill -9 -f "exploration_node" 2>/dev/null
  pkill -9 -f "mtsp_node" 2>/dev/null
  pkill -9 -f "plan_manage/traj_server" 2>/dev/null
  sleep 2
}

launch_exploration_node
echo "    OK - pronto al trigger."

# --- 6a. Warm-up: let the stationary drone accumulate an initial scan ---
echo "[6/6] Riscaldamento (accumulo scansione iniziale, ${WARMUP_S}s)..."
w=0
while [ $w -lt $WARMUP_S ]; do
  echo "    ...${w}/${WARMUP_S}s"
  sleep 5
  w=$((w + 5))
done

send_trigger() {
  (
    source "$ROS2_SETUP"
    # The goal pose in this message is ignored by the FSM (it triggers off the
    # drone's own current position, not this target) - any valid message works.
    timeout 6 ros2 topic pub /move_base_simple/goal geometry_msgs/msg/PoseStamped \
      "{header: {frame_id: 'world'}, pose: {position: {x: 0.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}" \
      -r 5 >> "$LOG_DIR/trigger.log" 2>&1
  ) &
  wait $!
}

TRIGGER_OK=0
for attempt in $(seq 1 $MAX_TRIGGER_ATTEMPTS); do
  echo "Invio trigger di esplorazione (tentativo $attempt/$MAX_TRIGGER_ATTEMPTS)..."
  send_trigger
  sleep 3

  if grep -q "WAIT_TRIGGER to PLAN_TRAJ" "$LOG_DIR/exploration.log" 2>/dev/null; then
    echo "    OK - esplorazione avviata, il drone sta decollando."
    TRIGGER_OK=1
    break
  fi

  if grep -q "WAIT_TRIGGER to FINISH" "$LOG_DIR/exploration.log" 2>/dev/null; then
    if [ "$attempt" -ge "$MAX_TRIGGER_ATTEMPTS" ]; then
      echo "    ATTENZIONE: l'esplorazione si e' fermata subito (nessuna frontiera trovata"
      echo "    dalla scansione iniziale) dopo $MAX_TRIGGER_ATTEMPTS tentativi. La mappa che"
      echo "    otterrai sara' minima."
      break
    fi
    echo "    Nessuna frontiera trovata alla prima scansione (l'FSM e' in FINISH, uno stato"
    echo "    terminale che non accetta piu' trigger) - riavvio il nodo di esplorazione e"
    echo "    riprovo..."
    stop_exploration_node
    launch_exploration_node
    continue
  fi

  echo "    Nessuna conferma nel log, riprovo l'invio del trigger..."
done

if [ "$TRIGGER_OK" -ne 1 ] \
   && ! grep -q "WAIT_TRIGGER to FINISH" "$LOG_DIR/exploration.log" 2>/dev/null; then
  echo "    ATTENZIONE: nessuna conferma di avvio esplorazione dopo $MAX_TRIGGER_ATTEMPTS tentativi."
fi

# --- 6b. Wait for exploration to finish (or crash), printing progress along the way ---
echo ""
echo "=== Esplorazione in corso... aspetto il completamento (timeout ${FINISH_TIMEOUT_S}s) ==="
ELAPSED=0
while [ $ELAPSED -lt $FINISH_TIMEOUT_S ]; do
  if grep -q "has finished exploration" "$LOG_DIR/exploration.log" 2>/dev/null; then
    echo "Esplorazione completata!"
    break
  fi
  if grep -q "CRASH DETECTED" "$LOG_DIR/exploration.log" 2>/dev/null; then
    echo "Il drone si e' capovolto (crash fisico) - esplorazione fermata dalla rete di sicurezza."
    grep "CRASH DETECTED" "$LOG_DIR/exploration.log" | tail -1
    break
  fi
  print_progress
  sleep 10
  ELAPSED=$((ELAPSED + 10))
done
if [ $ELAPSED -ge $FINISH_TIMEOUT_S ]; then
  echo "Timeout raggiunto, procedo comunque a salvare la mappa allo stato attuale."
fi

# --- Save the map ---
echo ""
echo "=== Salvataggio mappa ==="
bash -c "source $ROS2_SETUP; source $WS_SETUP; ros2 service call /sdf_map/save_map std_srvs/srv/Trigger"

if [ ! -f "$PCD_PATH" ]; then
  echo "ERRORE: mappa non trovata in $PCD_PATH"
  exit 1
fi

# --- Render and open it ---
# PATH=/usr/bin first: this machine's miniconda shadows the system python3
# that rclpy/colcon/matplotlib are actually installed against.
echo "=== Generazione immagine mappa ==="
PATH=/usr/bin:$PATH /usr/bin/python3 - "$PCD_PATH" "$MAP_PNG" <<'PYEOF'
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

pcd_path, out_path = sys.argv[1], sys.argv[2]

with open(pcd_path, "rb") as f:
    header = {}
    while True:
        line = f.readline().decode("ascii").strip()
        if line.startswith("DATA"):
            break
        parts = line.split()
        if parts:
            header[parts[0]] = parts[1:]
    n_points = int(header["POINTS"][0])
    raw = f.read(n_points * 3 * 4)
    pts = np.frombuffer(raw, dtype=np.float32).reshape(n_points, 3)

x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]

fig, axes = plt.subplots(1, 2, figsize=(14, 6))
sc0 = axes[0].scatter(x, y, c=z, s=2, cmap="viridis")
axes[0].set_title(f"Vista dall'alto - {n_points} punti")
axes[0].set_xlabel("x (m)"); axes[0].set_ylabel("y (m)")
axes[0].set_aspect("equal")
fig.colorbar(sc0, ax=axes[0], label="z (m)")

sc1 = axes[1].scatter(x, z, c=y, s=2, cmap="plasma")
axes[1].set_title("Vista laterale (x-z)")
axes[1].set_xlabel("x (m)"); axes[1].set_ylabel("z (m)")
axes[1].set_aspect("equal")
fig.colorbar(sc1, ax=axes[1], label="y (m)")

plt.tight_layout()
plt.savefig(out_path, dpi=150)
print(f"Salvata in {out_path} ({n_points} punti)")
PYEOF

echo "=== Apertura immagine ==="
# xdg-open, not eog: eog is not installed on this machine and the script
# ended on "eog: command not found" after generating the image just fine.
xdg-open "$MAP_PNG" >/dev/null 2>&1 &
disown

echo ""
echo "Fatto. Mappa PCD: $PCD_PATH | Immagine: $MAP_PNG"
echo "Premi Ctrl+C in questo terminale per chiudere tutta la simulazione."
wait
