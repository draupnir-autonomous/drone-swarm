#!/bin/bash
# REGISTRAZIONE DELLA FINESTRA DI GAZEBO durante un volo dimostrativo.
#
# Lancia run_multi_exploration.sh con la GUI 3D di Gazebo, aggancia la camera
# della GUI al drone (servizio /gui/follow) e registra la finestra con GStreamer
# (ximagesrc, H.264). A volo concluso ferma registrazione e simulazione.
# Uscita: <cartella>/gazebo_grezzo.mp4 (tempo reale, 60 fps) e il log della run.
#
# Uso: registra_demo_gazebo.sh <mondo> <cartella_uscita>   (es. cubicle_office)
# Variabili: VEL_MAX, VISITA, FASE_FOTO, IMBARDATA_FOTO come run_multi_exploration.sh;
#            SEGUI_OFFSET (default "-3.5, -3.5, 6.0": di lato e dall'alto, attraverso il
#            soffitto, che in cubicle_office e' trasparente al 75%).
set -u
MONDO="${1:-cubicle_office}"
OUT="${2:-/tmp/demo_gazebo}"
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SEGUI_OFFSET="${SEGUI_OFFSET:--3.5, -3.5, 6.0}"
# Camera che insegue il drone (camera_inseguimento.py): offset nel mondo (m) e
# riquadro dell'edificio per lo sfondo nero (vuoto = niente pannelli neri).
CAMERA_OFFSET="${CAMERA_OFFSET:--7,-7,10}"
case "$MONDO" in
  cubicle_office) RIQUADRO="${RIQUADRO:--15.36,16.04,-15.89,15.51}" ;;
  *) RIQUADRO="${RIQUADRO:-}" ;;
esac
export DISPLAY="${DISPLAY:-:10.0}"
mkdir -p "$OUT"
LOG="$OUT/run_gazebo.log"
VIDEO="$OUT/gazebo_grezzo.mp4"
rm -f "$VIDEO"

# NON si carica l'ambiente ROS qui: il suo 'gz' ridotto (gz_tools_vendor, senza
# 'gz sim') finirebbe davanti nel PATH e PX4 non riuscirebbe ad avviare Gazebo.
# Per i servizi della GUI si usa il gz di sistema.
GZ=/usr/bin/gz

# GUI dedicata (sola vista 3D a tutto schermo): si mette al posto di quella
# dell'utente e si rimette la sua all'uscita, qualunque cosa succeda.
GUI_CFG="$HOME/.gz/sim/8/gui.config"
# Copia rimasta da un tentativo interrotto: e' quella vera, si rimette prima.
if [ -f "$GUI_CFG.prima_della_demo" ]; then mv -f "$GUI_CFG.prima_della_demo" "$GUI_CFG"; fi
if [ -f "$GUI_CFG" ]; then cp -p "$GUI_CFG" "$GUI_CFG.prima_della_demo"; fi
ripristina_gui() {
  if [ -f "$GUI_CFG.prima_della_demo" ]; then mv -f "$GUI_CFG.prima_della_demo" "$GUI_CFG"; fi
}
trap ripristina_gui EXIT
trap "exit 130" INT TERM
mkdir -p "$(dirname "$GUI_CFG")"
cp "$WS/scripts/gui_demo.config" "$GUI_CFG"

echo "[demo] avvio del volo con GUI ($MONDO)"
cd "$WS"
( printf 'y\n' | MOSTRA_MAPPA=0 ODOM_SOURCE=lio exec setsid bash scripts/run_multi_exploration.sh "$MONDO" 1 ) \
  > "$LOG" 2>&1 &

# Finestra di Gazebo
WID=""
for i in $(seq 1 180); do
  WID=$(xwininfo -root -tree 2>/dev/null | grep -i '"Gazebo Sim"' | head -1 | awk '{print $1}')
  [ -n "$WID" ] && break
  sleep 1
done
if [ -z "$WID" ]; then
  echo "[demo] finestra di Gazebo non trovata"; exit 1
fi
echo "[demo] finestra di Gazebo: $WID"

# Camera della GUI agganciata al drone (quando il modello esiste)
for i in $(seq 1 120); do
  if $GZ service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 2000 \
       --req 'data: "x500_mid360_0"' 2>/dev/null | grep -q "data: true"; then
    sleep 2
    for k in 1 2 3; do
      $GZ service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean --timeout 3000 \
        --req "x: ${SEGUI_OFFSET%%,*}, y: $(echo "$SEGUI_OFFSET" | cut -d, -f2), z: ${SEGUI_OFFSET##*,}" 2>&1 | tr '\n' ' '
      echo
      sleep 1
    done
    echo "[demo] camera agganciata al drone"
    break
  fi
  sleep 2
done

# /gui/follow risponde "true" ma la vista resta ferma: la camera la muove
# camera_inseguimento.py, che esce da solo a simulazione chiusa.
RIQUADRO_ARG=()
[ -n "$RIQUADRO" ] && RIQUADRO_ARG=("--riquadro=$RIQUADRO")
/usr/bin/python3 "$WS/scripts/camera_inseguimento.py" "$MONDO" x500_mid360_0 "--offset=$CAMERA_OFFSET" \
  "${RIQUADRO_ARG[@]}" > "$OUT/inseguimento.log" 2>&1 &
INSEGUI=$!

echo "[demo] registrazione in $VIDEO"
gst-launch-1.0 -e -q ximagesrc xid="$WID" use-damage=false show-pointer=false \
  ! video/x-raw,framerate=60/1 ! videoconvert ! videoscale add-borders=true \
  ! video/x-raw,width=1920,height=1080,pixel-aspect-ratio=1/1 \
  ! x264enc speed-preset=ultrafast bitrate=20000 key-int-max=60 ! video/x-h264,profile=baseline \
  ! h264parse ! mp4mux ! filesink location="$VIDEO" &
REC=$!

# Fine del volo: atterraggio, oppure volo interrotto, oppure fine dello script
while true; do
  if grep -qE "Tutti i droni sono rientrati e atterrati|Rientro incompleto|Premi Ctrl\+C|Preparazione fallita" "$LOG" 2>/dev/null; then
    break
  fi
  if grep -qE "CRASH DETECTED|VOLO INTERROTTO" /tmp/c2x_run_multi/exploration.log 2>/dev/null; then
    sleep 8; break
  fi
  sleep 3
done
sleep 4
echo "[demo] volo concluso: fermo la registrazione"
kill -INT "$REC" 2>/dev/null
wait "$REC" 2>/dev/null

# Prima di fermare la simulazione si lasciano salvare mappe e mappa 3D (lo
# script di run stampa "Premi Ctrl+C" quando ha finito), al massimo 5 minuti.
for i in $(seq 1 100); do
  grep -q "Premi Ctrl+C" "$LOG" 2>/dev/null && break
  sleep 3
done
# Ferma la simulazione: lo script di run (per PID, mai pkill sul proprio comando)
RUN_PID=$(pgrep -f "^bash scripts/run_multi_exploration.sh $MONDO 1" | head -1)
if [ -n "$RUN_PID" ]; then
  kill -INT "$RUN_PID" 2>/dev/null
  sleep 15
  kill -9 "$RUN_PID" 2>/dev/null
fi
kill "$INSEGUI" 2>/dev/null
echo "[demo] fatto: $VIDEO"
