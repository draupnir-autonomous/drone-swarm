#!/bin/bash
# Avvia tutto il necessario per il collaudo e apre collaudo.py (C2-Explorer, ottobre 2026).
#
#   ./avvia_collaudo.sh reale [/dev/ttyACM0]   drone vero: agente DDS sull'USB del Pixhawk,
#                                              driver Livox, FAST-LIO, ponte della stima
#   ./avvia_collaudo.sh sim [mondo]            simulazione (Gazebo + PX4 + FAST-LIO, senza
#                                              pianificatore), di serie cubicle_office
#
# Variabili utili: GUI=1 (sim: finestra di Gazebo), BAUD (reale, di serie 921600),
# LIVOX_LAUNCH (reale, di serie msg_MID360s_launch.py: il lidar del drone e' un Mid-360S),
# CAMERA_DEV (reale, di serie /dev/video0; CAMERA=0 per non avviarla), e gli argomenti di collaudo.py dopo
# "--", per esempio:  ./avvia_collaudo.sh reale /dev/ttyACM0 -- --joystick /dev/input/js0
# Alla chiusura di collaudo.py (che a drone in volo comanda prima l'atterraggio) si ferma tutto.
# (niente "set -u": gli script di ambiente di ROS usano variabili non definite)
QUI=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WS=$(cd "$QUI/../.." && pwd)
MODO=${1:-}
shift || true
ARG=""
if [ $# -gt 0 ] && [ "$1" != "--" ]; then ARG=$1; shift; fi
[ "${1:-}" = "--" ] && shift
EXTRA=("$@")

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
# Lo stesso profilo DDS delle run (memoria condivisa da 32 MB per le nuvole lidar).
export FASTRTPS_DEFAULT_PROFILES_FILE="$WS/scripts/fastdds_profilo.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
LOG="$QUI/collaudo_registri/processi_$(date +%Y-%m-%d_%H%M%S)"
mkdir -p "$LOG"
PIDS=()      # processi singoli (lo script della simulazione, che fa da se' la sua pulizia)
GRUPPI=()    # drone vero: ogni componente in una sessione sua (setsid), fermato per gruppo

# SIGTERM e non SIGINT: i processi lanciati in background da uno script ignorano SIGINT.
# Lo script della simulazione con SIGTERM fa la sua pulizia (PX4, Gazebo, nodi).
# Drone vero: il segnale va a tutto il gruppo, non solo a "ros2 launch". Fino al 19 ottobre
# 2026 si fermava solo ros2 launch, e driver Livox e FAST-LIO restavano vivi: dopo tre prove
# ce n'erano tre di ciascuno, tutti sullo stesso topic della stima.
ferma() {
  for p in "${PIDS[@]}"; do kill -TERM "$p" 2>/dev/null; done
  for p in "${GRUPPI[@]}"; do kill -TERM -- "-$p" 2>/dev/null; done
  for i in $(seq 1 15); do
    vivi=0
    for p in "${PIDS[@]}"; do kill -0 "$p" 2>/dev/null && vivi=1; done
    for p in "${GRUPPI[@]}"; do kill -0 -- "-$p" 2>/dev/null && vivi=1; done
    [ $vivi = 0 ] && return; sleep 1
  done
  for p in "${PIDS[@]}"; do kill -KILL "$p" 2>/dev/null; done
  for p in "${GRUPPI[@]}"; do kill -KILL -- "-$p" 2>/dev/null; done
}
trap ferma EXIT

case "$MODO" in
  sim)
    MONDO=${ARG:-cubicle_office}   # ricco di struttura: in empty_room FAST-LIO non vedeva gli spostamenti
    echo "Simulazione per il collaudo: $MONDO, FAST-LIO, nessun pianificatore."
    cd "$WS" || exit 1
    # env fa exec dello script: il PID registrato e' quello dello script stesso.
    # Stesso ambiente delle prove: lo script carica ROS da se', senza /opt/ros nel PATH.
    PATH_RUN=$(echo "$PATH" | tr ':' '\n' | grep -v '^/opt/ros' | paste -sd:)
    env PATH="$PATH_RUN" SOLO_STIMA=1 ODOM_SOURCE=lio MAPPA_REALISTICA=0 FOTO=0 PF_MIN_RAM_GB=3 MOSTRA_MAPPA=0 \
      ./scripts/run_multi_exploration.sh "$MONDO" 1 \
      <<< "$([ "${GUI:-0}" = 1 ] && echo y || echo N)" > "$LOG/simulazione.log" 2>&1 &
    PIDS+=($!)
    cd - > /dev/null
    echo "Attendo la simulazione (registro: $LOG/simulazione.log)..."
    for i in $(seq 1 240); do
      grep -q "Pronto per il collaudo" "$LOG/simulazione.log" && break
      grep -q "fallita\|ERRORE" "$LOG/simulazione.log" && { tail -5 "$LOG/simulazione.log"; exit 1; }
      sleep 1
    done
    grep -q "Pronto per il collaudo" "$LOG/simulazione.log" || { echo "simulazione non pronta"; exit 1; }
    python3 "$QUI/collaudo.py" --sim "${EXTRA[@]}"
    ;;
  reale)
    PORTA=${ARG:-/dev/ttyACM0}
    [ -e "$PORTA" ] || { echo "Nessun $PORTA: Pixhawk non collegato via USB? (ls /dev/ttyACM*)"; exit 1; }
    [ -r "$PORTA" ] && [ -w "$PORTA" ] || { echo "Permessi su $PORTA: sudo usermod -aG dialout $USER, poi rientra"; exit 1; }
    command -v MicroXRCEAgent >/dev/null || { echo "MicroXRCEAgent non trovato (Micro-XRCE-DDS-Agent)"; exit 1; }
    # ModemManager interroga le porte /dev/ttyACM* come se fossero modem e disturba il Pixhawk.
    if systemctl is-active --quiet ModemManager 2>/dev/null; then
      echo "ATTENZIONE: ModemManager e' attivo e puo' disturbare il Pixhawk su $PORTA."
      echo "Spegnilo una volta per tutte con:  sudo systemctl disable --now ModemManager"
      read -r -p "Continuo lo stesso? [s/N] " r; [ "$r" = "s" ] || exit 1
    fi
    ros2 pkg prefix livox_ros_driver2 >/dev/null 2>&1 && \
      [ -n "$(ros2 pkg executables livox_ros_driver2 2>/dev/null)" ] || {
      echo "Driver Livox (livox_ros_driver2 con Livox-SDK2) non installato: vedi LEGGIMI_COLLAUDO.md"; exit 1; }
    # Avanzi di una prova precedente (agente, driver, FAST-LIO, ponte): con due copie la
    # stima arriva a PX4 da due FAST-LIO diversi e l'agente vecchio ruba i dati della porta.
    VECCHI=$( { pgrep -x MicroXRCEAgent; pgrep -x livox_ros_drive; pgrep -x fastlio_mapping;
               pgrep -x lio_to_px4_brid; } | tr '\n' ' ')
    if [ -n "${VECCHI// /}" ]; then
      echo "Sono ancora in esecuzione programmi di una prova precedente:"
      ps -o pid,etime,args -p "$(echo $VECCHI | tr " " ",")" | cut -c1-110
      echo "Se NON c'e' un'altra prova aperta (o un drone in volo con un'altra missione), li fermo."
      read -r -p "Li fermo? [s/N] " r; [ "$r" = "s" ] || exit 1
      kill -TERM $VECCHI 2>/dev/null; sleep 3; kill -KILL $VECCHI 2>/dev/null
    fi
    # Niente agente sulla porta finche' c'e' il bootloader: se riceve dati resta fermo
    # (19 ottobre 2026, bloccato per minuti dopo un riavvio). px4_usb_dds.py lo fa partire.
    python3 "$QUI/px4_usb_dds.py" firmware "$PORTA" || {
      echo "Su $PORTA non parte PX4 (resta il bootloader?): stacca USB e batteria, riattacca."; exit 1; }
    # Con SYS_USB_AUTO diverso da 0 PX4 mette MAVLink sull'USB e il DDS non parte mai
    # (prima prova del 19 ottobre 2026: "FASE 0 NON SUPERATA - nessun vehicle_status").
    if python3 "$QUI/px4_usb_dds.py" controlla "$PORTA" 2>/dev/null; then
      echo "Il Pixhawk parla MAVLink su $PORTA: SYS_USB_AUTO non e' 0 e il DDS resta fermo."
      echo "Posso salvare tutti i parametri, impostare SYS_USB_AUTO = 0 e riavviare il Pixhawk."
      echo "Dopo, QGroundControl non si collega piu' via USB (per tornare: PROCEDURA_PROVE_REALI.md, A.3)."
      read -r -p "Procedo? [s/N] " r; [ "$r" = "s" ] || exit 1
      python3 "$QUI/px4_usb_dds.py" imposta "$PORTA" \
        "$QUI/collaudo_registri/parametri_px4_$(date +%Y-%m-%d_%H%M%S).params" || exit 1
      echo "Attendo che il Pixhawk si riavvii..."
      sleep 3
      python3 "$QUI/px4_usb_dds.py" firmware "$PORTA" || { echo "PX4 non e' ripartito su $PORTA"; exit 1; }
      sleep 3
    fi
    echo "Agente DDS su $PORTA a ${BAUD:-921600} baud..."
    setsid MicroXRCEAgent serial --dev "$PORTA" -b "${BAUD:-921600}" > "$LOG/agente_dds.log" 2>&1 < /dev/null &
    GRUPPI+=($!)
    echo "Driver Livox (${LIVOX_LAUNCH:-msg_MID360s_launch.py})..."
    setsid ros2 launch livox_ros_driver2 "${LIVOX_LAUNCH:-msg_MID360s_launch.py}" > "$LOG/livox.log" 2>&1 < /dev/null &
    GRUPPI+=($!)
    sleep 4
    if grep -q "bind failed\|Init lds lidar fail" "$LOG/livox.log"; then
      CFG="$WS/src/terze_parti/livox_ros_driver2/config/MID360s_config.json"
      echo "Il driver Livox non si collega al lidar (\"bind failed\"): il Jetson non ha sulla porta"
      echo "Ethernet l'indirizzo indicato in host_net_info di $CFG"
      echo "(ip -br addr per vedere gli indirizzi; vedi LEGGIMI_COLLAUDO.md, rete del lidar)."
      exit 1
    fi
    # PX4 crea i suoi topic dopo ~5 s di sincronizzazione dell'ora con l'agente (500 scambi).
    # Il 21 ottobre 2026, col Pixhawk acceso da ore (la batteria lo teneva acceso anche col
    # cavo USB staccato e col Jetson riavviato), restava fermo li': sessione aperta, nessun
    # topic, fase 0 "nessun vehicle_status". Risolto solo spegnendolo del tutto.
    echo "Attendo i topic di PX4 (sincronizzazione dell'ora, ~5 s)..."
    for i in $(seq 1 20); do grep -q "topic created" "$LOG/agente_dds.log" && break; sleep 1; done
    if ! grep -q "topic created" "$LOG/agente_dds.log"; then
      if grep -q "participant created" "$LOG/agente_dds.log"; then
        echo "PX4 apre la sessione DDS ma resta fermo alla sincronizzazione dell'ora e non crea i topic."
        echo "Spegni del tutto il Pixhawk (stacca batteria E cavo USB), riaccendilo e rilancia."
        echo "Rimedio definitivo: UXRCE_DDS_SYNCT = 0 (scheda_sd_una_volta/etc/config.txt, PROCEDURA A.3)."
      else
        echo "PX4 non apre la sessione DDS su $PORTA (SYS_USB_AUTO = 0? /etc/extras.txt sulla scheda SD?)."
      fi
      exit 1
    fi
    echo "FAST-LIO e ponte della stima verso PX4$([ -e "${CAMERA_DEV:-/dev/video0}" ] && [ "${CAMERA:-1}" = 1 ] && echo ", telecamera ${CAMERA_DEV:-/dev/video0}")..."
    export C2X_LIO_DIR="$LOG/fast_lio"
    setsid ros2 launch "$QUI/collaudo_reale.launch.py" > "$LOG/stima.log" 2>&1 < /dev/null &
    GRUPPI+=($!)
    sleep 5
    echo "Registri dei processi in $LOG"
    python3 "$QUI/collaudo.py" "${EXTRA[@]}"
    ;;
  *)
    sed -n 2,13p "$0"
    exit 1
    ;;
esac
