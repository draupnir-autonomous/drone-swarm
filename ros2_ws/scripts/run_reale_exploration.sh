#!/bin/bash
# Missione di esplorazione C2-Explorer sul DRONE VERO (ottobre 2026): lo stesso stack di
# run_multi_exploration.sh senza simulazione. Un drone solo: Pixhawk collegato al Jetson via
# USB (uXRCE-DDS), Livox Mid-360 col suo driver, FAST-LIO, pianificatore.
#
#   BOX="xmin,ymin,xmax,ymax" ./scripts/run_reale_exploration.sh [/dev/ttyACM0]
#   BOX="xmin,ymin,xmax,ymax" ./scripts/run_reale_exploration.sh sim [mondo]
#
# Il secondo comando e' la PROVA GENERALE in simulazione (Gazebo + PX4 + FAST-LIO al posto
# di Pixhawk, lidar e driver; di serie cubicle_office): stessa sequenza, stessi controlli,
# stesso sorvegliante. Va fatta prima di ogni cambio dello stack, a casa.
#
# PRIMA: collaudo superato sul drone vero (scripts/collaudo, fasi 0-7: senza, i controlli a
# terra rifiutano il volo) e LEGGIMI_COLLAUDO.md letto. Drone appoggiato NEL PUNTO DI
# DECOLLO, col muso nella direzione che diventera' l'asse x, eliche montate, nessuno entro 3 m.
#
# BOX (obbligatorio): il recinto della missione, in metri, nelle coordinate di FAST-LIO:
# origine nel punto di decollo, x in AVANTI (muso), y a SINISTRA. Esempio: BOX="-1.5,-2,4,2"
# = da 1.5 m dietro a 4 m davanti, 2 m per lato. Il decollo deve stare a 1 m dai bordi; il
# recinto deve stare dentro lo spazio libero con margine (il drone puo' sforare di 0.5 m
# prima che il sorvegliante lo faccia atterrare).
#
# Sequenza:
#   [0] verifiche (porta, agente DDS, driver Livox, spazio di lavoro, nessun residuo di
#       sessioni precedenti) e configurazione del recinto (scripts/collaudo/prepara_reale.py)
#   [1] agente DDS sulla seriale del Pixhawk            [2] driver del Mid-360
#   [3] stima: FAST-LIO e ponte verso PX4 (launch reale:=true parte:=stima)
#   [4] controlli a terra (scripts/collaudo/sorveglia_missione.py controlli)
#   [5] consenso: si scrive VOLA
#   [6] missione (parte:=missione): il ponte dei comandi ARMA E DECOLLA SUBITO a 1 m, fa un
#       giro di 360 gradi, poi parte l'esplorazione; il sorvegliante resta in primo piano:
#           SPAZIO = EMERGENZA (motori spenti, il drone CADE)     l = atterra
#   [7] a drone atterrato: mappe salvate, tutto spento, run archiviata in risultati_prova/
#
# Variabili (di serie i valori per le PRIME PROVE):
#   VEL_MAX=0.3            velocita' massima del pianificatore [m/s]
#   QUOTA_ZONA=2.2         quota massima della zona da esplorare [m]. Il rientro vola a 1.8 m
#                          e il riferimento non supera 2.0 m: servono almeno 2.8 m di soffitto
#   TEMPO_MAX=240          tetto al tempo di esplorazione [s], poi rientro e atterraggio
#   TEMPO_VOLO_MAX         tetto all'intero volo [s] (di serie TEMPO_MAX + 180): poi il
#                          sorvegliante fa atterrare dov'e'
#   VISITA=0               fase di visita dopo l'esplorazione (1 = si')
#   GIRO_INIZIALE=0.5      giro di 360 gradi a quota di decollo [rad/s]
#   PARTENZA_Z=0.20        quota da terra del punto di riferimento del drone appoggiato [m]
#   LIDAR_POS="0,0,0.10"   posizione del Mid-360 rispetto a quel punto (x avanti, y sinistra,
#                          z su) [m]. Vanno MISURATE entrambe (LEGGIMI_COLLAUDO.md)
#   BAUD=921600            velocita' della seriale del Pixhawk
#   LIVOX_LAUNCH=msg_MID360s_launch.py   launch del driver Livox (il lidar del drone e' un Mid-360S)
#   MAPPA_REALISTICA=1     mappa 3D dal lidar (grigia: la telecamera vera non e' collegata)
#   SENZA_COLLAUDO=1       non pretendere le fasi 0-7 del collaudo (sconsigliato; resta scritto)
#   ARCHIVIA=0             niente archivio in risultati_prova/
# Registri della sessione in /tmp/c2x_run_reale (sovrascritti dalla successiva) e, a fine
# missione, in risultati_prova/<data_ora>_reale_lio_v<VEL_MAX>/. Il registro di volo di PX4
# (.ulg) resta sulla scheda SD del Pixhawk.
# (niente "set -u": gli script di ambiente di ROS usano variabili non definite)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PROJECT_DIR="$(cd "$WS_DIR/.." && pwd)"
COLLAUDO_DIR="$SCRIPT_DIR/collaudo"
ROS2_SETUP=/opt/ros/jazzy/setup.bash
WS_SETUP=$WS_DIR/install/setup.bash
LOG_DIR=/tmp/c2x_run_reale
RISULTATI_DIR="$PROJECT_DIR/risultati_prova"

MODO=reale
if [ "${1:-}" = "sim" ]; then
  MODO=sim
  MONDO_SIM="${2:-cubicle_office}"
else
  PORTA="${1:-/dev/ttyACM0}"
fi

VEL_MAX="${VEL_MAX:-0.3}"
QUOTA_ZONA="${QUOTA_ZONA:-2.2}"
TEMPO_MAX="${TEMPO_MAX:-240}"
TEMPO_VOLO_MAX="${TEMPO_VOLO_MAX:-$((${TEMPO_MAX%.*} + 180))}"
VISITA="${VISITA:-0}"
GIRO_INIZIALE="${GIRO_INIZIALE:-0.5}"
PARTENZA_Z="${PARTENZA_Z:-0.20}"
LIDAR_POS="${LIDAR_POS:-0,0,0.10}"
BAUD="${BAUD:-921600}"
LIVOX_LAUNCH="${LIVOX_LAUNCH:-msg_MID360s_launch.py}"
MAPPA_REALISTICA="${MAPPA_REALISTICA:-1}"
SENZA_COLLAUDO="${SENZA_COLLAUDO:-0}"
ARCHIVIA="${ARCHIVIA:-1}"
# Tetto del riferimento (max_flight_z del launch) e quota oltre la quale il sorvegliante fa
# atterrare: il riferimento puo' chiedere fino a 2.0 m, il drone insegue con qualche cm.
MAX_FLIGHT_Z=2.0
QUOTA_ALLARME=2.3
READY_TIMEOUT_S="${READY_TIMEOUT_S:-120}"
RUN_INIZIO=$(date +%Y-%m-%d_%H%M)
RUN_T0=$(date +%s)

mkdir -p "$LOG_DIR"
# Tutto quello che passa dal terminale finisce anche in run.log. tee non deve morire ne' con
# Ctrl+C ne' con la sessione SSH (-p: se il terminale sparisce continua a scrivere il file):
# morto tee, la prima riga scritta fermerebbe lo script a meta' pulizia.
exec > >(trap '' INT TERM HUP; exec tee -p "$LOG_DIR/run.log") 2>&1

if [ -z "$BOX" ]; then
  sed -n 2,33p "$0"
  echo
  echo "ERRORE: manca BOX=\"xmin,ymin,xmax,ymax\" (il recinto della missione, vedi sopra)."
  exit 1
fi

source "$ROS2_SETUP"
[ -f "$WS_SETUP" ] || { echo "ERRORE: spazio di lavoro non compilato ($WS_SETUP)"; exit 1; }
source "$WS_SETUP"
# Lo stesso profilo DDS delle simulazioni (memoria condivisa da 32 MB per le nuvole lidar).
export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/fastdds_profilo.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
unset C2X_SOLO_STIMA

# ---------------------------------------------------------------- processi
# Ogni componente gira in una sessione sua (setsid): Ctrl+C e la caduta della sessione SSH
# arrivano solo a questo script e al sorvegliante, che decidono l'ordine di spegnimento. La
# stima, in particolare, si spegne SOLO a drone disarmato: senza, PX4 perde la posizione in
# volo. Si fermano per gruppo di processi (il PID e' anche il gruppo), mai per nome.
declare -A PID
avvia() {   # avvia <nome> <registro> <comando...>
  local nome=$1 log=$2
  shift 2
  setsid "$@" > "$log" 2>&1 < /dev/null &
  PID[$nome]=$!
}
vivo() { [ -n "${PID[$1]}" ] && kill -0 "${PID[$1]}" 2>/dev/null; }
ferma() {   # ferma <nome> [attesa in s]
  local nome=$1 attesa=${2:-10} p=${PID[$1]} i
  [ -n "$p" ] || return 0
  kill -TERM -- "-$p" 2>/dev/null
  for i in $(seq 1 "$attesa"); do kill -0 "$p" 2>/dev/null || break; sleep 1; done
  kill -KILL -- "-$p" 2>/dev/null
  PID[$nome]=""
}

SORVEGLIA=("$COLLAUDO_DIR/sorveglia_missione.py")
ARCHIVIATA=0
MISSIONE_AVVIATA=0
REGISTRATORE=""
VIA_PID=""
ESITO="interrotta prima del volo"

archivia_run() {
  [ "$ARCHIVIA" = "1" ] && [ "$ARCHIVIATA" = "0" ] && [ "$MISSIONE_AVVIATA" = "1" ] || return 0
  ARCHIVIATA=1
  local nome="${RUN_INIZIO}_reale_lio_v${VEL_MAX}"
  [ "$MODO" = "sim" ] && nome="${RUN_INIZIO}_reale-sim_${MONDO_NOME:-mondo}_lio_v${VEL_MAX}"
  local dst="$RISULTATI_DIR/$nome"
  [ -e "$dst" ] && dst="${dst}_$(date +%S)"
  mkdir -p "$dst" || { echo "    ATTENZIONE: archivio non creato in $dst"; return 1; }
  cp "$LOG_DIR/exploration.log" "$dst/rel_expl_1.log" 2>/dev/null
  cp "$LOG_DIR/mondo/config.yaml" "$dst/rel_config.yaml" 2>/dev/null
  cp "$LOG_DIR/mondo/mondo.json" "$dst/rel_recinto.json" 2>/dev/null
  if [ "$MODO" = "sim" ]; then
    # Prova in simulazione: mondo simulato e registro di volo di PX4, nel formato delle run
    # simulate, cosi' analisi_campagna.py misura margini veri, quota e inseguimento.
    local sim=/tmp/c2x_run_multi px4b="$PROJECT_DIR/PX4-Autopilot/build/px4_sitl_default" ulg
    cp "$sim/mondo/mondo.json" "$dst/rel_mondo.json" 2>/dev/null
    cp "$sim/mondo/$MONDO_NOME.sdf" "$dst/rel_mondo.sdf" 2>/dev/null
    cp "$sim/px4_0.log" "$dst/rel_px4_1_0.log" 2>/dev/null
    ulg=$(find "$px4b/rootfs/instance_0/log" -name '*.ulg' -newermt "@$RUN_T0" -printf '%T@ %p\n' 2>/dev/null \
          | sort -n | tail -1 | cut -d' ' -f2-)
    [ -n "$ulg" ] && cp "$ulg" "$dst/rel_ulog_1_0.ulg"
  fi
  cp /tmp/c2x_explored_map_drone1.pcd "$dst/rel_map_1.pcd" 2>/dev/null
  local f
  for f in stima livox agente_dds sorveglianza via registra_mappa_1 simulazione; do
    [ -s "$LOG_DIR/$f.log" ] && cp "$LOG_DIR/$f.log" "$dst/rel_$f.log"
  done
  local src="$LOG_DIR/mappa_realistica_drone1"
  if [ -d "$src" ]; then
    mkdir -p "$dst/rel_mappa3d_1_1"
    cp "$src"/anteprima_*.png "$src"/mappa_realistica.ply "$src"/nuvola_colorata.ply \
      "$dst/rel_mappa3d_1_1/" 2>/dev/null
  fi
  {
    echo "inizio:          $(date -d "@$RUN_T0" '+%Y-%m-%d %H:%M:%S')"
    echo "fine:            $(date '+%Y-%m-%d %H:%M:%S')"
    echo "comando:         BOX=\"$BOX\" VEL_MAX=$VEL_MAX QUOTA_ZONA=$QUOTA_ZONA TEMPO_MAX=$TEMPO_MAX TEMPO_VOLO_MAX=$TEMPO_VOLO_MAX VISITA=$VISITA GIRO_INIZIALE=$GIRO_INIZIALE PARTENZA_Z=$PARTENZA_Z LIDAR_POS=\"$LIDAR_POS\" MAPPA_REALISTICA=$MAPPA_REALISTICA ./scripts/run_reale_exploration.sh $([ "$MODO" = sim ] && echo "sim $MONDO_SIM" || echo "$PORTA")"
    echo "modo:            $([ "$MODO" = sim ] && echo "prova in simulazione ($MONDO_NOME)" || echo "drone vero")"
    echo "esito:           $ESITO"
    echo "esplorazione finita: $(grep -c 'has finished expl' "$LOG_DIR/exploration.log" 2>/dev/null)/1"
    echo "divergenze FAST-LIO: $(grep -c 'DIVERGENZA FAST-LIO' "$LOG_DIR/exploration.log" 2>/dev/null)"
    [ "$MODO" = "reale" ] && echo "registro di volo: sulla scheda SD del Pixhawk (.ulg di questo volo)"
  } > "$dst/run.txt"
  cp "$LOG_DIR/run.log" "$dst/rel_run_1.log" 2>/dev/null
  echo "    Run archiviata in $dst ($(du -sh "$dst" | cut -f1))"
}

pulizia() {
  trap '' INT TERM HUP
  echo ""
  echo "[pulizia] Spegnimento (la stima per ultima, a drone disarmato)..."
  [ -n "$VIA_PID" ] && kill -TERM "$VIA_PID" 2>/dev/null
  if [ "$MISSIONE_AVVIATA" = "1" ]; then
    # Se per qualunque motivo il drone e' ancora armato: atterraggio, e si aspetta il disarmo.
    python3 "${SORVEGLIA[@]}" atterra --attesa 90 --registro "$LOG_DIR/sorveglianza.log"
  fi
  ferma missione 10
  if [ -n "$REGISTRATORE" ] && kill -0 "$REGISTRATORE" 2>/dev/null; then
    kill -INT "$REGISTRATORE" 2>/dev/null
    for _ in $(seq 1 300); do kill -0 "$REGISTRATORE" 2>/dev/null || break; sleep 1; done
  fi
  ferma stima 10
  ferma livox 10
  ferma agente 5
  if [ -n "${PID[sim]}" ]; then
    # Lo script della simulazione fa la sua pulizia (PX4, Gazebo, nodi) con SIGTERM.
    kill -TERM "${PID[sim]}" 2>/dev/null
    for _ in $(seq 1 30); do kill -0 "${PID[sim]}" 2>/dev/null || break; sleep 1; done
  fi
  archivia_run
  echo "[pulizia] fatto."
}
trap pulizia EXIT
trap 'exit 130' INT TERM HUP

echo "=== C2-Explorer sul drone $([ "$MODO" = sim ] && echo "SIMULATO (prova generale)" || echo VERO): missione nel recinto BOX=$BOX ==="

# ---------------------------------------------------------------- [0] verifiche
echo "[0/7] Verifiche e configurazione..."
# Residui di sessioni precedenti: un secondo agente sulla stessa seriale, un secondo FAST-LIO
# sullo stesso topic o una simulazione accesa (che pubblica /fmu/out come il Pixhawk)
# falserebbero tutto. Non si uccidono da qui: potrebbero appartenere a un'altra sessione con
# il drone in aria. Esclusi questo script e i suoi antenati.
MIEI="$$"
p=$$
while [ -n "$p" ] && [ "$p" -gt 1 ]; do MIEI="$MIEI|$p"; p=$(ps -o ppid= -p "$p" 2>/dev/null | tr -d ' '); done
SCHEMA="MicroXRCEAgent|fastlio_mapping|exploration_node|c2x_px4_bridge|bin/px4|gz sim|run_multi_exploration\.sh|registra_mappa\.py"
# Il driver Livox conta solo sul drone vero: in simulazione non c'e' un lidar che pubblichi.
[ "$MODO" = "reale" ] && SCHEMA="$SCHEMA|livox_ros_driver2_node"
RESIDUI=$(ps -eo pid,args | grep -E "$SCHEMA" | grep -v grep | grep -vE "^ *($MIEI) ")
if [ -n "$RESIDUI" ]; then
  echo "ERRORE: processi di una sessione precedente ancora attivi:"
  echo "$RESIDUI" | sed 's/^/    /'
  echo "Fermali (a drone disarmato!) e riprova."
  exit 1
fi
if [ "$MODO" = "reale" ]; then
  [ -e "$PORTA" ] || { echo "ERRORE: nessun $PORTA: Pixhawk non collegato via USB? (ls /dev/ttyACM*)"; exit 1; }
  [ -r "$PORTA" ] && [ -w "$PORTA" ] || { echo "ERRORE: permessi su $PORTA: sudo usermod -aG dialout $USER, poi rientra"; exit 1; }
  command -v MicroXRCEAgent >/dev/null || { echo "ERRORE: MicroXRCEAgent non trovato (Micro-XRCE-DDS-Agent)"; exit 1; }
  ros2 pkg executables livox_ros_driver2 2>/dev/null | grep -q livox_ros_driver2_node || {
    echo "ERRORE: driver Livox (livox_ros_driver2 con Livox-SDK2) non installato: vedi scripts/collaudo/LEGGIMI_COLLAUDO.md"; exit 1; }
  [ -t 0 ] || { echo "ERRORE: serve un terminale vero (i tasti di emergenza del sorvegliante)"; exit 1; }
fi
rm -rf "$LOG_DIR/mondo"
for f in exploration stima livox agente_dds sorveglianza via registra_mappa_1 simulazione trigger; do
  : > "$LOG_DIR/$f.log"
done
rm -f /tmp/c2x_explored_map_drone1.pcd

PARTENZA_XY="0,0"
SPAWN_POSES_ARG="0,0,$PARTENZA_Z,0"
MONDO_NOME=reale
if [ "$MODO" = "sim" ]; then
  # ---------------------------------------------------------- [1-3] simulazione
  # Gazebo, PX4 (con i limiti di collaudo_px4.params, PX4_COLLAUDO=1), FAST-LIO e ponte della
  # stima: run_multi_exploration.sh con SOLO_STIMA=1, come avvia_collaudo.sh sim. Al posto di
  # agente seriale, driver Livox e launch reale:=true parte:=stima.
  echo "[1-3/7] Simulazione al posto del drone: $MONDO_SIM, PX4, FAST-LIO (registro $LOG_DIR/simulazione.log)..."
  PATH_RUN=$(echo "$PATH" | tr ':' '\n' | grep -v '^/opt/ros' | paste -sd:)
  # BOX/SPAWN/RECINTO di questo script NON vanno alla simulazione: prepara_mondo.py li
  # leggerebbe come zona e partenze del mondo, invece delle partenze tarate.
  setsid env -u BOX -u SPAWN -u RECINTO PATH="$PATH_RUN" SOLO_STIMA=1 ODOM_SOURCE=lio MAPPA_REALISTICA=0 FOTO=0 \
    PF_MIN_RAM_GB="${PF_MIN_RAM_GB:-3}" MOSTRA_MAPPA=0 \
    "$SCRIPT_DIR/run_multi_exploration.sh" "$MONDO_SIM" 1 <<< "N" > "$LOG_DIR/simulazione.log" 2>&1 &
  PID[sim]=$!
  for _ in $(seq 1 300); do
    grep -q "Pronto per il collaudo" "$LOG_DIR/simulazione.log" && break
    grep -qE "fallita|ERRORE|non utilizzabile" "$LOG_DIR/simulazione.log" && break
    kill -0 "${PID[sim]}" 2>/dev/null || break
    sleep 1
  done
  if ! grep -q "Pronto per il collaudo" "$LOG_DIR/simulazione.log"; then
    echo "ERRORE: simulazione non pronta:"; tail -5 "$LOG_DIR/simulazione.log" | sed 's/^/    /'; exit 1
  fi
  # La partenza del drone simulato (il recinto resta relativo al decollo).
  # shellcheck disable=SC1091
  source /tmp/c2x_run_multi/mondo/mondo.env
  IFS=',' read -r sx sy sz _sr _sp syaw <<< "${SPAWN_POSES[0]}"
  PARTENZA_XY="$sx,$sy"
  PARTENZA_Z="$sz"
  SPAWN_POSES_ARG="$sx,$sy,$sz,${syaw:-0}"
  echo "    simulazione pronta (partenza $SPAWN_POSES_ARG)."
fi

if ! /usr/bin/python3 "$COLLAUDO_DIR/prepara_reale.py" --box="$BOX" --quota-zona "$QUOTA_ZONA" \
       --tempo-max "$TEMPO_MAX" --partenza="$PARTENZA_XY" --uscita "$LOG_DIR/mondo"; then
  echo "Recinto non utilizzabile."
  exit 1
fi
MONDO_CONFIG="$LOG_DIR/mondo/config.yaml"

LAUNCH_COMUNE=(odom_source:=lio drone_num:=1 config_name:="$MONDO_CONFIG" spawn_poses:="$SPAWN_POSES_ARG;"
               initial_spin_rate:="$GIRO_INIZIALE" max_vel:="$VEL_MAX" max_flight_z:="$MAX_FLIGHT_Z"
               photo_completion:=false photo_yaw:=false
               visit_completion:="$([ "$VISITA" = 1 ] && echo true || echo false)")
if [ "$MODO" = "reale" ]; then
  LAUNCH_COMUNE+=(reale:=true fast_lio_config:="$COLLAUDO_DIR/fast_lio_mid360.yaml" lidar_offset:="$LIDAR_POS")

  # ---------------------------------------------------------- [1] agente DDS
  echo "[1/7] Agente DDS su $PORTA a $BAUD baud..."
  avvia agente "$LOG_DIR/agente_dds.log" MicroXRCEAgent serial --dev "$PORTA" -b "$BAUD"
  for _ in $(seq 1 20); do
    grep -qi "session established" "$LOG_DIR/agente_dds.log" && break
    vivo agente || break
    sleep 1
  done
  vivo agente || { echo "ERRORE: l'agente DDS si e' fermato:"; tail -5 "$LOG_DIR/agente_dds.log"; exit 1; }
  grep -qi "session established" "$LOG_DIR/agente_dds.log" && echo "    OK - PX4 collegato." \
    || echo "    nessuna sessione ancora (lo verificano i controlli a terra)."

  # ---------------------------------------------------------- [2] driver Livox
  echo "[2/7] Driver del Mid-360 ($LIVOX_LAUNCH)..."
  avvia livox "$LOG_DIR/livox.log" ros2 launch livox_ros_driver2 "$LIVOX_LAUNCH"
  sleep 3
  vivo livox || { echo "ERRORE: il driver Livox si e' fermato:"; tail -5 "$LOG_DIR/livox.log"; exit 1; }

  # ---------------------------------------------------------- [3] stima
  echo "[3/7] Stima: FAST-LIO sul Mid-360 e ponte verso PX4..."
  avvia stima "$LOG_DIR/stima.log" ros2 launch exploration_manager multi_drone_exploration.launch.py \
    "${LAUNCH_COMUNE[@]}" parte:=stima
  sleep 5
  vivo stima || { echo "ERRORE: la stima si e' fermata:"; tail -5 "$LOG_DIR/stima.log"; exit 1; }
fi

# ---------------------------------------------------------------- [4] controlli
echo "[4/7] Controlli a terra..."
CONTROLLI=(controlli --durata 8 --registro "$LOG_DIR/sorveglianza.log")
[ "$SENZA_COLLAUDO" = "1" ] || [ "$MODO" = "sim" ] && CONTROLLI+=(--senza-collaudo)
python3 "${SORVEGLIA[@]}" "${CONTROLLI[@]}" || { echo "Controlli non superati: niente volo."; exit 1; }

# ---------------------------------------------------------------- [5] consenso
echo ""
echo "[5/7] Pronto al decollo. Recinto x ${BOX} (x avanti, y sinistra), quota fino a $QUOTA_ZONA m,"
echo "      velocita' $VEL_MAX m/s, esplorazione al massimo $TEMPO_MAX s, volo al massimo $TEMPO_VOLO_MAX s."
echo "      Il drone DECOLLA SUBITO dopo il consenso. In volo: SPAZIO = EMERGENZA, l = atterra."
read -r -p "      Scrivi VOLA per decollare (altro = esci): " CONSENSO
if [ "$CONSENSO" != "VOLA" ]; then
  echo "Nessun consenso: niente volo."
  exit 0
fi

# ---------------------------------------------------------------- [6] missione
echo "[6/7] Missione: ponte dei comandi (arma e decolla), pianificatore..."
if [ "$MAPPA_REALISTICA" = "1" ]; then
  rm -rf "$LOG_DIR/mappa_realistica_drone1"
  # Come in run_multi_exploration.sh, ma solo lidar (nessuna telecamera collegata). HUP
  # ignorato: se cade la sessione SSH la mappa va salvata lo stesso.
  ( trap '' HUP; exec nice -n 10 python3 "$SCRIPT_DIR/registra_mappa.py" --drone 1 \
      --out "$LOG_DIR/mappa_realistica_drone1" ) > "$LOG_DIR/registra_mappa_1.log" 2>&1 &
  REGISTRATORE=$!
fi
MISSIONE_AVVIATA=1
ESITO="missione avviata"
# 3 s di ritardo: il sorvegliante (piu' sotto) deve gia' ricevere stima e stato di PX4
# quando il ponte dei comandi arma.
avvia missione "$LOG_DIR/exploration.log" bash -c 'sleep 3; exec "$@"' _ \
  ros2 launch exploration_manager multi_drone_exploration.launch.py "${LAUNCH_COMUNE[@]}" parte:=missione

# Il via all'esplorazione, in sottofondo: quando il drone dichiara di essere pronto (decollo
# e giro conclusi, mappa e frontiere viste) si pubblica il trigger, come in
# run_multi_exploration.sh. Se non arriva, il sorvegliante fa atterrare dopo un minuto.
(
  trap '' HUP
  if python3 "$SCRIPT_DIR/wait_ready.py" --drones 1 --timeout "$READY_TIMEOUT_S"; then
    for t in 1 2 3 4 5; do
      timeout 6 ros2 topic pub /move_base_simple/goal geometry_msgs/msg/PoseStamped \
        "{header: {frame_id: 'world'}, pose: {position: {x: 0.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}" \
        -r 5 >> "$LOG_DIR/trigger.log" 2>&1
      sleep 2
      if grep -qE "WAIT_TRIGGER to (PLAN_TRAJ|FINISH)" "$LOG_DIR/exploration.log"; then
        echo "via dato (tentativo $t)"; exit 0
      fi
    done
    echo "via non confermato dal registro"
  else
    echo "handshake di prontezza fallito"
  fi
) > "$LOG_DIR/via.log" 2>&1 &
VIA_PID=$!

VOLO=(volo --box="$BOX" --margine 0.5 --quota-max "$QUOTA_ALLARME" --partenza-z "$PARTENZA_Z"
      --pgid-missione "${PID[missione]}" --log-esplorazione "$LOG_DIR/exploration.log"
      --tempo-volo-max "$TEMPO_VOLO_MAX" --registro "$LOG_DIR/sorveglianza.log")
[ "$MODO" = "reale" ] && VOLO+=(--pid-stima "${PID[stima]}")
[ "$MODO" = "sim" ] && VOLO+=(--pid-stima "${PID[sim]}")
[ "$MODO" = "sim" ] && [ ! -t 0 ] && VOLO+=(--senza-tastiera)
python3 "${SORVEGLIA[@]}" "${VOLO[@]}"
RC=$?
case $RC in
  0) ESITO="atterrato" ;;
  *) ESITO="sorveglianza conclusa con codice $RC (vedi sorveglianza.log)" ;;
esac
# Le stesse righe di esito di run_multi_exploration.sh: analisi_campagna.py le legge.
FINITA=0
grep -q "has finished exploration" "$LOG_DIR/exploration.log" && FINITA=1
[ "$FINITA" = 1 ] && echo "    Esplorazione completata da tutti i droni!"
if grep -q "da solo (failsafe" "$LOG_DIR/sorveglianza.log"; then
  echo "    Atterraggio deciso da PX4 (failsafe?), sul posto: vedi sorveglianza.log"
  ESITO="$ESITO, atterraggio deciso da PX4"
elif [ "$RC" = 0 ] && [ "$FINITA" = 1 ] && ! grep -q "ATTERRAGGIO:" "$LOG_DIR/sorveglianza.log"; then
  echo "    Tutti i droni sono rientrati e atterrati."
elif grep -q "ATTERRAGGIO:" "$LOG_DIR/sorveglianza.log"; then
  echo "    Atterraggio comandato dal sorvegliante: $(grep -h "ATTERRAGGIO:" "$LOG_DIR/sorveglianza.log" | tail -1 | sed 's/.*ATTERRAGGIO: //')"
  ESITO="$ESITO, atterraggio comandato dal sorvegliante"
fi

# ---------------------------------------------------------------- [7] mappe e chiusura
echo "[7/7] Mappe..."
kill -TERM "$VIA_PID" 2>/dev/null
VIA_PID=""
if vivo missione; then
  timeout 60 ros2 service call /drone_1/sdf_map/save_map std_srvs/srv/Trigger > "$LOG_DIR/save_map_1.log" 2>&1
fi
[ -f /tmp/c2x_explored_map_drone1.pcd ] && echo "    mappa del pianificatore: /tmp/c2x_explored_map_drone1.pcd" \
  || echo "    ATTENZIONE: mappa del pianificatore non salvata"
ferma missione 10
if [ -n "$REGISTRATORE" ]; then
  kill -INT "$REGISTRATORE" 2>/dev/null
  for _ in $(seq 1 300); do kill -0 "$REGISTRATORE" 2>/dev/null || break; sleep 1; done
  kill -0 "$REGISTRATORE" 2>/dev/null && echo "    ATTENZIONE: il registratore della mappa non ha finito di salvare in 300 s"
  REGISTRATORE=""
  dir="$LOG_DIR/mappa_realistica_drone1"
  if [ -f "$dir/punti.npz" ]; then
    echo "    mappa 3D (superficie dal lidar)..."
    if PATH=/usr/bin:$PATH /usr/bin/python3 "$SCRIPT_DIR/ricostruisci_mappa.py" "$dir" --min_conteggio 1 \
         > "$LOG_DIR/ricostruisci_1.log" 2>&1; then
      grep "^\[ricostruisci\]" "$LOG_DIR/ricostruisci_1.log" | sed 's/^/    /'
    else
      echo "    ATTENZIONE: mappa 3D non riuscita; ultime righe di $LOG_DIR/ricostruisci_1.log:"
      tail -5 "$LOG_DIR/ricostruisci_1.log" | sed 's/^/      /'
    fi
  else
    echo "    ATTENZIONE: dati della mappa 3D non trovati in $dir"
  fi
fi
echo "=== Fine: $ESITO ==="
# Il resto (stima, driver, agente, archivio) lo fa la pulizia all'uscita.
exit 0
