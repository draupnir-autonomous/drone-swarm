#!/bin/bash
# Orchestrates the multi-UAV C2-Explorer stack:
#   N x PX4 SITL (sharing one Gazebo world) -> Micro XRCE-DDS Agent ->
#   multi_drone_exploration.launch.py (per-drone bridges + exploration stack +
#   the swarm relay) -> trigger -> wait -> save each drone's map -> render.
#
# Usage: ./run_multi_exploration.sh [mondo] [drone_num]
#   mondo:     QUALSIASI mondo Gazebo: il nome di uno dei mondi del progetto
#              (src/c2x_worlds/worlds: empty_room, cubicle_office, office_small,
#              open_field, parco) o di PX4 (Tools/simulation/gz/worlds), oppure il
#              percorso di un file .sdf. scripts/prepara_mondo.py lo prepara: zona da
#              esplorare, partenza libera, recinto se e' aperto (vedi li'). Default
#              empty_room. SPAWN="x,y[,yaw];..." e BOX="xmin,ymin,xmax,ymax" per
#              scegliere a mano partenze e zona.
#   drone_num: number of UAVs (default 3)
#
# Il risultato di ogni missione, su qualunque mondo, e' la MAPPA 3D (superficie
# colorata dalla telecamera, mostrata a fine run e archiviata in risultati_prova/).
#
# Drone i <-> PX4 SITL instance i-1 <-> Gazebo model x500_mid360_<i-1> <->
# ROS namespace /drone_i. Instance 0 keeps PX4's bare /fmu topics; instance k>0
# is namespaced /px4_k by PX4 itself.

SCENARIO="${1:-empty_room}"
# Sorgente della posa: px4 (EKF con GPS simulato) oppure lio (FAST-LIO su
# lidar+IMU). Con lio parte anche un nodo FAST-LIO per drone.
#   ODOM_SOURCE=lio ./scripts/run_multi_exploration.sh office_small 1
ODOM_SOURCE="${ODOM_SOURCE:-px4}"
# Volo senza GPS: con ODOM_SOURCE=lio si spegne anche il GNSS dentro PX4 e
# l'autopilota si regge sulla visione esterna. Si puo' forzare separatamente per
# provare le due cose in isolamento (PX4_GPS_DENIED=0 tiene il GPS acceso pur
# usando FAST-LIO per l'esplorazione).
# ODOM_SOURCE=livo: come lio, ma con FAST-LIVO2 (lidar + IMU + telecamera) al posto di
# FAST-LIO (ottobre 2026, prova); la telecamera arriva in ROS da registra_mappa.py, quindi
# servono MAPPA_REALISTICA=1 e FOTO=1 (i valori di serie).
if [ "$ODOM_SOURCE" = "lio" ] || [ "$ODOM_SOURCE" = "livo" ]; then
  PX4_GPS_DENIED="${PX4_GPS_DENIED:-1}"
else
  PX4_GPS_DENIED="${PX4_GPS_DENIED:-0}"
fi
export PX4_GPS_DENIED
DRONE_NUM="${2:-3}"

ROS2_SETUP=/opt/ros/jazzy/setup.bash
# MicroXRCEAgent e' installato in ~/.local/bin (compilato senza sudo).
export PATH="$HOME/.local/bin:$PATH"
# Percorsi RELATIVI alla posizione di questo script: il progetto si puo'
# spostare o copiare su un'altra macchina senza toccare nulla.
#   <progetto>/ros2_ws/scripts/<questo file>
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Trasporto DDS: memoria condivisa con segmento grande, cosi' le scansioni lidar
# (~650 KB) non vengono spezzate in frammenti UDP e perse a raffica. Vedi il
# commento in fastdds_profilo.xml. Vale per tutti i processi lanciati da qui.
export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/fastdds_profilo.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/fastdds_profilo.xml"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PROJECT_DIR="$(cd "$WS_DIR/.." && pwd)"
WS_SETUP=$WS_DIR/install/setup.bash
PX4_DIR=$PROJECT_DIR/PX4-Autopilot
PX4_BUILD=$PX4_DIR/build/px4_sitl_default
LOG_DIR=/tmp/c2x_run_multi
# Archivio automatico: a fine run tutto quello che serve a rivederla si copia in
# risultati_prova/<data_ora>_<scenario>_<odom>_v<VEL_MAX>/, nel formato rel_*
# delle campagne (video_replay3d.py, video_demo.py, analisi_campagna.py la leggono
# cosi' com'e'). /tmp/c2x_run_multi viene sovrascritta dalla run successiva e i
# .ulg vengono potati da preflight.sh dopo un giorno: senza archivio una run
# andava persa. ARCHIVIA=0 lo spegne (reliability_campaign.sh archivia per conto suo).
ARCHIVIA="${ARCHIVIA:-1}"
RISULTATI_DIR="$PROJECT_DIR/risultati_prova"
RUN_INIZIO=$(date +%Y-%m-%d_%H%M)
RUN_T0=$(date +%s)
ARCHIVIATA=0
# Mappa realistica (superficie 3D colorata dalla telecamera): vedi
# scripts/registra_mappa.py e scripts/ricostruisci_mappa.py. MAPPA_REALISTICA=0
# la disattiva (la telecamera non viene nemmeno resa da Gazebo).
MAPPA_REALISTICA="${MAPPA_REALISTICA:-1}"
# SOLO_STIMA=1 (collaudo, ottobre 2026): Gazebo, PX4, sensori, stimatore e ponte della
# stima verso PX4, nient'altro (ne' pianificatore ne' ponte dei comandi ne' mappa 3D). Lo
# usa scripts/collaudo/avvia_collaudo.sh sim: a comandare PX4 e' collaudo.py.
SOLO_STIMA="${SOLO_STIMA:-0}"
if [ "$SOLO_STIMA" = "1" ]; then
  MAPPA_REALISTICA=0
  if [ "$ODOM_SOURCE" != "lio" ] && [ "$ODOM_SOURCE" != "livo" ]; then
    echo "SOLO_STIMA=1 serve con ODOM_SOURCE=lio (o livo)."; exit 1
  fi
fi
# Telecamera per colorare la mappa 3D: di serie SI' (richiesta, 17 ottobre 2026),
# ma solo per questo. registra_mappa.py tiene un fotogramma chiave ogni 0.3 m o 10
# gradi in /tmp/c2x_run_multi/mappa_realistica_drone<N>/fotogrammi, che
# ricostruisci_mappa.py usa per colorare la superficie: sono file di lavoro, non
# si archiviano in risultati_prova/ e la run successiva li sovrascrive. Nessuna
# fase fotografica (FASE_FOTO), nessuna imbardata verso le foto (IMBARDATA_FOTO),
# nessun video (lo streaming di PX4, GstCameraSystem, e' tolto da server.config).
# FOTO=0 = mappa 3D grigia, telecamera mai resa da Gazebo.
FOTO="${FOTO:-1}"
if [ "$ODOM_SOURCE" = "livo" ] && { [ "$MAPPA_REALISTICA" != "1" ] || [ "$FOTO" != "1" ]; }; then
  echo "ODOM_SOURCE=livo usa la telecamera portata in ROS da registra_mappa.py: servono MAPPA_REALISTICA=1 e FOTO=1."
  exit 1
fi
# Giro di 360 gradi a quota di decollo prima del via, in rad/s (0 = nessun
# giro): vedi initial_spin_rate in position_command_bridge.
GIRO_INIZIALE="${GIRO_INIZIALE:-0.5}"
# Finestra interattiva sulla mappa 3D a fine run (ruotabile col mouse): vedi
# scripts/mostra_mappa.py. MOSTRA_MAPPA=0 la disattiva (le campagne lo fanno).
MOSTRA_MAPPA="${MOSTRA_MAPPA:-1}"
# Velocita' massima del pianificatore in m/s (tutti i moduli insieme).
# 0.9 dalla campagna V14 (3 run per valore, senza GPS, office_small):
#   0.6 -> volo 136-148 s, margine minimo dalle pareti 0.82-0.85 m
#   0.9 -> volo  74-90 s,  margine 0.80-0.85 m, copertura 3D 99.7%
#   1.2 -> volo  66-74 s,  margine 0.65-0.70 m, velocita' vera fino a 1.7 m/s
# 0.9 dimezza il volo senza perdere nulla; 1.2 toglie altri 8 s ma avvicina il
# drone alle pareti e supera il tetto di velocita' di PX4.
# Col planner corretto (V15): 0.9 -> missione 88-91 s, margine 0.60-0.81 m;
# 1.2 -> 74-82 s, margine 0.40-0.70 m. 1.0 su richiesta (5 ottobre 2026), misurata
# nelle campagne V29-V30: 8/8 run pulite fra office_small e cubicle_office.
VEL_MAX="${VEL_MAX:-1.0}"
# Fase fotografica dopo l'esplorazione: 1 = il drone, finite le frontiere, va a
# fotografare le zone della mappa 3D rimaste grigie prima di rientrare (fino a
# 300 s; su cubicle_office mappa fotografata 97.6-99.0% con la fase, V30).
# 0 (di serie) = solo esplorazione e rientro.
FASE_FOTO="${FASE_FOTO:-0}"
# Imbardata da foto in esplorazione: 1 = la telecamera si gira verso le superfici
# non ancora fotografate (~2400 gradi/min di rotazione contro ~900); 0 (di serie)
# = guarda nella direzione di marcia. Con la fase di visita (il drone passa
# ovunque) la telecamera in avanti fotografa gia' la mappa 3D: su cubicle_office
# grigio 1.6-2.7% senza contro 5.6% con, e la visita finisce prima (V33).
IMBARDATA_FOTO="${IMBARDATA_FOTO:-0}"
# Fase di visita dopo l'esplorazione: 1 (di serie) = il drone passa in tutto lo
# spazio percorribile, anche nei corridoi ciechi che il lidar "chiude" da lontano
# (cubicle_office: 99.5-99.9% visitato, missione ~9 min, V35); 0 = solo
# esplorazione (79-86% visitato, ~5.5 min, V31).
VISITA="${VISITA:-1}"
# TEMPO SIMULATO per tutto lo stack (sciame, ottobre 2026): vedi tempo_sim nel launch.
# Con tre droni il Jetson tiene la simulazione sotto RTF 0.4 e lo stack a orologio di
# parete correva 2-3 volte piu' veloce della fisica. 0 = comportamento di prima (solo
# FAST-LIO a tempo simulato). Tutti i tempi di missione delle pagelle sono comunque
# tempi SIMULATI (registro di volo di PX4), cioe' quelli che avrebbe il volo vero.
TEMPO_SIM="${TEMPO_SIM:-1}"
# Collegamento radio fra i droni (vedi communication.* nelle configurazioni): di serie
# quelli del file, 60 m senza linea di vista (Wi-Fi realistico, ottobre 2026). RAGGIO_RADIO
# in metri e LINEA_VISTA=0/1 li cambiano per tutti i droni: RAGGIO_RADIO=10 LINEA_VISTA=1
# e' il modello del paper (collegamento ottico a corto raggio).
RAGGIO_RADIO="${RAGGIO_RADIO:-}"
LINEA_VISTA="${LINEA_VISTA:-}"
REGISTRATORI=()
# Quanto si aspetta il "finished exploration" prima di salvare le mappe e uscire.
# Era 900 s, e siccome agisce indipendentemente dal tetto della campagna,
# all'esplorazione non sono mai stati concessi piu' di 900 s: ogni run finiva a
# ~972 s (900 + avvio) con finiti=0/3, e sembrava che l'esplorazione non
# completasse mai quando in realta' non le era mai stato dato il tempo di
# provarci. cubicle_office e' 31x31 m percorsi a 0.6 m/s da tre droni.
FINISH_TIMEOUT_S="${FINISH_TIMEOUT_S:-2400}"
WARMUP_S="${WARMUP_S:-0}"
                       # Azzerato: l'handshake di prontezza verifica la
                       # condizione VERA (ogni drone dichiara odometria, mappa e
                       # frontiere viste), quindi non resta niente da
                       # approssimare con un cronometro.
                       # Era 30 s fissi, ed erano il 33% della finestra di
                       # missione: i droni stavano fermi ad accumulare la prima
                       # scansione perche' altrimenti il primo controllo delle
                       # frontiere non trovava nulla e l'automa concludeva
                       # "niente da esplorare". Ora quel caso e' gestito nel
                       # codice - triggerCallback non conclude piu' su zero
                       # frontiere e il ramo IDLE ha un periodo di grazia
                       # (min_explore_time_) - quindi qui basta il tempo di far
                       # arrivare la prima nuvola.
STALE_WARN_S=30
MAX_TRIGGER_ATTEMPTS=5
# Quanto si aspetta che TUTTI i droni raggiungano WAIT_TRIGGER, a partire dal
# lancio dello stack di esplorazione. Misurato: i nodi impiegano 9-11 s: non e'
# l'allocazione della mappa a pesare, ma l'attesa della prima odometria da PX4,
# che dipende da quanto e' carica la macchina. Erano 45 s fissi e su open_field
# la run veniva abortita prima di cominciare; 60 s sono cinque volte il tempo
# misurato senza allungare inutilmente ogni run di una campagna.
# 120 s: la prontezza ora comprende anche decollo e giro iniziale (~25 s).
READY_TIMEOUT_S="${READY_TIMEOUT_S:-120}"

if ! [[ "$DRONE_NUM" =~ ^[0-9]+$ ]] || [ "$DRONE_NUM" -lt 1 ]; then
  echo "drone_num non valido: $DRONE_NUM (serve un intero >= 1)"
  exit 1
fi
# Le attese qui sotto sono in secondi di PARETE, mentre lo stack (con TEMPO_SIM=1) e i
# suoi tetti di tempo (fsm.max_explore_time, fase di visita) contano secondi SIMULATI: con
# N droni il Jetson simula a ~1/N della velocita' di un drone solo, quindi le attese di
# serie crescono con N. Restano solo reti di sicurezza contro un blocco: la fine vera la
# segnalano i droni nel registro.
if [ "$DRONE_NUM" -gt 1 ] && [ "$TEMPO_SIM" = "1" ]; then
  FINISH_TIMEOUT_S="${FINISH_TIMEOUT_S_SCIAME:-$((FINISH_TIMEOUT_S * DRONE_NUM))}"
  READY_TIMEOUT_S="${READY_TIMEOUT_S_SCIAME:-$((READY_TIMEOUT_S * DRONE_NUM))}"
fi
# Registratore della mappa 3D: uno solo, anche per lo sciame (mappa FUSA, vedi
# registra_mappa.py). Un drone: la cartella di sempre.
if [ "$DRONE_NUM" -gt 1 ]; then
  MAPPA_DIR="$LOG_DIR/mappa_realistica_sciame"
else
  MAPPA_DIR="$LOG_DIR/mappa_realistica_drone1"
fi

mkdir -p "$LOG_DIR"
# Copia di tutto quello che lo script scrive a terminale: finisce nell'archivio
# come rel_run_1.log (il formato delle campagne). tee ignora Ctrl+C: se morisse
# con lo script, la prima riga scritta dalla pulizia ucciderebbe lo script (SIGPIPE)
# a meta' pulizia, lasciando PX4 e Gazebo accesi. Esce da solo a fine script.
exec > >(trap '' INT TERM; exec tee "$LOG_DIR/run.log") 2>&1

# Archivia la run in risultati_prova/ (vedi ARCHIVIA in cima). Senza punti.npz,
# la nuvola lidar grezza da 100-400 MB: la mappa 3D finita (.ply) basta per
# guardarla e per i video; serve solo a rifare la ricostruzione, e in quel caso va
# copiata a mano da /tmp/c2x_run_multi/mappa_realistica_drone<N>/ prima della run
# successiva. $1 = suffisso del nome ("_interrotta" se la run non e' arrivata alla fine).
archivia_run() {
  [ "$ARCHIVIA" = "1" ] && [ "$ARCHIVIATA" = "0" ] || return 0
  [ -s "$LOG_DIR/exploration.log" ] || return 0
  ARCHIVIATA=1
  local nome="${RUN_INIZIO}_${MONDO_NOME:-mondo}_${ODOM_SOURCE}_v${VEL_MAX}"
  [ "$DRONE_NUM" -gt 1 ] && nome="${nome}_${DRONE_NUM}droni"
  local dst="$RISULTATI_DIR/${nome}$1"
  [ -e "$dst" ] && dst="${dst}_$(date +%S)"
  mkdir -p "$dst" || { echo "    ATTENZIONE: archivio non creato in $dst"; return 1; }
  cp "$LOG_DIR/exploration.log" "$dst/rel_expl_1.log"
  # Il mondo come e' stato volato: descrizione (origine, partenze, zona), mondo
  # normalizzato e configurazione dello stack.
  cp "$LOG_DIR/mondo/mondo.json" "$dst/rel_mondo.json" 2>/dev/null
  cp "$LOG_DIR/mondo/config.yaml" "$dst/rel_config.yaml" 2>/dev/null
  cp "$LOG_DIR/mondo/$MONDO_NOME.sdf" "$dst/rel_mondo.sdf" 2>/dev/null
  # Mappa del planner (drone 1 senza suffisso, come in reliability_campaign.sh).
  cp /tmp/c2x_explored_map_drone1.pcd "$dst/rel_map_1.pcd" 2>/dev/null
  local d i src ulg
  for d in $(seq 2 "$DRONE_NUM"); do
    cp "/tmp/c2x_explored_map_drone$d.pcd" "$dst/rel_map_1_$d.pcd" 2>/dev/null
  done
  # Mappa 3D: un drone -> rel_mappa3d_1_1 come sempre; sciame -> la mappa FUSA di tutti i
  # droni in rel_mappa3d_1_sciame.
  if [ -d "$MAPPA_DIR" ]; then
    local dmappa="$dst/rel_mappa3d_1_1"
    [ "$DRONE_NUM" -gt 1 ] && dmappa="$dst/rel_mappa3d_1_sciame"
    mkdir -p "$dmappa"
    cp "$MAPPA_DIR"/anteprima_*.png "$MAPPA_DIR"/mappa_realistica.ply "$MAPPA_DIR"/nuvola_colorata.ply \
      "$dmappa/" 2>/dev/null
    # I fotogrammi della telecamera NON si archiviano: servono solo a colorare la
    # mappa 3D, gia' fatta qui sopra (vedi FOTO in cima). Le loro pose si' (piccole):
    # dicono quale drone ha fotografato cosa.
    cp "$MAPPA_DIR/fotogrammi/pose.json" "$dmappa/pose_fotogrammi.json" 2>/dev/null
  fi
  for i in $(seq 0 $((DRONE_NUM - 1))); do
    cp "$LOG_DIR/px4_$i.log" "$dst/rel_px4_1_$i.log" 2>/dev/null
    # Registro di volo: il piu' recente dell'istanza, ma solo se scritto durante
    # QUESTA run (altrimenti, se PX4 non ha registrato, si archivierebbe quello
    # di una run precedente).
    ulg=$(find "$PX4_BUILD/rootfs/instance_$i/log" -name '*.ulg' -newermt "@$RUN_T0" \
            -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -1 | cut -d' ' -f2-)
    [ -n "$ulg" ] && cp "$ulg" "$dst/rel_ulog_1_$i.ulg"
  done
  local cpu_min cpu_max
  cpu_min=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq 2>/dev/null)
  cpu_max=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq 2>/dev/null)
  {
    echo "inizio:          $(date -d "@$RUN_T0" '+%Y-%m-%d %H:%M:%S')"
    echo "fine:            $(date '+%Y-%m-%d %H:%M:%S')${1:+  (run interrotta)}"
    echo "comando:         ODOM_SOURCE=$ODOM_SOURCE VEL_MAX=$VEL_MAX VISITA=$VISITA FASE_FOTO=$FASE_FOTO IMBARDATA_FOTO=$IMBARDATA_FOTO GIRO_INIZIALE=$GIRO_INIZIALE MAPPA_REALISTICA=$MAPPA_REALISTICA FOTO=$FOTO TEMPO_SIM=$TEMPO_SIM${RAGGIO_RADIO:+ RAGGIO_RADIO=$RAGGIO_RADIO}${LINEA_VISTA:+ LINEA_VISTA=$LINEA_VISTA}${PX4_VEL_P_ACC:+ PX4_VEL_P_ACC=$PX4_VEL_P_ACC}${SALTO_RIFERIMENTO:+ SALTO_RIFERIMENTO=$SALTO_RIFERIMENTO}${SPAWN:+ SPAWN=\"$SPAWN\"}${BOX:+ BOX=\"$BOX\"} ./scripts/run_multi_exploration.sh $SCENARIO $DRONE_NUM"
    echo "droni:           $DRONE_NUM"
    echo "RTF medio:       $( [ -s "$LOG_DIR/rtf.txt" ] && awk '{s+=$1; n++} END {if (n) printf "%.2f (%d campioni)", s/n, n}' "$LOG_DIR/rtf.txt" || echo '-')"
    echo "mondo:           $MONDO_NOME ($MONDO_ORIGINE)"
    echo "GPS in PX4:      $([ "$PX4_GPS_DENIED" = 1 ] && echo spento || echo acceso)"
    echo "GUI di Gazebo:   ${SHOW_GUI:-n}"
    echo "macchina:        $(uname -m), clock CPU fissati: $([ -n "$cpu_min" ] && [ "$cpu_min" = "$cpu_max" ] && echo si || echo no)"
    echo "esplorazione finita: $(grep -c 'has finished expl' "$LOG_DIR/exploration.log")/$DRONE_NUM droni"
    echo "crash rilevati:      $(grep -c 'CRASH DETECTED' "$LOG_DIR/exploration.log")"
    echo "divergenze FAST-LIO: $(grep -c 'DIVERGENZA FAST-LIO' "$LOG_DIR/exploration.log")"
  } > "$dst/run.txt"
  cp "$LOG_DIR/run.log" "$dst/rel_run_1.log" 2>/dev/null
  echo "    Run archiviata in $dst ($(du -sh "$dst" | cut -f1))"
}

# All pkill/rm on /dev/shm below only ever run either before anything is
# launched, or in the EXIT trap after everything has been asked to die - never
# while the stack is live. Cleaning DDS shared-memory segments out from under
# still-running nodes corrupts their discovery state.

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
  pkill -9 -f "parameter_bridge" 2>/dev/null
  pkill -9 -f "c2x_px4_bridge" 2>/dev/null
  pkill -9 -f "exploration_node" 2>/dev/null
  pkill -9 -f "mtsp_node" 2>/dev/null
  pkill -9 -f "plan_manage/traj_server" 2>/dev/null
  pkill -9 -f "swarm_relay" 2>/dev/null
  pkill -9 -f "fastlio_mapping" 2>/dev/null
  pkill -9 -f "registra_mappa.py" 2>/dev/null
  # Run chiusa a meta' (Ctrl+C in volo, errore): se lo stack era partito si
  # archivia quello che c'e'. A fine run normale e' gia' archiviata.
  # Le sessioni di collaudo (SOLO_STIMA=1) non sono missioni: niente archivio in risultati_prova.
  [ -n "$EXPL_PID" ] && [ "$SOLO_STIMA" != "1" ] && archivia_run "_interrotta"
}
# Con Ctrl+C o SIGTERM a meta' run, dopo la pulizia si ESCE (ottobre 2026): prima il trap
# ripuliva e poi lo script riprendeva il ciclo di attesa in cui si trovava, girando a vuoto
# fino ai tetti di tempo (fino a 2400 s con PX4 e Gazebo gia' spenti). La pulizia e'
# idempotente (l'archivio e' protetto da ARCHIVIATA), quindi il secondo giro su EXIT non nuoce.
trap cleanup EXIT
trap 'cleanup; exit 130' INT TERM

# Fattore di tempo reale di Gazebo (simulato/parete), un campione: va in rtf.txt e nel
# riepilogo della run. Con piu' droni dice quanto piu' lenta del vero e' la simulazione
# (e quindi quanto dura in parete una missione di N secondi simulati).
campiona_rtf() {
  local rtf
  rtf=$(timeout 5 gz topic -e -n 1 -t "/world/$PX4_GZ_WORLD/stats" 2>/dev/null |
        awk '/real_time_factor/ {print $2; exit}')
  [ -n "$rtf" ] && echo "$rtf" >> "$LOG_DIR/rtf.txt" && echo "$rtf"
}

print_progress() {
  local log_mtime now stale_s
  for d in $(seq 1 "$DRONE_NUM"); do
    local last_line
    last_line=$(grep -E "drone_$d\]|Drone $d " "$LOG_DIR/exploration.log" 2>/dev/null |
      grep -E "\[FSM\]:|Triggered|finished exploration|No coverable frontier|CRASH DETECTED" |
      tail -1)
    [ -z "$last_line" ] && last_line="(nessuna attivita registrata ancora)"
    echo "  [drone $d] ${last_line}"
  done
  log_mtime=$(stat -c %Y "$LOG_DIR/exploration.log" 2>/dev/null || echo 0)
  now=$(date +%s)
  stale_s=$((now - log_mtime))
  if [ "$stale_s" -ge "$STALE_WARN_S" ]; then
    echo "    ATTENZIONE: nessuna nuova riga di log da ${stale_s}s - potrebbe essersi bloccato"
    echo "    (o e' semplicemente impegnato a ricalcolare la mappa su una scena complessa)."
  fi
}

echo "=== C2-Explorer multi-UAV: avvio (mondo: $SCENARIO, droni: $DRONE_NUM) ==="
read -r -p "Vuoi vedere la GUI 3D di Gazebo? (y/N - no e' piu' leggero per la CPU e per la reattivita' del planner): " SHOW_GUI
SHOW_GUI=${SHOW_GUI:-n}

# --- Il mondo: qualsiasi .sdf, preparato da prepara_mondo.py ---
# Scrive in $LOG_DIR/mondo il mondo normalizzato, la configurazione dello stack e
# mondo.env (nome, percorsi, pose di partenza "x,y,z,roll,pitch,yaw"). Per i mondi
# gia' tarati (cubicle_office, parco, ...) pose e configurazione sono quelle misurate
# nelle campagne, per gli altri si ricavano dalla geometria del mondo.
# QUOTA DI PARTENZA 0.227 m in tutte le pose: e' l'altezza a cui sta il corpo
# dell'x500 appoggiato sui pattini (pattini 0.2195 + 0.0075 sotto base_link, vedi
# x500_base/model.sdf). La stessa z va ai ponti come origine della terna locale nel
# mondo. Prima era 0.1 (quota di lancio, con i pattini 13 cm sotto il pavimento:
# Gazebo faceva rimbalzare il modello fino a 0.227) e tutta la mappa, col GPS e
# senza, risultava 12.7 cm piu' bassa del vero.
rm -rf "$LOG_DIR/mondo"
if ! PATH=/usr/bin:$PATH /usr/bin/python3 "$SCRIPT_DIR/prepara_mondo.py" "$SCENARIO" \
       --droni "$DRONE_NUM" --uscita "$LOG_DIR/mondo"; then
  echo "Mondo non utilizzabile: non avvio la simulazione."
  exit 1
fi
# shellcheck disable=SC1091
source "$LOG_DIR/mondo/mondo.env"
PX4_GZ_WORLD="$MONDO_NOME"
# Modelli del mondo (accanto al file, del progetto, di PX4, ...): Gazebo li trova da
# qui, senza rete per quelli gia' presenti.
export GZ_SIM_RESOURCE_PATH="$MONDO_RISORSE${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

# Preparazione: eliminazione dei residui, potatura dei registri, verifica di
# risorse, file e allineamento dei binari ai sorgenti. Vedi scripts/preflight.sh
# per il guasto reale dietro ognuno dei controlli.
#
# Sostituisce una pulizia che uccideva i figli ma NON lo script di run: quello
# restava vivo e ne riavviava altri, ed e' cosi' che sono arrivate a convivere
# quattro simulazioni, una delle quali da 8 ore. Ora, se la macchina non si
# riesce a sgomberare, la run non parte affatto invece di misurare rumore.
# shellcheck source=preflight.sh
source "$SCRIPT_DIR/preflight.sh"
if ! preflight "$MONDO_SDF" "$DRONE_NUM"; then
  echo "Preparazione fallita: non avvio la simulazione."
  exit 1
fi

# Il demone ROS va riavviato dopo la pulizia della memoria condivisa, altrimenti
# conserva riferimenti a nodi che non esistono piu'.
# Tetto di 20 s (19 ottobre 2026): due volte "ros2 daemon stop" e' rimasto fermo per 6 e 46
# minuti, e la run con lui.
timeout 20 bash -c "source $ROS2_SETUP; ros2 daemon stop" >/dev/null 2>&1
timeout 20 bash -c "source $ROS2_SETUP; ros2 daemon start" >/dev/null 2>&1
sleep 1

# --- 1. PX4 SITL instances (the first one also brings up the Gazebo world) ---
echo "[1/6] Avvio di $DRONE_NUM istanze PX4 SITL + Gazebo (mondo: $PX4_GZ_WORLD)..."
# Ambiente di Gazebo per PX4: px4-rc.gzsim legge ../gz_env.sh dalla cartella
# dell'istanza. Quello scritto dalla build di PX4 contiene percorsi ASSOLUTI della
# macchina su cui e' stata fatta la build (e la cartella dei mondi di PX4): lo si
# riscrive a ogni run dai percorsi di QUESTA copia del progetto, con la cartella del
# mondo preparato. Cosi' il progetto si sposta o si copia senza toccare nulla.
#
# Con ODOM_SOURCE=livo il drone viene da una COPIA del modello nella cartella della run,
# con la telecamera a LIVO_FOTO_HZ (di serie 10) invece di 5: FAST-LIVO2 aggiorna la stima
# solo all'arrivo di un'immagine, e a 5 Hz la stima andava a PX4 a 5 Hz con 0.16 s di
# ritardo (FAST-LIO: 10 Hz, 0.08 s). Il modello di PX4 resta com'e', per le altre run.
MODELLI_PX4="$PX4_DIR/Tools/simulation/gz/models"
MODELLI_RUN="$MODELLI_PX4"
if [ "$ODOM_SOURCE" = "livo" ]; then
  MODELLI_RUN="$LOG_DIR/modelli"
  rm -rf "$MODELLI_RUN"; mkdir -p "$MODELLI_RUN"
  cp -rL "$MODELLI_PX4/x500_mid360" "$MODELLI_RUN/"   # -L: in PX4 e' un collegamento
  python3 - "$MODELLI_RUN/x500_mid360/model.sdf" "${LIVO_FOTO_HZ:-10}" <<'PY' || exit 1
import re, sys
f, hz = sys.argv[1], sys.argv[2]
s = open(f).read()
i = s.index('<sensor name="front_camera"')
j = s.index('</sensor>', i)
blocco, n = re.subn(r'<update_rate>[^<]*</update_rate>', f'<update_rate>{hz}</update_rate>', s[i:j], count=1)
if n != 1:
    sys.exit('telecamera frontale senza update_rate nel modello')
open(f, 'w').write(s[:i] + blocco + s[j:])
PY
  echo "    FAST-LIVO2: telecamera a ${LIVO_FOTO_HZ:-10} Hz (modello in $MODELLI_RUN)."
fi
cat > "$PX4_BUILD/rootfs/gz_env.sh" <<GZENV
#!/usr/bin/env bash
# SCRITTO DA scripts/run_multi_exploration.sh a ogni run (percorsi di questa copia).
export PX4_GZ_MODELS=$MODELLI_RUN
export PX4_GZ_WORLDS=$LOG_DIR/mondo
export PX4_GZ_PLUGINS=$PX4_BUILD/src/modules/simulation/gz_plugins
export PX4_GZ_SERVER_CONFIG=$PX4_DIR/src/modules/simulation/gz_bridge/server.config

export GZ_SIM_RESOURCE_PATH=\$GZ_SIM_RESOURCE_PATH:\$PX4_GZ_MODELS:$MODELLI_PX4:\$PX4_GZ_WORLDS
export GZ_SIM_SYSTEM_PLUGIN_PATH=\$GZ_SIM_SYSTEM_PLUGIN_PATH:\$PX4_GZ_PLUGINS
export GZ_SIM_SERVER_CONFIG_PATH=\$PX4_GZ_SERVER_CONFIG
GZENV
for d in $(seq 1 "$DRONE_NUM"); do
  inst=$((d - 1))
  pose="${SPAWN_POSES[$inst]}"
  : > "$LOG_DIR/px4_$inst.log"
  # Each instance needs its own working directory (dataman/parameter/log files),
  # and px4-rc.gzsim sources ../gz_env.sh relative to it - hence a subdirectory
  # of rootfs/, where the build puts that file. Same layout as PX4's own
  # Tools/simulation/sitl_multiple_run.sh.
  inst_dir="$PX4_BUILD/rootfs/instance_$inst"
  mkdir -p "$inst_dir"
  # PX4 SALVA I PARAMETRI SU FILE E LI RILEGGE AL RIAVVIO. Sembra un dettaglio
  # e invece rende le run non riproducibili: l'airframe 22000 imposta i
  # parametri del volo senza GPS dentro un "if", ma non ha un ramo "else" che
  # li ripristini, quindi una run con PX4_GPS_DENIED=0 EREDITA la
  # configurazione senza GPS della run precedente.
  # E' gia' successo: una prova di controllo che doveva volare COL GPS e'
  # partita con EKF2_GPS_CTRL=0 ereditato, il drone non si e' nemmeno armato e
  # la prova non misurava quello che credevo.
  # Cancellare il file qui e' piu' solido di un ramo "else": non serve
  # conoscere i valori predefiniti di ogni parametro, e ogni run riparte
  # esattamente dalla definizione dell'airframe.
  rm -f "$inst_dir"/parameters.bson "$inst_dir"/parameters_backup.bson
  (
    # Niente source di ROS2 qui: su Jazzy il setup mette in testa al PATH il
    # 'gz' ridotto di gz_tools_vendor (solo transport/msgs, senza 'gz sim'), e
    # px4-rc.gzsim concluderebbe "Gazebo gz sim not found". PX4 parla con ROS
    # tramite l'agente XRCE, l'ambiente ROS non gli serve.
    cd "$inst_dir" || exit 1
    # Librerie di PX4 e dei suoi plugin Gazebo (libOpticalFlow.so per il plugin del
    # flusso ottico): dalla build di QUESTA copia. I binari hanno scritto dentro il
    # percorso ASSOLUTO della build originale (RUNPATH), e da una copia del progetto
    # Gazebo caricava la libreria della cartella vecchia (visto in /proc/<pid>/maps);
    # LD_LIBRARY_PATH viene cercato prima del RUNPATH.
    export LD_LIBRARY_PATH="$PX4_BUILD/OpticalFlow/install/lib:$PX4_BUILD/external/Install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    export PX4_SYS_AUTOSTART=22000
    export PX4_SIM_MODEL=gz_x500_mid360
    export PX4_GZ_WORLD
    export PX4_GZ_MODEL_POSE="$pose"
    # Volo senza GPS: la legge l'airframe 22000 per spegnere il GNSS e
    # accendere la fusione della visione esterna. Va riesportata QUI dentro
    # insieme alle altre: la prima prova senza GPS e' volata col GPS acceso
    # proprio perche' mancava, e me ne sono accorto solo leggendo i parametri
    # nel registro di volo (EKF2_GPS_CTRL valeva 7 invece di 0).
    export PX4_GPS_DENIED="${PX4_GPS_DENIED:-0}"
    # Collaudo (SOLO_STIMA=1): l'airframe 22000 applica i limiti di collaudo_px4.params.
    export PX4_COLLAUDO="$SOLO_STIMA"
    # Instance 0 starts the gz server (and the GUI if asked); the later ones
    # find the world already running and just spawn their model into it.
    if [[ "$SHOW_GUI" =~ ^[Yy] ]] && [ "$inst" -eq 0 ]; then
      "$PX4_BUILD/bin/px4" -i "$inst" -d "$PX4_BUILD/etc" > "$LOG_DIR/px4_$inst.log" 2>&1
    else
      HEADLESS=1 "$PX4_BUILD/bin/px4" -i "$inst" -d "$PX4_BUILD/etc" \
        > "$LOG_DIR/px4_$inst.log" 2>&1
    fi
  ) &
  PIDS+=($!)

  # Wait for this instance before starting the next: instance 0 has to have the
  # world up before the others probe for it, and staggering the spawns keeps
  # them from racing on the same /world/<w>/create service.
  for i in $(seq 1 60); do
    grep -q "Startup script returned successfully" "$LOG_DIR/px4_$inst.log" 2>/dev/null && break
    sleep 1
  done
  if ! grep -q "Startup script returned successfully" "$LOG_DIR/px4_$inst.log" 2>/dev/null; then
    echo "ERRORE: l'istanza PX4 $inst non e' partita entro 60s. Controlla $LOG_DIR/px4_$inst.log"
    exit 1
  fi
  echo "    OK - istanza $inst pronta (modello x500_mid360_$inst, pose $pose)."
done

# --- 2. Micro XRCE-DDS Agent (one agent serves every instance) ---
echo "[2/6] Avvio Micro XRCE-DDS Agent..."
(MicroXRCEAgent udp4 -p 8888 > "$LOG_DIR/xrce.log" 2>&1) &
PIDS+=($!)
for i in $(seq 1 20); do
  # One session per PX4 instance.
  [ "$(grep -ci "session established" "$LOG_DIR/xrce.log" 2>/dev/null)" -ge "$DRONE_NUM" ] && break
  sleep 1
done
echo "    OK - $(grep -ci 'session established' "$LOG_DIR/xrce.log" 2>/dev/null) sessioni PX4 connesse."

# --- 3-5. The whole ROS2 side in one launch ---
echo "[3/6] Avvio bridge + stack di esplorazione + relay swarm (config: $MONDO_CONFIG)..."

# The launch file needs the same spawn poses PX4 was given: PX4's local position
# is relative to each vehicle's own spawn point, so odometry_bridge has to add
# the offset back to land every drone in one shared world frame. Reformat
# "x,y,z,roll,pitch,yaw" (PX4) into the "x,y,z,yaw;..." list the launch expects.
SPAWN_POSES_ARG=""
for d in $(seq 1 "$DRONE_NUM"); do
  IFS=',' read -r px py pz _proll _ppitch pyaw <<< "${SPAWN_POSES[$((d - 1))]}"
  SPAWN_POSES_ARG+="${px},${py},${pz},${pyaw:-0};"
done

EXPL_PID=""
launch_exploration_stack() {
  : > "$LOG_DIR/exploration.log"
  (
    source "$ROS2_SETUP"; source "$WS_SETUP"
    # FAST-LIVO2 (ODOM_SOURCE=livo) scrive qui i suoi file (tempi per fotogramma in
    # lio_timing.csv, stati in mat_pre.txt / mat_out.txt).
    export C2X_LIVO_DIR="$LOG_DIR/fast_livo2"
    export C2X_SOLO_STIMA="$SOLO_STIMA"
    ros2 launch exploration_manager multi_drone_exploration.launch.py \
      odom_source:="${ODOM_SOURCE:-px4}" \
      tempo_sim:="$([ "$TEMPO_SIM" = 1 ] && echo true || echo false)" \
      ${RAGGIO_RADIO:+raggio_radio:="$RAGGIO_RADIO"} \
      ${LINEA_VISTA:+linea_vista:="$([ "$LINEA_VISTA" = 1 ] && echo true || echo false)"} \
      drone_num:="$DRONE_NUM" config_name:="$MONDO_CONFIG" world:="$PX4_GZ_WORLD" \
      spawn_poses:="$SPAWN_POSES_ARG" initial_spin_rate:="$GIRO_INIZIALE" max_vel:="$VEL_MAX" \
      photo_completion:="$([ "$FASE_FOTO" = 1 ] && echo true || echo false)" \
      photo_yaw:="$([ "$IMBARDATA_FOTO" = 1 ] && echo true || echo false)" \
      visit_completion:="$([ "$VISITA" = 1 ] && echo true || echo false)" \
      > "$LOG_DIR/exploration.log" 2>&1
  ) &
  EXPL_PID=$!
  PIDS+=($EXPL_PID)
  [ "$SOLO_STIMA" = "1" ] && return 0   # nessun pianificatore da aspettare
  # Handshake esplicito: si aspetta che ogni drone DICHIARI di essere pronto.
  #
  # Prima si leggevano i log con grep, contando le occorrenze di "wait for
  # trigger" - che ogni drone ripete finche' non parte. Un solo drone che lo
  # stampava 62 volte soddisfaceva la soglia di 3, e le run partivano con un
  # drone mai avviato. Contare i droni distinti ha tolto quel difetto, ma
  # restava una verifica indiretta: il log dice che il nodo e' vivo, non che ha
  # mappa ed e' in grado di esplorare.
  # Ora ogni drone pubblica su exploration_ready le proprie precondizioni
  # (odometria, mappa, frontiere viste) e il coordinatore riferisce QUALE manca
  # e PERCHE'.
  # Sottoshell con l'ambiente ROS: questa parte dello script non lo ha
  # (le sorgenti vengono caricate solo nella sottoshell che lancia ros2), e
  # senza rclpy il coordinatore non parte nemmeno.
  if ! ( source "$ROS2_SETUP"; source "$WS_SETUP"; \
         python3 "$SCRIPT_DIR/wait_ready.py" --drones "$DRONE_NUM" \
                 --timeout "$READY_TIMEOUT_S" ); then
    echo "ERRORE: handshake di prontezza fallito. Controlla $LOG_DIR/exploration.log"
    exit 1
  fi
}

stop_exploration_stack() {
  [ -n "$EXPL_PID" ] && kill -9 "$EXPL_PID" 2>/dev/null
  pkill -9 -f "exploration_node" 2>/dev/null
  pkill -9 -f "mtsp_node" 2>/dev/null
  pkill -9 -f "plan_manage/traj_server" 2>/dev/null
  pkill -9 -f "c2x_px4_bridge" 2>/dev/null
  pkill -9 -f "parameter_bridge" 2>/dev/null
  pkill -9 -f "swarm_relay" 2>/dev/null
  sleep 3
}

# Registratore della mappa realistica, uno per drone: accumula le nuvole lidar
# gia' posate e i fotogrammi della telecamera, e salva tutto quando riceve
# SIGINT (al salvataggio delle mappe, piu' sotto).
# Parte PRIMA dello stack di esplorazione, non dopo il via: cosi' registra anche
# il decollo e il giro iniziale di 360 gradi, che e' proprio il momento in cui la
# telecamera frontale fotografa tutto l'intorno della partenza.
if [ "$MAPPA_REALISTICA" = "1" ]; then
  rm -rf "$LOG_DIR"/mappa_realistica_drone* "$LOG_DIR/mappa_realistica_sciame"
  # A priorita' bassa (nice 10): non serve al volo, e sul Jetson la CPU e' satura
  # (12% libera in volo su cubicle_office, Gazebo scendeva a RTF 0.87 mentre da
  # solo tiene 1.0). Cosi' cede CPU a Gazebo, PX4, FAST-LIO e pianificazione e si
  # prende quella che resta; per il suo ~35% di un core basta.
  # UN SOLO registratore per tutti i droni (sciame, ottobre 2026): la mappa esce gia'
  # fusa e la RAM non si triplica (vedi registra_mappa.py).
  ( source "$ROS2_SETUP"; source "$WS_SETUP"
    exec nice -n 10 python3 "$SCRIPT_DIR/registra_mappa.py" --world "$PX4_GZ_WORLD" \
      --drone "$(seq -s, 1 "$DRONE_NUM")" --out "$MAPPA_DIR" \
      $([ "$FOTO" = 1 ] && echo --foto) $([ "$TEMPO_SIM" = 1 ] && echo --tempo_sim) \
    ) > "$LOG_DIR/registra_mappa.log" 2>&1 &
  REGISTRATORI+=($!)
  echo "    Registrazione della mappa realistica avviata ($([ "$FOTO" = 1 ] && echo "telecamera + lidar" || echo "solo lidar, telecamera spenta")$([ "$DRONE_NUM" -gt 1 ] && echo ", mappa fusa di $DRONE_NUM droni"))."
fi

launch_exploration_stack
if [ "$SOLO_STIMA" = "1" ]; then
  echo "    Pronto per il collaudo: PX4 e FAST-LIO attivi, nessun pianificatore."
  echo "    (scripts/collaudo/collaudo.py --sim; questo script si chiude con SIGTERM o Ctrl+C)"
  wait "$EXPL_PID"
  exit 0
fi
echo "    OK - tutti i droni pronti al trigger."

# --- 6a. Warm-up ---
echo "[4/6] Riscaldamento (accumulo scansione iniziale, ${WARMUP_S}s)..."
w=0
while [ $w -lt $WARMUP_S ]; do
  echo "    ...${w}/${WARMUP_S}s"
  sleep 5
  w=$((w + 5))
done

send_trigger() {
  (
    source "$ROS2_SETUP"
    # /move_base_simple/goal is the one absolute topic in the stack, so this
    # single publication starts every drone. The goal pose itself is ignored by
    # the FSM (it triggers off each drone's own current position).
    # transient_local + reliable come la sottoscrizione dell'FSM, e pubblicatore vivo
    # 20 s (erano 6): con tre droni sulla CPU satura la scoperta DDS di questo processo
    # nuovo poteva non completarsi in tempo per tutti (sciame, ottobre 2026); cosi' un
    # drone che si accoppia in ritardo riceve comunque l'ultimo messaggio. Il ciclo qui
    # sotto lo interrompe appena tutti sono partiti.
    exec timeout 20 ros2 topic pub /move_base_simple/goal geometry_msgs/msg/PoseStamped \
      "{header: {frame_id: 'world'}, pose: {position: {x: 0.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}" \
      -r 2 --qos-durability transient_local --qos-reliability reliable >> "$LOG_DIR/trigger.log" 2>&1
  ) &
  local pid=$! k started finished
  for k in $(seq 1 10); do
    sleep 2
    started=$(grep -c "WAIT_TRIGGER to PLAN_TRAJ" "$LOG_DIR/exploration.log" 2>/dev/null)
    finished=$(grep -c "WAIT_TRIGGER to FINISH" "$LOG_DIR/exploration.log" 2>/dev/null)
    [ $((started + finished)) -ge "$DRONE_NUM" ] && break
  done
  kill "$pid" 2>/dev/null
  wait "$pid" 2>/dev/null
}

echo "[5/6] Trigger esplorazione..."
TRIGGER_OK=0
for attempt in $(seq 1 $MAX_TRIGGER_ATTEMPTS); do
  echo "Invio trigger (tentativo $attempt/$MAX_TRIGGER_ATTEMPTS)..."
  send_trigger
  sleep 1

  started=$(grep -c "WAIT_TRIGGER to PLAN_TRAJ" "$LOG_DIR/exploration.log" 2>/dev/null)
  finished=$(grep -c "WAIT_TRIGGER to FINISH" "$LOG_DIR/exploration.log" 2>/dev/null)
  # Serve che TUTTI abbiano lasciato WAIT_TRIGGER, non almeno uno: il trigger e'
  # un messaggio istantaneo, e un nodo che si iscrive dopo la pubblicazione non
  # lo ricevera' mai piu'. Uscendo al primo drone partito, uno che fosse ancora
  # indietro nell'avvio restava a terra per l'intera run - successo davvero: il
  # drone 1 e' rimasto in WAIT_TRIGGER per 682 s con zero cicli di pianificazione
  # mentre gli altri due esploravano.
  if [ $((started + finished)) -ge "$DRONE_NUM" ]; then
    echo "    OK - $started/$DRONE_NUM droni hanno iniziato a esplorare (${finished} senza frontiere)."
    TRIGGER_OK=1
    break
  fi

  if [ "$finished" -ge "$DRONE_NUM" ]; then
    if [ "$attempt" -ge "$MAX_TRIGGER_ATTEMPTS" ]; then
      echo "    ATTENZIONE: nessuna frontiera trovata dalla scansione iniziale dopo"
      echo "    $MAX_TRIGGER_ATTEMPTS tentativi. Le mappe saranno minime."
      break
    fi
    echo "    Nessuna frontiera alla prima scansione (FSM in FINISH, stato terminale)"
    echo "    - riavvio lo stack di esplorazione e riprovo..."
    stop_exploration_stack
    launch_exploration_stack
    continue
  fi

  echo "    Nessuna conferma nel log, riprovo l'invio del trigger..."
done

if [ "$TRIGGER_OK" -ne 1 ]; then
  echo "    ATTENZIONE: nessuna conferma di avvio esplorazione."
fi

# --- 6b. Wait for exploration to finish (or crash) ---
echo ""
echo "=== Esplorazione in corso... aspetto il completamento (timeout ${FINISH_TIMEOUT_S}s) ==="
ELAPSED=0
while [ $ELAPSED -lt $FINISH_TIMEOUT_S ]; do
  done_cnt=$(grep -c "has finished exploration" "$LOG_DIR/exploration.log" 2>/dev/null)
  if [ "$done_cnt" -ge "$DRONE_NUM" ]; then
    echo "Esplorazione completata da tutti i droni!"
    break
  fi
  if grep -q "CRASH DETECTED" "$LOG_DIR/exploration.log" 2>/dev/null; then
    echo "Almeno un drone si e' capovolto (crash fisico):"
    grep "CRASH DETECTED" "$LOG_DIR/exploration.log" | tail -3
    break
  fi
  # Volo finito senza conclusione dell'esplorazione: disarmo in volo, stima di
  # FAST-LIO divergente (in entrambi i casi e' gia' stato chiesto l'atterraggio),
  # oppure lo stack ROS spento da un nodo critico terminato (PX4 atterra da solo
  # per perdita del collegamento offboard). Aspettare il timeout non servirebbe.
  if grep -qE "VOLO INTERROTTO|DIVERGENZA FAST-LIO" "$LOG_DIR/exploration.log" 2>/dev/null; then
    echo "Volo interrotto prima della fine dell'esplorazione:"
    grep -E "VOLO INTERROTTO|DIVERGENZA FAST-LIO|DISARMATO IN VOLO" "$LOG_DIR/exploration.log" | tail -3
    break
  fi
  if [ -n "$EXPL_PID" ] && ! kill -0 "$EXPL_PID" 2>/dev/null; then
    echo "Lo stack ROS si e' fermato (nodo critico terminato): PX4 atterra per perdita del collegamento."
    grep -E "process has died|nodo critico" "$LOG_DIR/exploration.log" | tail -3
    break
  fi
  echo "[$(date +%H:%M:%S)] completati: $done_cnt/$DRONE_NUM  (RTF $(campiona_rtf || echo '?'))"
  print_progress
  sleep 15
  ELAPSED=$((ELAPSED + 15))
done
if [ $ELAPSED -ge $FINISH_TIMEOUT_S ]; then
  echo "Timeout raggiunto, procedo comunque a salvare le mappe allo stato attuale."
fi

# --- Seconda fase: il RIENTRO ---
#
# Il ciclo qui sopra esce appena tutti i droni hanno DICHIARATO conclusa
# l'esplorazione, ma quel messaggio precede la partenza per il rientro. Senza
# questa attesa lo script salvava le mappe e uccideva tutto mentre i droni erano
# appena partiti verso casa: misurato, 1 volo su 6 tornava davvero, gli altri
# venivano interrotti a meta' strada. Non e' che i droni non rientrassero - li
# si interrompeva.
# Le due fasi restano separate anche nel conteggio: il tempo di ESPLORAZIONE e'
# quello sopra e resta confrontabile con tutte le misure precedenti, il rientro
# si somma a parte.
# 450 s (era 120): fra la fine delle frontiere e il rientro c'e' ora la fase
# fotografica (fsm.photo_completion_time, fino a 300 s). Con 120 s due run su
# tre della campagna V26 venivano chiuse a meta' rientro, a 3 cm da casa.
RETURN_TIMEOUT_S="${RETURN_TIMEOUT_S:-450}"
# Sciame a tempo simulato: la fase di visita (fino a 300 s SIMULATI) e il rientro durano
# in parete ~N volte di piu' (vedi FINISH_TIMEOUT_S).
if [ "$DRONE_NUM" -gt 1 ] && [ "$TEMPO_SIM" = "1" ]; then
  RETURN_TIMEOUT_S="${RETURN_TIMEOUT_S_SCIAME:-$((RETURN_TIMEOUT_S * DRONE_NUM + 300))}"
fi
echo "=== Rientro alla partenza (timeout ${RETURN_TIMEOUT_S}s) ==="
R_ELAPSED=0
while [ $R_ELAPSED -lt $RETURN_TIMEOUT_S ]; do
  # Droni DISTINTI che hanno concluso il rientro (atterraggio chiesto a casa, fuori tempo o
  # gia' alla partenza). Contare le righe con "atterra" non bastava: il ponte ne scrive una
  # sua per lo stesso atterraggio ("[Casa] ... atterraggio verticale"), e con tre droni il
  # conteggio arrivava a 3 quando ne erano atterrati due (ottobre 2026).
  landed=$(grep -oE "\[Rientro\] Drone [0-9]+ (arrivato|tempo scaduto|gia' alla partenza)" \
             "$LOG_DIR/exploration.log" 2>/dev/null | grep -oE "Drone [0-9]+" | sort -u | wc -l)
  if [ "$landed" -ge "$DRONE_NUM" ]; then
    echo "    Tutti i droni sono rientrati e atterrati (${R_ELAPSED}s)."
    break
  fi
  echo "    rientrati: $landed/$DRONE_NUM (${R_ELAPSED}s, RTF $(campiona_rtf || echo '?'))"
  sleep 10
  R_ELAPSED=$((R_ELAPSED + 10))
done
[ $R_ELAPSED -ge $RETURN_TIMEOUT_S ] && echo "    Rientro incompleto allo scadere del tempo."


# --- Save each drone's map ---
echo ""
echo "=== Salvataggio mappe ==="
# Prima si ferma il registratore della mappa realistica, mentre lo stack e'
# ancora vivo: alla ricezione di SIGINT salva voxel e fotogrammi.
for pid in "${REGISTRATORI[@]}"; do kill -INT "$pid" 2>/dev/null; done
# Fino a 300 s (erano 60): con 15-16 milioni di voxel il salvataggio sul Jetson ne
# richiedeva di piu', e la mappa 3D partiva su dati scritti a meta' (ottobre 2026).
for pid in "${REGISTRATORI[@]}"; do
  for i in $(seq 1 300); do kill -0 "$pid" 2>/dev/null || break; sleep 1; done
  kill -0 "$pid" 2>/dev/null && echo "    ATTENZIONE: il registratore della mappa ($pid) non ha finito di salvare in 300 s"
done
PCD_PATHS=()
for d in $(seq 1 "$DRONE_NUM"); do
  # Tetto di 60 s: se il nodo di esplorazione e' morto (crash) il servizio non arriva mai e
  # la chiamata aspettava per sempre ("waiting for service to become available...").
  timeout 60 bash -c "source $ROS2_SETUP; source $WS_SETUP; ros2 service call /drone_$d/sdf_map/save_map std_srvs/srv/Trigger" \
    > "$LOG_DIR/save_map_$d.log" 2>&1
  p="/tmp/c2x_explored_map_drone$d.pcd"
  if [ -f "$p" ]; then
    PCD_PATHS+=("$p")
    echo "    drone $d -> $p"
  else
    echo "    ATTENZIONE: mappa del drone $d non trovata in $p"
  fi
done

if [ "${#PCD_PATHS[@]}" -eq 0 ]; then
  echo "ERRORE: nessuna mappa salvata."
  exit 1
fi

# --- Mappa realistica: superficie 3D colorata ---
if [ "$MAPPA_REALISTICA" = "1" ]; then
  echo "=== Mappa realistica (superficie colorata dalla telecamera$([ "$DRONE_NUM" -gt 1 ] && echo ", fusa da $DRONE_NUM droni")) ==="
  for dir in "$MAPPA_DIR"; do
    if [ -f "$dir/punti.npz" ]; then
      # Senza GPS si tengono anche le celle viste una sola volta (copertura delle
      # pareti visibili 99.9-100%, precisione 100%). Col GPS no: l'imbardata
      # dalla bussola sbaglia di ~2 gradi e quei punti sono proprio i doppioni
      # delle pareti (precisione 3D 88-92% contro 93-97%, campagna V16).
      if [ "${ODOM_SOURCE:-px4}" = "lio" ] || [ "${ODOM_SOURCE:-px4}" = "livo" ]; then MIN_CONTEGGIO=1; else MIN_CONTEGGIO=2; fi
      # Uscita completa in $LOG_DIR/ricostruisci.log; qui solo le righe [ricostruisci] e,
      # se fallisce, la coda dell'errore (prima un errore non si vedeva affatto).
      if ! PATH=/usr/bin:$PATH /usr/bin/python3 "$SCRIPT_DIR/ricostruisci_mappa.py" "$dir" \
        --min_conteggio "$MIN_CONTEGGIO" --max_fotogrammi $((1000 * DRONE_NUM)) \
        > "$LOG_DIR/ricostruisci.log" 2>&1; then
        grep "^\[ricostruisci\]" "$LOG_DIR/ricostruisci.log"
        echo "    ATTENZIONE: mappa 3D non riuscita; ultime righe di $LOG_DIR/ricostruisci.log:"
        tail -5 "$LOG_DIR/ricostruisci.log" | sed 's/^/      /'
      else
        grep "^\[ricostruisci\]" "$LOG_DIR/ricostruisci.log"
      fi
      # Finestra separata dalla simulazione (setsid): resta aperta anche dopo il
      # Ctrl+C che chiude tutto il resto, cosi' la mappa si guarda con calma.
      if [ "$MOSTRA_MAPPA" = "1" ] && [ -n "$DISPLAY" ] && [ -f "$dir/mappa_realistica.ply" ]; then
        echo "    Apro la mappa 3D in una finestra (ruota col mouse, C = soffitto, Q = chiudi)."
        setsid /usr/bin/python3 "$SCRIPT_DIR/mostra_mappa.py" "$dir" \
          > "$LOG_DIR/mostra_mappa.log" 2>&1 < /dev/null &
      fi
    else
      echo "    ATTENZIONE: dati della mappa realistica non trovati in $dir"
    fi
  done
fi
echo "=== Archivio della run ==="
archivia_run
echo "Premi Ctrl+C in questo terminale per chiudere tutta la simulazione."
wait
