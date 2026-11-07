#!/bin/bash
# Preparazione e verifiche prima di lanciare una simulazione.
#
# Nasce da guasti reali, non da prudenza generica. Ognuna delle funzioni qui
# sotto corrisponde a un problema che ha gia' fatto perdere ore:
#
#   * quattro simulazioni contemporanee, di cui una rimasta viva 8 ore e 15
#     minuti da un esperimento della mattina. Condividendo dominio DDS, porte e
#     memoria condivisa producono droni che non ricevono mai l'odometria, stalli
#     all'avvio, e campagne con configurazione IDENTICA che danno esiti opposti
#     (5 run complete su 8 contro 1 su 8). Sintomi che sembrano difetti del
#     codice e non lo sono.
#   * 51 GB di registri di volo accumulati in dodici giorni, 1093 file, mai
#     rimossi: scrittura continua su una macchina con 14 GB di RAM.
#   * binari vecchi misurati come se fossero nuovi, tre volte: una libreria
#     statica ricompilata senza rilinkare chi la usa, e una build fatta mentre
#     una campagna era in corso.
#
# Uso:  source preflight.sh; preflight <mondo.sdf | nome di un mondo di PX4> <drone_num>
# Esce diverso da zero se qualcosa non torna: meglio non partire che misurare
# rumore.

# Percorsi relativi alla posizione di questo file (viene incluso con source,
# quindi BASH_SOURCE e non $0).
PF_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PF_WS="${PF_WS:-$(cd "$PF_SCRIPT_DIR/.." && pwd)}"
PF_PX4_DIR="${PF_PX4_DIR:-$(cd "$PF_WS/.." && pwd)/PX4-Autopilot}"
PF_PX4_ROOT="${PF_PX4_ROOT:-$PF_PX4_DIR/build/px4_sitl_default}"
PF_LOG_KEEP_DAYS="${PF_LOG_KEEP_DAYS:-1}"
PF_MIN_RAM_GB="${PF_MIN_RAM_GB:-4}"
PF_MIN_DISK_GB="${PF_MIN_DISK_GB:-10}"

pf_say()  { echo "    [preflight] $*"; }
pf_fail() { echo "    [preflight] ERRORE: $*" >&2; return 1; }

# --- 1. Nessuna simulazione precedente deve sopravvivere -------------------
# Lo SCRIPT di run va ucciso per primo: eliminarne prima i figli lo lascia vivo
# e lui ne riavvia o ne aspetta altri.
pf_kill_stale() {
  # Lo script di run si riconosce dalla shell che lo ESEGUE ("bash <percorso>
  # /run_multi_exploration.sh"), non dal nome dentro una riga di comando
  # qualsiasi: cosi' si eliminava anche la shell di chi stava lanciando la run.
  # Il confronto e' su "pid args", da cui il prefisso numerico.
  local pattern="^ *[0-9]+ (/usr)?(/bin/)?bash [^ ]*run_multi_exploration\.sh|bin/px4|gz sim|MicroXRCE|exploration_node"
  pattern="$pattern|mtsp_node|c2x_px4_bridge|parameter_bridge|swarm_relay"
  # fastlio_mapping MANCAVA: un FAST-LIO sopravvissuto a una run precedente
  # continuava a pubblicare una seconda stima sullo stesso topic della run nuova.
  pattern="$pattern|plan_manage/traj_server|ros_gz_bridge|fastlio_mapping"

  # Noi stessi e i nostri antenati vanno esclusi, SEMPRE.
  #
  # Senza questa esclusione la funzione si suicidava: il pattern contiene
  # "run_multi_exploration.sh" - indispensabile per eliminare gli script rimasti
  # da sessioni precedenti - e lo script che esegue preflight corrisponde a quel
  # pattern come chiunque altro. Si vedeva solo un "Killed" e la simulazione non
  # partiva mai.
  local mine p guard=0
  p=$$
  mine=""
  while [ -n "$p" ] && [ "$p" -gt 1 ] && [ "$guard" -lt 20 ]; do
    mine="${mine}${p}|"
    p=$(ps -o ppid= -p "$p" 2>/dev/null | tr -d ' ')
    guard=$((guard + 1))
  done
  mine="${mine%|}"
  [ -z "$mine" ] && mine="@nessuno@"

  pf_targets() {
    ps -eo pid,args | grep -E "$pattern" | grep -v grep | awk '{print $1}' \
      | grep -vxE "$mine"
  }

  local n
  n=$(pf_targets | wc -l)
  [ "$n" -eq 0 ] && { pf_say "nessun processo residuo"; return 0; }

  pf_say "$n processi residui da una sessione precedente - li rimuovo"
  pf_targets | xargs -r kill -9 2>/dev/null
  sleep 4
  # Secondo giro: i figli orfani possono comparire dopo la morte del padre, e
  # se ne sono visti sopravvivere 13 a un primo passaggio.
  pf_targets | xargs -r kill -9 2>/dev/null
  sleep 3

  n=$(ps -eo args | grep -E "bin/px4|gz sim|exploration_node" | grep -v grep | wc -l)
  [ "$n" -gt 0 ] && return $(pf_fail "$n processi non eliminabili; una nuova run misurerebbe rumore")
  pf_say "processi residui eliminati"
  return 0
}

# --- 2. Memoria condivisa DDS ---------------------------------------------
# Va ripulita SOLO a processi morti: cancellare i segmenti sotto nodi ancora
# vivi ne corrompe lo stato di discovery.
pf_clean_shm() {
  local n
  n=$(ls /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null | wc -l)
  [ "$n" -eq 0 ] && return 0
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
  pf_say "$n segmenti di memoria condivisa DDS rimossi"
  return 0
}

# --- 3. Spazio: registri di volo ------------------------------------------
# I .ulg sono l'unica fonte di assetto, riferimento e velocita' campionati -
# indispensabili per capire un ribaltamento - ma solo quelli recenti servono.
# Le copie che contano vengono archiviate dalla campagna in /tmp/c2x_reliability
# e NON sono toccate qui.
pf_prune_logs() {
  [ -d "$PF_PX4_ROOT/rootfs" ] || return 0
  local prima dopo n
  prima=$(du -sm "$PF_PX4_ROOT/rootfs" 2>/dev/null | cut -f1)
  [ -z "$prima" ] && return 0
  n=$(find "$PF_PX4_ROOT/rootfs" -name "*.ulg" -mtime +"$PF_LOG_KEEP_DAYS" 2>/dev/null | wc -l)
  if [ "$n" -gt 0 ]; then
    find "$PF_PX4_ROOT/rootfs" -name "*.ulg" -mtime +"$PF_LOG_KEEP_DAYS" -delete 2>/dev/null
    find "$PF_PX4_ROOT/rootfs" -type d -empty -delete 2>/dev/null
    dopo=$(du -sm "$PF_PX4_ROOT/rootfs" 2>/dev/null | cut -f1)
    pf_say "registri di volo: $n file rimossi, ${prima} MB -> ${dopo} MB"
  else
    pf_say "registri di volo: ${prima} MB, niente da rimuovere"
  fi
  return 0
}

# --- 4. Risorse ------------------------------------------------------------
pf_check_resources() {
  local ram disk
  ram=$(free -g | awk 'NR==2{print $7}')          # "available", non "free"
  disk=$(df -BG --output=avail "$PF_WS" 2>/dev/null | tail -1 | tr -dc '0-9')
  [ -z "$ram" ] && ram=0
  [ -z "$disk" ] && disk=0
  pf_say "risorse: ${ram} GB RAM disponibili, ${disk} GB su disco"
  [ "$ram" -lt "$PF_MIN_RAM_GB" ] && \
    return $(pf_fail "solo ${ram} GB di RAM disponibili (minimo ${PF_MIN_RAM_GB}); tre PX4 piu' Gazebo non ci stanno")
  [ "$disk" -lt "$PF_MIN_DISK_GB" ] && \
    return $(pf_fail "solo ${disk} GB liberi su disco (minimo ${PF_MIN_DISK_GB})")
  return 0
}

# --- 5. Eseguibili e file di scena ----------------------------------------
pf_check_files() {
  local scenario="$1"
  [ -x "$PF_PX4_ROOT/bin/px4" ] || return $(pf_fail "px4 non trovato in $PF_PX4_ROOT/bin")
  command -v MicroXRCEAgent >/dev/null || return $(pf_fail "MicroXRCEAgent non nel PATH")
  command -v gz >/dev/null || return $(pf_fail "gz non nel PATH")
  [ -f "$PF_WS/install/setup.bash" ] || return $(pf_fail "workspace non compilato: manca install/setup.bash")

  # Il mondo: un file .sdf (quello preparato da prepara_mondo.py) o, per chi chiama
  # ancora con il solo nome, uno dei mondi di PX4.
  local world="$scenario"
  [ -f "$world" ] || world="$PF_PX4_DIR/Tools/simulation/gz/worlds/${scenario}.sdf"
  [ -f "$world" ] || return $(pf_fail "mondo Gazebo mancante: $world")
  pf_say "eseguibili e mondo '$(basename "$world" .sdf)' presenti"
  return 0
}

# --- 5b. Collegamenti dell'install relativi --------------------------------
# colcon --symlink-install li crea assoluti: spostata o copiata la cartella del
# progetto, lo stack userebbe i file della copia vecchia (o non partirebbe). Vedi
# percorsi_relativi.py; una build li rifa' assoluti, quindi si controlla sempre.
pf_relocate_install() {
  if find "$PF_WS/install" -type l -lname '/*/ros2_ws/*' -print -quit 2>/dev/null | grep -q .; then
    pf_say "$(/usr/bin/python3 "$PF_SCRIPT_DIR/percorsi_relativi.py" "$PF_WS")"
  fi
  local rotti
  rotti=$(find "$PF_WS/install" -xtype l 2>/dev/null | wc -l)
  [ "$rotti" -gt 0 ] && return $(pf_fail "$rotti collegamenti rotti in install/: ricompila il workspace")
  return 0
}

# --- 6. I binari sono piu' recenti dei sorgenti? --------------------------
# Il guasto piu' insidioso di tutti: si misura codice vecchio credendolo nuovo.
# E' successo tre volte - una libreria statica ricompilata senza rilinkare chi
# la usa, una build lanciata mentre una campagna era in corso, e una mistura di
# binari strumentati e non. Nessuno dei tre da' errore: la run parte e i numeri
# sembrano validi.
pf_check_fresh() {
  local newest_src newest_bin
  newest_src=$(find "$PF_WS/src" \( -name "*.cpp" -o -name "*.h" -o -name "*.hpp" -o -name "*.yaml" \) \
               -newer "$PF_WS/install/setup.bash" 2>/dev/null | head -5)
  if [ -n "$newest_src" ]; then
    pf_say "ATTENZIONE: sorgenti piu' recenti dell'ultima compilazione:"
    echo "$newest_src" | sed 's|^|        |'
    pf_say "la run misurerebbe il codice PRECEDENTE. Ricompila, oppure imposta PF_ALLOW_STALE=1."
    [ "${PF_ALLOW_STALE:-0}" = "1" ] || return 1
  else
    pf_say "binari allineati ai sorgenti"
  fi
  return 0
}

# --- Sequenza completa -----------------------------------------------------
preflight() {
  local scenario="${1:-empty_room}"
  local drones="${2:-3}"
  echo "[0/6] Preparazione (mondo: $(basename "$scenario" .sdf), droni: $drones)..."
  pf_kill_stale     || return 1
  pf_clean_shm      || return 1
  pf_prune_logs     || return 1
  pf_check_resources|| return 1
  pf_check_files "$scenario" || return 1
  pf_relocate_install || return 1
  pf_check_fresh    || return 1
  pf_say "tutti i controlli superati"
  return 0
}
