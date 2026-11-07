#!/bin/bash
# CAMPAGNA DI CONFRONTO DRONE SINGOLO / SCIAME (ottobre 2026).
#
# Per ogni mondo e per ogni modo (con e senza fase di visita) esegue una missione con un
# drone e una con DRONI droni, con lo stesso codice e le stesse impostazioni, poi scrive la
# pagella comparativa (pagella_sciame.py) in risultati_prova/CAMPAGNA_SCIAME_<data>/.
# Le run restano archiviate come al solito in risultati_prova/ (una cartella per run).
#
# Uso (dalla cartella ros2_ws, PATH senza /opt/ros):
#   ./scripts/campagna_sciame.sh [mondo ...]
# Ambiente: DRONI (di serie 3), MODI = valori di VISITA da provare (di serie "1 0"),
# ODOM_SOURCE (di serie lio), PF_MIN_RAM_GB (di serie 3). Ogni altra variabile dello script
# di run (VEL_MAX, ...) passa invariata a tutte le run.
# RUN="mondo:droni:visita ..." esegue esattamente quelle missioni, in quell'ordine, al posto
# del giro completo (per esempio per rifare solo le run dello sciame dopo una correzione).
# Si puo' lanciare staccato dal terminale: setsid nohup ./scripts/campagna_sciame.sh ... &
#
# Durata indicativa sul Jetson: drone singolo 5-15 min a run, sciame di tre droni 15-40 min
# (la simulazione gira a ~0.3 volte il tempo reale; i tempi delle pagelle sono SIMULATI).

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
RISULTATI="$(cd "$WS_DIR/.." && pwd)/risultati_prova"
MONDI=("$@")
[ ${#MONDI[@]} -eq 0 ] && MONDI=(office_small empty_room open_field parco cubicle_office)
DRONI="${DRONI:-3}"
MODI="${MODI:-1 0}"
export ODOM_SOURCE="${ODOM_SOURCE:-lio}"
export PF_MIN_RAM_GB="${PF_MIN_RAM_GB:-3}"
export MOSTRA_MAPPA=0
CAMPAGNA="$RISULTATI/CAMPAGNA_SCIAME_$(date +%Y-%m-%d_%H%M)"
mkdir -p "$CAMPAGNA"
ELENCO="$CAMPAGNA/run.txt"
: > "$ELENCO"

# Una missione: risponde N alla GUI e chiude la simulazione quando lo script di run arriva
# alla fine ("Premi Ctrl+C"). Stampa la cartella archiviata.
una_run() {
  local mondo=$1 n=$2 visita=$3 prima dopo pid
  prima=$(ls -d "$RISULTATI"/2*/ 2>/dev/null | sort | tail -1)
  : > /tmp/c2x_run_multi/run.log 2>/dev/null
  (cd "$WS_DIR" && echo N | env VISITA="$visita" ./scripts/run_multi_exploration.sh "$mondo" "$n") \
    > "$CAMPAGNA/terminale_${mondo}_${n}droni_visita${visita}.log" 2>&1 &
  pid=$!
  while kill -0 $pid 2>/dev/null; do
    if grep -q "Premi Ctrl+C" /tmp/c2x_run_multi/run.log 2>/dev/null; then
      sleep 2; kill -TERM $pid 2>/dev/null
      for _ in $(seq 1 20); do kill -0 $pid 2>/dev/null || break; sleep 2; done
      kill -9 $pid 2>/dev/null
      break
    fi
    sleep 5
  done
  wait $pid 2>/dev/null
  dopo=$(ls -d "$RISULTATI"/2*/ 2>/dev/null | sort | tail -1)
  [ "$dopo" != "$prima" ] && echo "${dopo%/}"
}

MISSIONI=()
if [ -n "${RUN:-}" ]; then
  read -r -a MISSIONI <<< "$RUN"
else
  for mondo in "${MONDI[@]}"; do
    for visita in $MODI; do
      for n in 1 "$DRONI"; do MISSIONI+=("$mondo:$n:$visita"); done
    done
  done
fi

for m in "${MISSIONI[@]}"; do
  IFS=: read -r mondo n visita <<< "$m"
  echo "[$(date +%H:%M)] $mondo, $n droni, VISITA=$visita" | tee -a "$CAMPAGNA/avanzamento.txt"
  cartella=$(una_run "$mondo" "$n" "$visita")
  echo "$mondo $n $visita ${cartella:-NESSUNA}" >> "$ELENCO"
done

echo "[$(date +%H:%M)] pagella" | tee -a "$CAMPAGNA/avanzamento.txt"
cartelle=$(awk '$4 != "NESSUNA" {print $4}' "$ELENCO")
# shellcheck disable=SC2086
nice -n 5 /usr/bin/python3 "$SCRIPT_DIR/pagella_sciame.py" $cartelle > "$CAMPAGNA/pagella.txt" 2>&1
echo "[$(date +%H:%M)] fine: $CAMPAGNA/pagella.txt" | tee -a "$CAMPAGNA/avanzamento.txt"
