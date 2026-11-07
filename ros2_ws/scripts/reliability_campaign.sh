#!/bin/bash
# Campagna di affidabilità: ripete la run a 3 droni N volte e riassume gli
# indicatori di ognuna, per poter confrontare configurazioni diverse su numeri
# invece che su impressioni.
#
# Uso: ./reliability_campaign.sh [N] [tag] [max_run_s] [scenario]
#   N          numero di run (default 5)
#   tag        etichetta che compare nel riepilogo (default "run")
#   scenario   empty_room | cubicle_office (default empty_room)
#   max_run_s  tetto in secondi per ogni run; 0 = aspetta il completamento
#             (default 0). Con un tetto la campagna dura un tempo prevedibile
#             invece di essere dominata dalla coda lunga: le run vanno da ~100s
#             a ~500s, e quelle lunghe non aggiungono informazione sui difetti
#             che stiamo cercando (impallamenti e crash si manifestano presto).
#
# Risultati in $OUT_DIR/reliability.txt; il log completo di ogni run viene
# conservato come rel_expl_<n>.log per poterci tornare sopra (es. per leggere
# uno stack trace).

N="${1:-5}"
TAG="${2:-run}"
MAX_RUN_S="${3:-0}"
SCENARIO="${4:-empty_room}"
# Numero di droni: parametrizzato per poter confrontare 2 contro 3 a parita' di
# tutto il resto. Era fisso a 3 in due punti - il lancio e il denominatore di
# "finiti" - e cambiarne uno solo avrebbe prodotto conteggi falsi.
DRONI="${5:-3}"
# Percorsi relativi alla posizione di questo script.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_SCRIPT=$SCRIPT_DIR/run_multi_exploration.sh
LOG_DIR=/tmp/c2x_run_multi
PX4_ROOT=$(cd "$SCRIPT_DIR/../.." && pwd)/PX4-Autopilot/build/px4_sitl_default/rootfs
OUT_DIR=/tmp/c2x_reliability
OUT="$OUT_DIR/reliability.txt"
# Copia per-campagna: reliability.txt viene azzerato a ogni avvio, e senza
# questa il riepilogo della campagna precedente sparisce (mi e' gia' successo).
OUT_TAGGED="$OUT_DIR/reliability_$TAG.txt"
# Nessuna finestra della mappa 3D durante le campagne: si accumulerebbero.
export MOSTRA_MAPPA=0

mkdir -p "$OUT_DIR"
: > "$OUT"

kill_all() {
  # Prima lo SCRIPT di run, poi i suoi processi: ucciderli in ordine inverso
  # lascia lo script vivo, che ne riavvia o ne aspetta altri.
  # Solo i processi che ESEGUONO lo script ("bash <percorso>/run_multi_...sh"),
  # non qualunque riga di comando che ne contenga il nome: con -f e il solo
  # nome si uccideva anche la shell da cui la campagna era stata lanciata.
  pkill -9 -f "^(/usr)?(/bin/)?bash [^ ]*run_multi_exploration\.sh"
  pkill -9 -f "bin/px4"; pkill -9 -f "gz sim"; pkill -9 -f MicroXRCE
  pkill -9 -f exploration_node; pkill -9 -f mtsp_node; pkill -9 -f c2x_px4_bridge
  pkill -9 -f parameter_bridge; pkill -9 -f swarm_relay; pkill -9 -f "plan_manage/traj_server"
  pkill -9 -f "ros_gz_bridge"
  pkill -9 -f "fastlio_mapping"
  pkill -9 -f "scripts/registra_mappa\.py"
}

# Nessuna simulazione deve sopravvivere all'inizio di una campagna.
#
# Trovate quattro run simultanee, di cui una rimasta in piedi 8 ore e 15 minuti
# da un esperimento della mattina. Simulazioni concorrenti condividono dominio
# DDS, porte e memoria condivisa: producono droni che non ricevono mai
# l'odometria, stalli, e campagne con configurazione IDENTICA che danno esiti
# opposti (CONF 5 run complete su 8, REV 1 su 8). Senza questo controllo ogni
# misura successiva e' sospetta.
guard_clean_start() {
  # ALTRE CAMPAGNE: vanno eliminate per prime, e non bastava eliminare le
  # simulazioni. Lanciandone una mentre un'altra girava, la protezione uccideva
  # le simulazioni della prima ma non il suo ciclo, che ne avviava subito di
  # nuove: le due campagne si sono alternate uccidendosi a vicenda, scrivendo
  # sullo stesso file di risultati. E' successo fra HOME e LAND, e i numeri di
  # entrambe sono da buttare.
  # Si escludono SE' STESSI E TUTTI GLI ANTENATI, non solo il proprio pid.
  #
  # Lanciando la campagna in primo piano, il processo che la ospita ha
  # "reliability_campaign.sh" nella propria riga di comando: la guardia lo
  # scambiava per una campagna rivale e uccideva il proprio genitore,
  # portandosi dietro la run. Con l'avvio in sottofondo non si vedeva, perche'
  # il processo ospite usciva subito - il difetto era latente e si e'
  # manifestato solo quando ho cambiato modo di lanciarla.
  # E' lo stesso difetto gia' corretto in preflight.sh, dove la protezione
  # analoga si autoeliminava.
  # Si escludono anche i DISCENDENTI, non solo gli antenati: il ciclo delle run
  # crea sottoshell che ereditano la riga di comando dello script, e la guardia
  # ne ha ucciso uno insieme alla run che stava eseguendo (campagna V6, run 1
  # persa cosi'). Il modo semplice e robusto e' escludere l'intero gruppo di
  # processi, che con setsid coincide con "questa campagna e i suoi figli".
  local mio_gruppo
  mio_gruppo=$(ps -o pgid= -p $$ 2>/dev/null | tr -d ' ')
  local antenati="" pp=$$
  while [ "$pp" -gt 1 ]; do
    antenati="$antenati $pp"
    pp=$(ps -o ppid= -p "$pp" 2>/dev/null | tr -d ' ')
    [ -z "$pp" ] && break
  done
  local altre
  # Solo le shell che eseguono lo script (secondo campo = percorso dello
  # script), non chi lo nomina in una riga di comando - per esempio la shell
  # che ha appena lanciato questa campagna.
  altre=$(ps -eo pid,args | grep -E "^ *[0-9]+ (/usr)?(/bin/)?bash [^ -][^ ]*reliability_campaign\.sh( |$)" \
          | awk -v esclusi=" $antenati " '{ if (index(esclusi, " " $1 " ") == 0) print $1 }')
  if [ -n "$mio_gruppo" ]; then
    altre=$(for pid in $altre; do
              g=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')
              # g vuoto = processo gia' uscito (tipicamente la sottoshell di
              # questo stesso ps): non e' un'altra campagna.
              [ -n "$g" ] && [ "$g" != "$mio_gruppo" ] && echo "$pid"
            done)
  fi
  if [ -n "$altre" ]; then
    echo "ATTENZIONE: altre campagne in corso (PID $(echo $altre | tr '\n' ' ')) - le interrompo"
    echo "$altre" | xargs -r kill -9 2>/dev/null
    sleep 3
  fi

  local residui
  residui=$(ps -eo args | grep -E "bin/px4|gz sim|exploration_node|^(/usr)?(/bin/)?bash [^ ]*run_multi_exploration" \
            | grep -v grep | wc -l)
  if [ "$residui" -gt 0 ]; then
    echo "ATTENZIONE: $residui processi di simulazione gia' attivi - li rimuovo prima di partire"
    kill_all
    sleep 5
    ps -eo pid,args | grep -E "bin/px4|gz sim|exploration_node" | grep -v grep \
      | awk '{print $1}' | xargs -r kill -9 2>/dev/null
    sleep 3
  fi
  residui=$(ps -eo args | grep -E "bin/px4|gz sim|exploration_node" | grep -v grep | wc -l)
  if [ "$residui" -gt 0 ]; then
    echo "ERRORE: $residui processi non eliminabili, la campagna misurerebbe rumore. Interrompo."
    exit 1
  fi
}
guard_clean_start

# I RESIDUI DELLA CAMPAGNA PRECEDENTE VANNO VIA PRIMA DI COMINCIARE.
#
# rel_ulog_* e rel_expl_* restavano nella cartella condivisa fra una campagna e
# l'altra, e una campagna con MENO run o MENO droni ereditava i file di quella
# prima: un confronto 2 contro 3 droni si e' ritrovato 12 registri di volo dove
# ne erano attesi 8, con dentro un terzo drone che non era mai decollato.
# Analizzare quella cartella "a fine campagna" sembra sicuro e non lo e' - e'
# la stessa contaminazione in cui sono gia' inciampato aggregando log di
# campagne diverse.
rm -f "$OUT_DIR"/rel_ulog_*.ulg "$OUT_DIR"/rel_expl_*.log "$OUT_DIR"/rel_run_*.log "$OUT_DIR"/rel_map_*.pcd

for run in $(seq 1 "$N"); do
  echo "=== $TAG run $run/$N (scenario $SCENARIO) ==="
  # Rimuovi il log della run precedente PRIMA di partire: senza questo, una run
  # che non si avvia lascia in piedi il vecchio exploration.log e il blocco di
  # analisi lo ricopia e lo conteggia una seconda volta (è successo in due
  # campagne, che risultavano avere due run con numeri identici).
  rm -f "$LOG_DIR/exploration.log"
  # Mappe della run precedente: se questa non riesce a salvarle, non devono
  # essere scambiate per le sue.
  rm -f /tmp/c2x_explored_map_drone*.pcd

  # setsid: ogni run nel proprio process group, così la pulizia fra una run e
  # l'altra non può colpire i processi della run successiva. ARCHIVIA=0: la
  # campagna archivia da se' in $OUT_DIR, non serve la copia in risultati_prova/.
  printf 'n\n' | ARCHIVIA=0 setsid bash "$RUN_SCRIPT" "$SCENARIO" "$DRONI" > "$OUT_DIR/rel_run_$run.log" 2>&1 &
  script_pid=$!
  sleep 2
  pgid=$(ps -o pgid= -p "$script_pid" 2>/dev/null | tr -d ' ')

  waited=0
  limit=2000
  [ "$MAX_RUN_S" -gt 0 ] && limit=$MAX_RUN_S
  stall=0
  last_size=0
  while [ "$waited" -lt "$limit" ]; do
    grep -q "Salvataggio mappe" "$OUT_DIR/rel_run_$run.log" 2>/dev/null && break
    # Rilevatore di stallo: se il log dell'esplorazione non cresce per STALL_S
    # la run e' bloccata e continuare e' solo tempo perso. Succede: uno stack
    # fermo per minuti mentre il tetto della campagna scorre fino in fondo.
    cur_size=$(stat -c %s "$LOG_DIR/exploration.log" 2>/dev/null || echo 0)
    if [ "$cur_size" -eq "$last_size" ] && [ "$cur_size" -gt 0 ]; then
      stall=$((stall + 10))
      if [ "$stall" -ge "${STALL_S:-90}" ]; then
        echo "$TAG $run: STALLO rilevato (log fermo da ${stall}s) - run interrotta" >> "$OUT"
        break
      fi
    else
      stall=0
    fi
    last_size=$cur_size
    # Uscita anticipata: se un nodo e' morto o un drone si e' ribaltato non c'e'
    # niente da guadagnare ad aspettare il resto della run.
    grep -qE "Segmentation fault|CRASH DETECTED|VOLO INTERROTTO|DIVERGENZA FAST-LIO" "$LOG_DIR/exploration.log" 2>/dev/null && break
    sleep 10
    waited=$((waited + 10))
  done
  # Fine VERA della run: lo script stampa "Premi Ctrl+C" solo dopo aver salvato
  # le mappe e ricostruito la mappa realistica (superficie colorata), che puo'
  # richiedere qualche minuto. Prima si aspettavano 10 s fissi, e la
  # ricostruzione veniva uccisa a meta'.
  if grep -q "Salvataggio mappe" "$OUT_DIR/rel_run_$run.log" 2>/dev/null; then
    for i in $(seq 1 60); do
      grep -q "Premi Ctrl+C" "$OUT_DIR/rel_run_$run.log" 2>/dev/null && break
      sleep 5
    done
  fi
  sleep 5

  L="$LOG_DIR/exploration.log"
  if [ ! -s "$L" ]; then
    echo "$TAG $run: RUN_NON_PARTITA (nessun log prodotto)" >> "$OUT"
    [ -n "$pgid" ] && kill -9 -"$pgid" 2>/dev/null
    kill_all; sleep 15; continue
  fi
  cp "$L" "$OUT_DIR/rel_expl_$run.log" 2>/dev/null
  # Mappa esplorata (drone 1 senza suffisso, per analisi_campagna.py).
  cp /tmp/c2x_explored_map_drone1.pcd "$OUT_DIR/rel_map_$run.pcd" 2>/dev/null
  for d in $(seq 2 "$DRONI"); do
    cp "/tmp/c2x_explored_map_drone$d.pcd" "$OUT_DIR/rel_map_${run}_$d.pcd" 2>/dev/null
  done
  # Il mondo come e' stato volato e la mappa realistica (superficie colorata +
  # anteprime), il risultato di ogni missione.
  cp "$LOG_DIR/mondo/mondo.json" "$OUT_DIR/rel_mondo_$run.json" 2>/dev/null
  for d in $(seq 1 "$DRONI"); do
    src="$LOG_DIR/mappa_realistica_drone$d"
    [ -d "$src" ] || continue
    dst="$OUT_DIR/rel_mappa3d_${run}_$d"
    mkdir -p "$dst"
    # punti.npz (nuvola lidar grezza, ~100 MB): serve a ritarare la ricostruzione
    # 3D offline sugli stessi voli invece di rifarli.
    cp "$src"/anteprima_*.png "$src"/mappa_realistica.ply "$src"/punti.npz "$dst/" 2>/dev/null
    # I fotogrammi della telecamera non si archiviano: servono solo a colorare la
    # mappa 3D (vedi FOTO in run_multi_exploration.sh).
  done
  # Conserva anche i log PX4: se un drone si ribalta, la causa puo' essere un
  # failsafe dell'autopilota (PX4 esce da offboard se l'heartbeat
  # OffboardControlMode scende sotto 2Hz), e quella prova esiste solo qui.
  # Solo le istanze REALMENTE avviate: con "for i in 0 1 2" fisso, una campagna
  # a due droni copiava anche il registro rimasto nella cartella dell'istanza 2
  # da una campagna precedente - quattro file identici da 141 s che finivano
  # nell'analisi come se fossero voli di questa campagna.
  for i in $(seq 0 $((DRONI - 1))); do
    cp "$LOG_DIR/px4_$i.log" "$OUT_DIR/rel_px4_${run}_$i.log" 2>/dev/null
    # E il registro di volo BINARIO, che e' l'unico posto dove esistono assetto,
    # riferimento e velocita' campionati. Senza, un ribaltamento si puo' solo
    # ipotizzare dai messaggi di testo: e' costato diverse diagnosi sbagliate
    # prima di estrarlo a mano. Il piu' recente e' quello della run appena finita.
    newest=$(ls -t "$PX4_ROOT/instance_$i/log"/*/*.ulg 2>/dev/null | head -1)
    [ -n "$newest" ] && cp "$newest" "$OUT_DIR/rel_ulog_${run}_$i.ulg" 2>/dev/null
  done

  t0=$(grep -oE '17[0-9]{8}' "$L" | head -1)
  t1=$(grep -oE '17[0-9]{8}' "$L" | tail -1)
  dur=$(( ${t1:-0} - ${t0:-0} ))
  cyc=""
  for d in 1 2 3; do
    n=$((6 + 8 * (d - 1)))
    cyc="$cyc $(grep -c "exploration_node-$n.*Drone $d ====" "$L" 2>/dev/null)"
  done
  {
    echo "$TAG $run:"
    echo "  finiti=$(grep -c 'has finished expl' "$L" 2>/dev/null)/$DRONI"
    echo "  crash_fisici=$(grep -c 'CRASH DETECTED' "$L" 2>/dev/null)"
    echo "  segfault=$(grep -cE 'Segmentation fault|Aborted \(' "$L" 2>/dev/null)"
    echo "  durata=${dur}s"
    echo "  cicli_per_drone=$cyc"
    echo "  clamp=$(grep -c 'clamped to' "$L" 2>/dev/null)"
    echo "  reanchor=$(grep -c 'behind its trajectory' "$L" 2>/dev/null)"
    echo "  watchdog=$(grep -c 'gained only' "$L" 2>/dev/null)"
    # Indicatore di impallamento: rapporto fra il drone che ha pianificato di
    # piu' e la mediana. Sopra 3 significa che uno sta macinando cicli mentre
    # gli altri hanno finito - il sintomo dell'"avanti e indietro sulla stessa
    # zona" - anche quando la run nel complesso risulta pulita.
    # max/min, non max/mediana: con cicli 34/1/35 la mediana e' 34 e il rapporto
    # dava 1.0, nascondendo che un drone non aveva fatto assolutamente nulla.
    echo "  squilibrio=$(echo $cyc | tr ' ' '\n' | grep -v '^$' | sort -n | awk '{a[NR]=$1} END {if (NR>=2) printf "%.1f", a[NR]/(a[1]>0?a[1]:0.5); else print "n/d"}')"
  } >> "$OUT"

  [ -n "$pgid" ] && kill -9 -"$pgid" 2>/dev/null
  kill -9 "$script_pid" 2>/dev/null
  sleep 3
  kill_all
  sleep 15
done
echo "TUTTE_LE_RUN_FINITE" >> "$OUT"
cp "$OUT" "$OUT_TAGGED"
