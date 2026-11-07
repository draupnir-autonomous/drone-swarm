#!/bin/bash
# Confronto 2 contro 3 droni a parita' di tutto il resto.
#
# Le due campagne vanno IN SEQUENZA e ciascuna archivia i propri risultati
# prima che parta la successiva: reliability.txt viene azzerato a ogni avvio e
# i registri per-run stanno in una cartella condivisa, quindi senza archiviare
# subito la prima campagna sparisce - e senza la pulizia iniziale (ora dentro
# reliability_campaign.sh) la seconda eredita i file della prima. Entrambi gli
# inciampi sono gia' successi in questa serie di misure: un confronto si e'
# ritrovato 12 registri di volo dove ne erano attesi 8, con dentro un terzo
# drone mai decollato.
set -u
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT=/tmp/c2x_reliability
cd "$WS" || exit 1

esegui() {
  # Assegnazioni separate: in un solo 'local' bash espande TUTTE le parole prima
  # di assegnarle, quindi ${tag} risulterebbe non definita e con set -u lo
  # script abortisce all'istante.
  local tag="$1"
  local droni="$2"
  local cart="$OUT/${tag}_ulog"
  rm -rf "$cart"; mkdir -p "$cart"
  ./scripts/reliability_campaign.sh 4 "$tag" 700 office_small "$droni" > "/tmp/c2x_${tag}.log" 2>&1
  cp "$OUT/reliability.txt" "$OUT/${tag}.out" 2>/dev/null
  cp "$OUT"/rel_ulog_*.ulg "$cart/" 2>/dev/null
  cp "$OUT"/rel_expl_*.log "$cart/" 2>/dev/null
  echo "[$tag] conclusa: $(grep -c 'durata=' "$OUT/${tag}.out" 2>/dev/null) run, $(ls "$cart"/*.ulg 2>/dev/null | wc -l) registri di volo"
}

esegui A2 2
sleep 15
esegui A3 3
echo FATTO
