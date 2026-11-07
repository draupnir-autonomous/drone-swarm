#!/bin/bash
# Missione a drone singolo: e' run_multi_exploration.sh con un drone.
#
# Fino a ottobre 2026 qui c'era un secondo orchestratore, piu' vecchio, con le sue
# configurazioni (single_drone_*.yaml), i suoi soli due mondi e come risultato
# un'immagine dall'alto della mappa. Ora ogni missione, su qualunque mondo, passa
# dallo stesso script e produce gli stessi risultati (la mappa 3D): vedi
# run_multi_exploration.sh e prepara_mondo.py. La versione vecchia e' conservata,
# non eseguibile, in scripts/archivio/run_exploration_2026-10.sh.
#
# Uso: ./run_exploration.sh [mondo]     (nome di un mondo o percorso di un .sdf)
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/run_multi_exploration.sh" "${1:-empty_room}" 1
