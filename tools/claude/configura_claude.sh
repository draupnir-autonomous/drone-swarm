#!/usr/bin/env bash
# Configura Claude Code per questo progetto su QUESTA macchina. Da rifare se sposti la cartella
# (lo fa da solo inizio_lavoro.sh quando se ne accorge).
#
#  - .claude/settings.local.json  autoMemoryDirectory = <progetto>/.claude/memoria
#                                 (la memoria automatica di Claude viaggia con il repo)
#  - .claude/macchina             nome breve della macchina: jetson, pc-a, pc-b, ...
#  - CLAUDE.local.md              scheda della macchina, letta da Claude a ogni sessione
#  - copia in .claude/memoria/ eventuali memorie rimaste nella cartella di Claude di questa macchina
# Sono tutti file locali (in .gitignore), tranne la memoria.
#
# Uso: tools/claude/configura_claude.sh [nome-macchina]
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]}")/comune.sh"
cd "$ROOT"
mkdir -p .claude/memoria

# 1. Nome della macchina
nome="${1:-$(cat .claude/macchina 2>/dev/null || true)}"
if [ -z "$nome" ]; then
  if [ -f /etc/nv_tegra_release ]; then nome=jetson; else nome="$(hostname)"; fi
  if [ -t 0 ]; then
    read -r -p "Nome breve di questa macchina (jetson, pc-a, pc-b) [$nome]: " r || true
    nome="${r:-$nome}"
  fi
fi
echo "$nome" > .claude/macchina

# 2. settings.local.json: aggiunge/aggiorna solo autoMemoryDirectory, il resto resta com'e'
python3 -I - "$ROOT/.claude/settings.local.json" "$ROOT/.claude/memoria" <<'EOF'
import json, os, sys
path, mem = sys.argv[1], sys.argv[2]
data = {}
if os.path.exists(path):
    with open(path) as f:
        data = json.load(f)
data["autoMemoryDirectory"] = mem
with open(path, "w") as f:
    json.dump(data, f, indent=2, ensure_ascii=False)
    f.write("\n")
EOF
echo "Memoria automatica di Claude -> $ROOT/.claude/memoria"

# 3. Memorie rimaste nella cartella standard di Claude (~/.claude/projects/<percorso>/memory):
#    si copiano quelle che mancano nel repo, senza sovrascrivere niente.
cfg="${CLAUDE_CONFIG_DIR:-$HOME/.claude}"
vecchia="$cfg/projects/$(printf '%s' "$ROOT" | sed 's/[^A-Za-z0-9]/-/g')/memory"
if [ -d "$vecchia" ]; then
  for f in "$vecchia"/*.md; do
    [ -e "$f" ] || continue
    b="$(basename "$f")"
    if [ "$b" = MEMORY.md ]; then
      continue
    elif [ ! -e ".claude/memoria/$b" ]; then
      cp -p "$f" ".claude/memoria/$b"
      echo "Memoria recuperata da $vecchia: $b (aggiungi la sua riga a .claude/memoria/MEMORY.md)"
    elif ! cmp -s "$f" ".claude/memoria/$b"; then
      echo "Nota: $b esiste sia in $vecchia sia nel repo, con contenuto diverso: tengo quella del repo."
    fi
  done
fi

# 4. Scheda della macchina (solo se non esiste: Claude e l'utente possono aggiungere note)
if [ ! -f CLAUDE.local.md ]; then
  gpu="nessuna GPU NVIDIA rilevata"
  if [ -f /etc/nv_tegra_release ]; then
    gpu="Jetson integrata ($(head -1 /etc/nv_tegra_release | cut -d, -f1-2 | sed 's/^# //'))"
  elif command -v nvidia-smi >/dev/null; then
    gpu="$(nvidia-smi --query-gpu=name,memory.total --format=csv,noheader 2>/dev/null | head -1)"
  fi
  ros="$(ls /opt/ros 2>/dev/null | tr '\n' ' ')"
  cat > CLAUDE.local.md <<EOF
# Questa macchina: $nome

Generato da tools/claude/configura_claude.sh il $(date +%Y-%m-%d). Non versionato: vale solo qui.

- hostname: $(hostname), architettura $(uname -m), $(. /etc/os-release && echo "$PRETTY_NAME")
- CPU: $(nproc) core; RAM: $(free -g | awk '/^Mem/{print $2}') GB; swap: $(free -g | awk '/^Swap/{print $2}') GB
- GPU: $gpu
- ROS2 in /opt/ros: ${ros:-non installato}
- progetto in: $ROOT

## Note solo per questa macchina
<!-- Claude: aggiungi qui cio' che vale solo su questa macchina (prestazioni, RTF tipico,
     dispositivi collegati, cose che mancano). Quello che vale ovunque va in CLAUDE.md o in memoria. -->
EOF
  echo "Creato CLAUDE.local.md (scheda di \"$nome\")."
fi
echo "Fatto. Se Claude e' aperto su questo progetto, la nuova configurazione vale dalla prossima sessione."
