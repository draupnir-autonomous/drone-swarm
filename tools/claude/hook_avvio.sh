#!/usr/bin/env bash
# Hook SessionStart di Claude Code (vedi .claude/settings.json).
# Quello che stampa entra nel contesto di Claude all'avvio della sessione: su che macchina siamo,
# se il repo e' indietro rispetto a GitHub, se gira una simulazione, se PX4 e build sono a posto.
# Non modifica niente (solo git fetch) ed esce sempre con 0, anche se qualcosa fallisce.
. "$(dirname "${BASH_SOURCE[0]}")/comune.sh" 2>/dev/null || exit 0
cd "$ROOT" 2>/dev/null || exit 0

MACCHINA="$(nome_macchina)"
echo "## Avvio sessione: $(date '+%Y-%m-%d %H:%M'), macchina \"$MACCHINA\" ($(uname -m))"

# Memoria automatica nel repo?
if ! grep -q "\"autoMemoryDirectory\": \"$ROOT/.claude/memoria\"" .claude/settings.local.json 2>/dev/null; then
  echo "- ATTENZIONE: su questa macchina la memoria automatica non punta a .claude/memoria/ (manca o e' sbagliato"
  echo "  .claude/settings.local.json). Proponi: bash tools/claude/configura_claude.sh, poi riavviare la sessione."
fi

suggerisci_riprendi=""
if git rev-parse --git-dir >/dev/null 2>&1; then
  ramo="$(git branch --show-current)"
  modificati="$(git status --porcelain 2>/dev/null | wc -l)"
  stato_remoto="nessun remoto origin"
  if git remote get-url origin >/dev/null 2>&1; then
    if timeout 8 bash -c '. "$1"; git_muto fetch --quiet origin' _ "$ROOT/tools/claude/comune.sh" 2>/dev/null; then
      fetch="aggiornato ora"
    else
      fetch="fetch non riuscito: dati di origin forse vecchi"
    fi
    if git rev-parse --abbrev-ref '@{u}' >/dev/null 2>&1; then
      read -r avanti indietro < <(git rev-list --left-right --count 'HEAD...@{u}' 2>/dev/null || echo "0 0")
      stato_remoto="avanti $avanti, indietro $indietro rispetto a $(git rev-parse --abbrev-ref '@{u}') ($fetch)"
      [ "${indietro:-0}" -gt 0 ] && suggerisci_riprendi="il repo e' indietro di $indietro commit"
    else
      stato_remoto="ramo senza upstream ($fetch)"
    fi
  fi
  echo "- git: ramo $ramo, $modificati file non committati, $stato_remoto"
  ultimo="$(git log -1 --format='%s' 2>/dev/null)"
  [ -n "$ultimo" ] && echo "- ultimo commit: $(git log -1 --format='%h %cd' --date=format:'%d/%m %H:%M') \"$ultimo\""
  # I commit di fine_lavoro.sh iniziano con [nome-macchina]
  case "$ultimo" in
    "[$MACCHINA]"*) ;;
    "["*"]"*) [ -z "$suggerisci_riprendi" ] && suggerisci_riprendi="l'ultimo lavoro e' stato fatto su un'altra macchina" ;;
  esac
fi

attive="$(run_attive)"
if [ -n "$attive" ]; then
  echo "- SIMULAZIONE/RUN IN CORSO su questa macchina: non fare git pull e non modificare gli script in uso:"
  echo "$attive" | head -5 | cut -c1-160 | sed 's/^/    /'
fi

[ -x tools/px4_patch.sh ] && bash tools/px4_patch.sh stato --breve 2>/dev/null | head -2 | sed 's/^/- /'

if [ ! -d ros2_ws/install ]; then
  echo "- build ROS2 assente su questa macchina (tools/setup_macchina.sh --compila)."
else
  campione="$(find ros2_ws/install -name '*.so' -type f 2>/dev/null | head -1)"
  a="$([ -n "$campione" ] && arch_elf "$campione")"
  [ -n "$a" ] && [ "$a" != "$(uname -m)" ] && echo "- build ROS2 fatta per $a, questa macchina e' $(uname -m): va rifatta."
fi

if [ -n "$suggerisci_riprendi" ]; then
  echo "- Nota per Claude: $suggerisci_riprendi. Prima di lavorare proponi all'utente /riprendi."
fi
exit 0
