# Funzioni comuni agli script di portabilità (da includere con: . tools/claude/comune.sh)

# Radice del progetto: la cartella che contiene tools/ (funziona da qualsiasi percorso e utente).
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

# Nome breve della macchina (jetson, pc-a, pc-b), scelto da configura_claude.sh.
nome_macchina() {
  local n
  n="$(cat "$ROOT/.claude/macchina" 2>/dev/null)"
  echo "${n:-$(hostname)}"
}

# Processi che rendono pericoloso un git pull: bash legge gli script mentre girano.
run_attive() {
  pgrep -af "run_multi_exploration|campagna_sciame|reliability_campaign|run_reale_exploration|avvia_collaudo|gz sim|px4_sitl_default/bin/px4" 2>/dev/null \
    | grep -v -E "pgrep|hook_avvio|inizio_lavoro|fine_lavoro" || true
}

# git senza richieste interattive di password (per hook e controlli automatici).
git_muto() {
  GIT_TERMINAL_PROMPT=0 GIT_SSH_COMMAND="ssh -o BatchMode=yes -o ConnectTimeout=5" git "$@"
}

# Architettura di un file ELF: aarch64 / x86_64 / vuoto.
arch_elf() {
  case "$(file -b "$1" 2>/dev/null)" in
    *aarch64*) echo aarch64 ;;
    *x86-64*)  echo x86_64 ;;
  esac
}
