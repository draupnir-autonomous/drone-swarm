#!/usr/bin/env bash
# Inizio lavoro: git pull in sicurezza + controlli di questa macchina.
# Lo chiama /riprendi; si puo' usare anche a mano prima di aprire Claude.
#
# Uso: tools/claude/inizio_lavoro.sh [--forza]
#   --forza  fa il pull anche se c'e' una simulazione in corso (sconsigliato)
set -uo pipefail
. "$(dirname "${BASH_SOURCE[0]}")/comune.sh"
cd "$ROOT" || exit 1
FORZA="${1:-}"
MACCHINA="$(nome_macchina)"
echo "== Macchina: $MACCHINA ($(uname -m)), progetto in $ROOT"

# 0. Configurazione di Claude per questa macchina (file locale, non versionato: si sistema da solo)
if ! grep -q "\"autoMemoryDirectory\": \"$ROOT/.claude/memoria\"" .claude/settings.local.json 2>/dev/null; then
  echo "== Configuro Claude per questa macchina (memoria nel repo)"
  bash tools/claude/configura_claude.sh
  echo "   La nuova posizione della memoria vale dalla PROSSIMA sessione di Claude."
fi

# 1. Pull, solo se e' sicuro
echo "== Sincronizzazione"
fai_pull=1
if ! git remote get-url origin >/dev/null 2>&1; then
  echo "Nessun remoto 'origin' (vedi tools/PORTABILITA.md): niente pull."; fai_pull=0
fi
attive="$(run_attive)"
if [ "$fai_pull" = 1 ] && [ -n "$attive" ] && [ "$FORZA" != "--forza" ]; then
  echo "Simulazione in corso: niente pull (bash legge gli script mentre girano)."
  echo "$attive" | head -5 | cut -c1-160 | sed 's/^/  /'
  fai_pull=0
fi
if [ "$fai_pull" = 1 ] && [ -n "$(git status --porcelain)" ]; then
  echo "Modifiche locali non committate: niente pull."
  git status --short | head -20 | sed 's/^/  /'
  echo "Decidi: committarle (tools/claude/fine_lavoro.sh), metterle da parte (git stash) o scartarle."
  fai_pull=0
fi
if [ "$fai_pull" = 1 ]; then
  prima="$(git rev-parse HEAD)"
  if git_muto pull --ff-only --quiet; then
    dopo="$(git rev-parse HEAD)"
    if [ "$prima" = "$dopo" ]; then
      echo "Gia' aggiornato."
    else
      echo "Arrivati $(git rev-list --count "$prima..$dopo") commit:"
      git log --format='  %h %cd %s' --date=format:'%d/%m %H:%M' "$prima..$dopo" | head -15
      git diff --stat "$prima" "$dopo" | tail -1 | sed 's/^/  /'
    fi
  else
    echo "PULL NON RIUSCITO. Se i rami sono divergenti (commit nuovi sia qui sia su GitHub, cioe' si e'"
    echo "lavorato su due macchine senza passaggio di consegne) va deciso come unirli. Per vedere la"
    echo "situazione: git log --oneline --graph --all -15. Di solito basta: git pull --rebase"
  fi
fi

# 2. Controlli della macchina
echo "== Controlli"
bash tools/px4_patch.sh stato

arch="$(uname -m)"
if [ ! -d ros2_ws/install ]; then
  echo "ROS2: build assente -> tools/setup_macchina.sh --compila (20+ minuti)"
else
  campione="$(find ros2_ws/install -name '*.so' -type f 2>/dev/null | head -1)"
  a="$([ -n "$campione" ] && arch_elf "$campione")"
  if [ -n "$a" ] && [ "$a" != "$arch" ]; then
    echo "ROS2: build fatta per $a, questa macchina e' $arch -> rm -rf ros2_ws/build ros2_ws/install ros2_ws/log e ricompilare"
  else
    # Per pacchetto: sorgenti piu' recenti dell'ultima build di QUEL pacchetto (build/<p>/colcon_build.rc)
    vecchi=""; mai=""
    while IFS= read -r xml; do
      d="$(dirname "$xml")"; [ -f "$d/COLCON_IGNORE" ] && continue
      p="$(sed -n 's:.*<name>\(.*\)</name>.*:\1:p' "$xml" | head -1)"
      rc="ros2_ws/build/$p/colcon_build.rc"
      if [ ! -f "$rc" ]; then mai="$mai $p"; continue; fi
      [ -n "$(find "$d" -type f -newer "$rc" \( -name '*.cpp' -o -name '*.h' -o -name '*.hpp' -o -name '*.py' -o -name 'CMakeLists.txt' -o -name '*.msg' -o -name '*.srv' -o -name '*.yaml' -o -name 'package.xml' \) -print -quit 2>/dev/null)" ] && vecchi="$vecchi $p"
    done < <(find ros2_ws/src -name package.xml -not -path '*/build/*')
    if [ -n "$vecchi$mai" ]; then
      [ -n "$vecchi" ] && echo "ROS2: sorgenti piu' recenti della build in:$vecchi"
      [ -n "$mai" ]    && echo "ROS2: pacchetti mai compilati qui:$mai"
      echo "  -> cd ros2_ws && colcon build --symlink-install --packages-up-to <pacchetti> (opzioni in CLAUDE.md)"
    else
      echo "ROS2: ogni pacchetto e' compilato dopo l'ultima modifica dei suoi sorgenti."
    fi
  fi
fi
if [ -x PX4-Autopilot/build/px4_sitl_default/bin/px4 ]; then
  a="$(arch_elf PX4-Autopilot/build/px4_sitl_default/bin/px4)"
  [ -n "$a" ] && [ "$a" != "$arch" ] && echo "PX4: build per $a, va rifatta (cd PX4-Autopilot && make px4_sitl_default)" || echo "PX4: build presente."
elif [ -d PX4-Autopilot ]; then
  echo "PX4: build assente -> cd PX4-Autopilot && make px4_sitl_default"
fi
python3 -c "import open3d" 2>/dev/null || echo "Open3D: mancante (serve a preparare ogni missione) -> pip install --user --break-system-packages open3d==0.20.0"
command -v MicroXRCEAgent >/dev/null || [ -x "$HOME/.local/bin/MicroXRCEAgent" ] || echo "MicroXRCEAgent: non installato -> tools/setup_macchina.sh --compila"
[ -f terze_parti/livox_sdk2/lib/liblivox_lidar_sdk_static.a ] || echo "Livox SDK2: non compilato (serve solo per il drone vero) -> tools/setup_macchina.sh --compila"

# 3. Consegne
echo "== Consegne"
sed -n '3p' .claude/CONSEGNE.md 2>/dev/null | sed 's/^> //'
echo "Ultimi commit:"
git log --format='  %h %cd %s' --date=format:'%d/%m %H:%M' -5 2>/dev/null
exit 0
