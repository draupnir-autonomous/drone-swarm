#!/usr/bin/env bash
# Prepara QUESTA macchina per il progetto (Jetson, PC A, PC B). Si puo' rilanciare: salta cio'
# che e' gia' fatto.
#
# Uso (dalla cartella del progetto, appena clonato da GitHub):
#   tools/setup_macchina.sh [--dipendenze] [--compila] [--nome pc-a]
#
#   (sempre)      scarica PX4 v1.17.0 e applica patches/px4/, scarica Micro-XRCE-DDS-Agent v3.0.1,
#                 configura Claude per questa macchina (memoria nel repo, CLAUDE.local.md)
#   --dipendenze  installa ROS2 Jazzy, Gazebo Harmonic e le dipendenze di PX4 (sudo; ~30-60 min):
#                 setup_jetson_jazzy.sh su aarch64, setup_jazzy.sh su x86_64
#   --compila     compila Livox SDK2 (dentro il progetto), l'agente DDS (sudo make install), PX4 SITL
#                 e il workspace ROS2 (20+ minuti)
#   --nome N      nome breve della macchina per Claude (jetson, pc-a, pc-b)
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

DIPENDENZE=0; COMPILA=0; NOME=""
while [ $# -gt 0 ]; do
  case "$1" in
    --dipendenze) DIPENDENZE=1 ;;
    --compila)    COMPILA=1 ;;
    --nome)       NOME="${2:?manca il nome}"; shift ;;
    -h|--help)    sed -n '2,16p' "$0"; exit 0 ;;
    *) echo "Opzione sconosciuta: $1"; exit 1 ;;
  esac
  shift
done

PX4_TAG="$(cat patches/px4/VERSIONE 2>/dev/null || echo v1.17.0)"
XRCE_TAG="v3.0.1"
ARCH="$(uname -m)"
JOBS="$(nproc)"; [ "$ARCH" = aarch64 ] && JOBS=$(( JOBS > 4 ? 4 : JOBS ))   # Jetson: 7 GB di RAM

passo() { echo; echo "==> $*"; }

. /etc/os-release
[ "${VERSION_ID:-}" = "24.04" ] || echo "ATTENZIONE: il progetto e' provato su Ubuntu 24.04, qui c'e' ${PRETTY_NAME:-?}."

passo "PX4-Autopilot $PX4_TAG"
if [ ! -d PX4-Autopilot/.git ]; then
  git clone --branch "$PX4_TAG" --recursive https://github.com/PX4/PX4-Autopilot.git PX4-Autopilot
fi
bash tools/px4_patch.sh applica

passo "Micro-XRCE-DDS-Agent $XRCE_TAG"
if [ ! -d Micro-XRCE-DDS-Agent/.git ]; then
  git clone --branch "$XRCE_TAG" https://github.com/eProsima/Micro-XRCE-DDS-Agent.git Micro-XRCE-DDS-Agent
else
  echo "gia' presente."
fi

if [ "$DIPENDENZE" = 1 ]; then
  if [ "$ARCH" = aarch64 ]; then
    passo "Dipendenze di sistema (setup_jetson_jazzy.sh, chiede sudo)"; bash setup_jetson_jazzy.sh
  else
    passo "Dipendenze di sistema (setup_jazzy.sh, chiede sudo)"; bash setup_jazzy.sh
  fi
  # Uguali su Jetson e PC. I due script sopra hanno elenchi di pacchetti diversi: rosdep
  # installa quelli dichiarati nei package.xml del workspace, qualunque sia l'architettura.
  passo "Dipendenze comuni: rosdep del workspace, SciPy, Open3D 0.20.0 (come sulla Jetson)"
  ( set +u; source /opt/ros/jazzy/setup.bash; set -u
    rosdep install --from-paths ros2_ws/src --ignore-src -r -y --rosdistro jazzy \
      --skip-keys "cmake_modules" ) || echo "ATTENZIONE: rosdep non ha installato tutto (vedi sopra)."
  sudo apt-get install -y python3-scipy
  # Open3D: lo usano prepara_mondo.py e geometria_mondo.py (preparazione di ogni missione) e
  # ricostruisci_mappa.py. Stessa versione della Jetson; c'e' la ruota sia aarch64 sia x86_64.
  python3 -c "import open3d" 2>/dev/null \
    || python3 -m pip install --user --break-system-packages open3d==0.20.0
fi

if [ "$COMPILA" = 1 ]; then
  [ -f /opt/ros/jazzy/setup.bash ] || { echo "ROS2 Jazzy non installato: rilancia con --dipendenze"; exit 1; }

  passo "Livox SDK2 -> terze_parti/livox_sdk2 (serve al driver del Mid-360, drone vero)"
  if [ ! -f terze_parti/livox_sdk2/lib/liblivox_lidar_sdk_static.a ]; then
    cmake -S terze_parti/Livox-SDK2 -B terze_parti/Livox-SDK2/build \
      -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$ROOT/terze_parti/livox_sdk2"
    cmake --build terze_parti/Livox-SDK2/build -j"$JOBS"
    cmake --install terze_parti/Livox-SDK2/build
  else
    echo "gia' compilato."
  fi

  passo "MicroXRCEAgent (sudo make install in /usr/local, come sulla Jetson)"
  if ! command -v MicroXRCEAgent >/dev/null; then
    cmake -S Micro-XRCE-DDS-Agent -B Micro-XRCE-DDS-Agent/build
    cmake --build Micro-XRCE-DDS-Agent/build -j"$JOBS"
    sudo cmake --install Micro-XRCE-DDS-Agent/build
    sudo ldconfig /usr/local/lib/
  else
    echo "gia' installato: $(command -v MicroXRCEAgent)"
  fi

  passo "PX4 SITL (make px4_sitl_default)"
  # Senza l'ambiente ROS davanti nel PATH (il gz ridotto di ROS confonde la build di PX4)
  ( cd PX4-Autopilot && make px4_sitl_default )

  passo "Workspace ROS2 (colcon build, 20+ minuti)"
  (
    cd ros2_ws
    set +u; source /opt/ros/jazzy/setup.bash; set -u
    colcon build --symlink-install --continue-on-error --parallel-workers "$JOBS" \
      --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_SOPHUS_TESTS=OFF -DBUILD_SOPHUS_EXAMPLES=OFF
    python3 scripts/percorsi_relativi.py || true
  )
fi

passo "Claude Code per questa macchina"
bash tools/claude/configura_claude.sh ${NOME:+"$NOME"}

echo
echo "FATTO. Prossimi passi:"
[ "$DIPENDENZE" = 0 ] && ! [ -f /opt/ros/jazzy/setup.bash ] && echo "  - mancano ROS2/Gazebo: tools/setup_macchina.sh --dipendenze"
[ "$COMPILA" = 0 ] && echo "  - per compilare tutto: tools/setup_macchina.sh --compila"
echo "  - apri il progetto con Claude (app desktop o 'claude' da terminale) e scrivi /riprendi"
