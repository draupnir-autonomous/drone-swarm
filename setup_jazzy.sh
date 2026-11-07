#!/bin/bash
# Installa le dipendenze di sistema per far girare C2-Explorer su Ubuntu 24.04:
#   ROS2 Jazzy + Gazebo Harmonic + dipendenze di build di PX4 v1.17.
#
# Il progetto originale era per Ubuntu 22.04 + ROS2 Humble; su 24.04 la coppia
# ufficiale e' Jazzy + Harmonic.
#
# Uso (NON con sudo davanti: la chiede lo script quando serve, e le
# dipendenze Python di PX4 vanno installate per l'utente, non per root):
#   bash setup_jazzy.sh

set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG="$DIR/setup_jazzy.log"
exec > >(tee -a "$LOG") 2>&1

if [ "$(id -u)" -eq 0 ]; then
  echo "Non lanciare questo script con sudo: esegui 'bash setup_jazzy.sh'."
  exit 1
fi

echo "=== [0/5] Password di amministratore ==="
sudo -v
# Tiene viva la sessione sudo finche' lo script gira (l'installazione dura piu'
# dei 15 minuti di validita' predefiniti).
( while true; do sudo -n true; sleep 50; kill -0 "$$" 2>/dev/null || exit; done ) 2>/dev/null &

echo "=== [1/5] Strumenti di base ==="
sudo apt-get update -y
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y software-properties-common curl wget lsb-release gnupg
sudo add-apt-repository -y universe

echo "=== [2/5] Repository ROS2 ==="
if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F "tag_name" | awk -F\" '{print $4}')
  CODENAME=$(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
  TMP_DEB=$(mktemp --suffix=.deb)
  curl -fL -o "$TMP_DEB" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${CODENAME}_all.deb"
  sudo dpkg -i "$TMP_DEB"
  rm -f "$TMP_DEB"
fi
sudo apt-get update -y

echo "=== [3/5] Dipendenze PX4 + Gazebo Harmonic (script ufficiale PX4) ==="
bash "$DIR/PX4-Autopilot/Tools/setup/ubuntu.sh" --no-nuttx

echo "=== [4/5] ROS2 Jazzy e librerie del workspace ==="
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
  ros-jazzy-desktop \
  ros-dev-tools \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-pcl-conversions \
  ros-jazzy-launch-testing-ros \
  libpcl-dev \
  libeigen3-dev \
  libnlopt-dev \
  libnlopt-cxx-dev \
  python3-numpy \
  python3-matplotlib \
  zstd

echo "=== [5/5] rosdep ==="
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init || true
rosdep update || true

echo ""
echo "=== INSTALLAZIONE DI SISTEMA COMPLETATA ==="
