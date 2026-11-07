#!/bin/bash
# Installa le dipendenze di sistema per Drone_fabio su Ubuntu 24.04 (Jetson, aarch64).
# Su 24.04 ROS2 Humble non esiste: si usa ROS2 Jazzy, che e' accoppiato
# ufficialmente a Gazebo Harmonic (lo stesso simulatore del progetto originale).
#
# Uso (da utente normale, chiede la password sudo una volta):
#   ./setup_jetson_jazzy.sh
set -euo pipefail

PX4_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/PX4-Autopilot"
export DEBIAN_FRONTEND=noninteractive

echo "==> [1/6] strumenti di base e locale"
sudo apt-get update -q
sudo apt-get install -y -q software-properties-common curl wget gnupg lsb-release locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sudo add-apt-repository -y universe

echo "==> [2/6] repository ROS2 (ros2-apt-source)"
if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  VER=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | awk -F'"' '{print $4}')
  curl -L -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${VER}/ros2-apt-source_${VER}.$(. /etc/os-release && echo "$VERSION_CODENAME")_all.deb"
  sudo apt-get install -y /tmp/ros2-apt-source.deb
fi

echo "==> [3/6] repository Gazebo (OSRF) per gz-harmonic"
sudo wget -q https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
sudo apt-get update -q

echo "==> [4/6] ROS2 Jazzy + Gazebo Harmonic + dipendenze del workspace e di PX4"
sudo apt-get install -y -q --no-install-recommends \
  ros-jazzy-desktop ros-dev-tools \
  ros-jazzy-ros-gz ros-jazzy-pcl-conversions ros-jazzy-pcl-ros \
  ros-jazzy-tf2-ros ros-jazzy-message-filters ros-jazzy-eigen3-cmake-module \
  ros-jazzy-rclcpp-components ros-jazzy-rmw-fastrtps-cpp \
  gz-harmonic cppzmq-dev libunwind-dev \
  libpcl-dev pcl-tools libnlopt-cxx-dev libnlopt-dev libeigen3-dev \
  build-essential ccache cmake ninja-build git gdb make file rsync unzip zip \
  astyle libssl-dev libxml2-dev libxml2-utils \
  python3 python3-dev python3-pip python3-setuptools python3-wheel python3-pil \
  python3-numpy python3-empy python3-jinja2 python3-yaml python3-lxml \
  dmidecode ffmpeg gnome-terminal \
  gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-ugly gstreamer1.0-libav libgstreamer-plugins-base1.0-dev

echo "==> [5/6] rosdep + pacchetti python di PX4 (utente, non root)"
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
rosdep update
# Su 24.04 pip richiede --break-system-packages (come fa Tools/setup/ubuntu.sh di PX4).
python3 -m pip install --user --break-system-packages -r "$PX4_DIR/Tools/setup/requirements.txt"

echo "==> [6/6] swap da 8 GB (la macchina ha 7.3 GB di RAM e nessuno swap:"
echo "    compilare PX4 e FAST-LIO senza swap rischia l'OOM killer)"
if ! swapon --show | grep -q /swapfile; then
  sudo fallocate -l 8G /swapfile
  sudo chmod 600 /swapfile
  sudo mkswap /swapfile
  sudo swapon /swapfile
  grep -q '^/swapfile' /etc/fstab || echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab >/dev/null
fi

echo
echo "FATTO. Aggiungi a ~/.bashrc:  source /opt/ros/jazzy/setup.bash"
