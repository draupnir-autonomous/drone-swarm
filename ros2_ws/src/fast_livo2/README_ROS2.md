# FAST-LIVO2 Environment Setup (Ubuntu + ROS 2)

This document describes the environment setup process for building FAST-LIVO2 on Ubuntu with ROS 2 (tested on Humble and Jazzy), including dependency installation, third-party libraries, and workspace compilation.

---

## Prerequisites

- Ubuntu 22.04 (Humble) or 24.04 (Jazzy)
- ROS 2 installed
- sudo privileges
- Internet access

Source the ROS 2 environment before starting:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
```

---

## Dependency Installation

Install required system libraries:

```bash
sudo apt install -y \
  libpcl-dev \
  libpcl-conversions-dev \
  libeigen3-dev \
  libopencv-dev
```

Install ROS 2 packages:

```bash
sudo apt install -y \
  ros-$ROS_DISTRO-pcl-ros \
  ros-$ROS_DISTRO-cv-bridge \
  ros-$ROS_DISTRO-image-transport
```

---

## Livox SDK 2 Installation

Download, build, and install Livox SDK 2:

```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git -b v1.3.1
cd Livox-SDK2
mkdir build
cd build
cmake ..
make -j$(nproc)
sudo make install
```

---

## Sophus Installation

Clone and build Sophus (version 1.22.10):

```bash
git clone https://github.com/strasdat/Sophus.git -b 1.22.10
cd Sophus
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

---

## rpg_vikit Common Library Installation

Clone and build vikit_common:

```bash
git clone https://github.com/Robotic-Developer-Road/rpg_vikit.git
# fix service issue
cd rpg_vikit
sed -i 's/wait_for_service(std::chrono::milliseconds(100))/wait_for_service(std::chrono::milliseconds(5000))/g' vikit_ros/include/vikit/params_helper.h
# build and install
cd rpg_vikit/vikit_common
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

---

## ROS 2 Workspace Setup and Build

Create the ROS 2 workspace and clone required repositories:

```bash
mkdir -p ws_livo2/src
cd ws_livo2/src

git clone https://github.com/intel-sandbox/frameworks.industrial.motion-control.fast-livo2 -b humble
git clone https://github.com/Livox-SDK/livox_ros_driver2.git -b 1.2.6
```

Move the ROS package from rpg_vikit into the workspace:

```bash
mv rpg_vikit/vikit_ros ws_livo2/src
```

Return to the workspace root:

```bash
cd ws_livo2
```

Build the Livox ROS 2 driver:

```bash
cp src/livox_ros_driver2/package_ROS2.xml src/livox_ros_driver2/package.xml
colcon build \
  --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=$ROS_DISTRO \
  --packages-select livox_ros_driver2
```

Build vikit_ros:

```bash
colcon build --packages-select vikit_ros
```

Build FAST-LIVO2:

```bash
colcon build --packages-select fast_livo2
```

---

## Environment Setup

After building, source the workspace:

```bash
source ws_livo2/install/setup.bash
```

You may add this command to `~/.bashrc` for convenience.

---

## Bag Conversion

Convert a ROS1 bag to a ROS2 bag:

```bash
pip install rosbags
rosbags-convert --src YOUR_DOWNLOADED.bag --dst YOUR_DOWNLOADED
```

After conversion, edit `metadata.yaml` in the converted bag folder.
Update the following fields for ROS 2 compatibility:

```
# msg type
-     type: livox_ros_driver/msg/CustomMsg
+     type: livox_ros_driver2/msg/CustomMsg

# offered_qos_profiles
-     offered_qos_profiles: []
+     offered_qos_profiles: ""
```

If this is not updated correctly, `ros2 bag play` may fail with:
`yaml-cpp: bad conversion`.

---

## Launch Demo with ROS2 bag

```bash
# terminal 1 for SLAM
ros2 launch fast_livo2 mapping_avia.launch.py use_rviz:=true

# terminal 2 for play bag
ros2 bag play YOUR_DOWNLOADED

```
