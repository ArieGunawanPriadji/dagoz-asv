#!/usr/bin/env bash
# Jalankan sekali di companion computer (Jetson/RPi) untuk siapin environment
set -e

sudo apt update
sudo apt install -y build-essential cmake libopencv-dev \
    ros-jazzy-cv-bridge ros-jazzy-mavros ros-jazzy-mavros-extras \
    python3-colcon-common-extensions

wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash install_geographiclib_datasets.sh
rm install_geographiclib_datasets.sh

echo "Setup selesai. Jangan lupa 'source /opt/ros/jazzy/setup.bash'"
