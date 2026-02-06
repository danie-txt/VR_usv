#!/bin/bash

echo "--- Limpiando procesos antiguos ---"
killall -9 gzserver gzclient 2>/dev/null

echo "--- Borrando configuración de Gazebo corrupta ---"
rm -rf ~/.gazebo/gui.ini

echo "--- Configurando variables de entorno ---"
export LIBGL_ALWAYS_SOFTWARE=1
# Añadimos la ruta de modelos de Gazebo y Turtlebot3
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models

echo "--- Cargando entornos (Sourcing) ---"
source /usr/share/gazebo/setup.sh
source /opt/ros/humble/setup.bash
source ~/vr_usv/install/setup.bash

echo "--- Lanzando Articubot One ---"
ros2 launch vr_usv launch_sim.launch.py

