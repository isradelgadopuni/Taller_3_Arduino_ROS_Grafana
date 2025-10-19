# Comandos para compilar

source /opt/ros/jazzy/setup.bash
cd /root/ros2_ws
colcon build --symlink-install
source install/setup.bash

# Iniciar imagen en Reto/

docker build -t wsn-reto:latest .

