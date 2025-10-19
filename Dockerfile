FROM osrf/ros:jazzy-desktop
SHELL ["/bin/bash","-lc"]

# Herramientas
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-matplotlib \
    python3-colcon-common-extensions \
    python3-colcon-ros \
    python3-serial \
    python3-prometheus-client \ 
    curl \
    nano \
    tree \
    iproute2 \
    net-tools \
    iputils-ping \
 && rm -rf /var/lib/apt/lists/*
 
# Documenta el puerto del exporter
EXPOSE 8000

#Copiar paquete
WORKDIR /root/ros2_ws
RUN mkdir -p src
COPY ./sensor_serial /root/ros2_ws/src/sensor_serial

#Entrypoint oficial de la imagen
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

