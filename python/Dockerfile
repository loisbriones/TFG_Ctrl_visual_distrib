# 1. Usamos la imagen base de Kilted Kaiju (2025/2026)
FROM ros:kilted-ros-base

# Evitamos preguntas interactivas durante la instalación
ENV DEBIAN_FRONTEND=noninteractive

# 2. Instalamos las dependencias actualizadas a Kilted
RUN apt-get update && apt-get install -y \
  ros-kilted-rosidl-default-generators \
  ros-kilted-rosidl-default-runtime \
  python3-opencv \
  python3-numpy \
  python3-serial \
  && rm -rf /var/lib/apt/lists/*

# 3. Directorio de trabajo
WORKDIR /ros2_ws

RUN echo "source /opt/ros/kilted/setup.bash" >> ~/.bashrc
