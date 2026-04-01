FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Instalamos dependencias para mensajes, compilación y procesamiento de imagen
RUN apt-get update && apt-get install -y \
  ros-humble-rosidl-default-generators \
  ros-humble-rosidl-default-runtime \
  python3-opencv \
  python3-numpy \
  build-essential \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

# Setup del entorno básico
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec \"$@\"", "--"]
