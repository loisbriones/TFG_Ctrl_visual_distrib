FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Instalamos solo las dependencias fijas 
RUN apt-get update && apt-get install -y \
  ros-humble-rosidl-default-generators \
  ros-humble-rosidl-default-runtime \
  python3-opencv \
  python3-numpy \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

# Setup automático del entorno de ROS
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
