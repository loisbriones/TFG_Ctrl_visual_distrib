FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Instalamos dependencias de ROS2 y C++
RUN apt-get update && apt-get install -y \
  build-essential \
  cmake \
  ros-humble-rosidl-default-generators \
  ros-humble-rosidl-default-runtime \
  ros-humble-cv-bridge \
  libopencv-dev \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

