FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
  ros-humble-rosidl-default-generators \
  ros-humble-rosidl-default-runtime \
  python3-opencv \
  python3-numpy \
  python3-serial \
  tzdata \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

# Para no tener que hacer el source a mano al entrar en el contenedor
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
