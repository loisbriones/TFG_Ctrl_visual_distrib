FROM ros:humble-ros-base

# Instalamos el soporte para formato MCAP (mejor para imagenes)
RUN apt-get update && apt-get install -y \
  ros-humble-rosbag2-storage-mcap \
  python3-colcon-common-extensions \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

# Setup automático del entorno de ROS
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

