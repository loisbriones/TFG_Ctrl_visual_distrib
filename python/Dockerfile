FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Instalamos solo las dependencias fijas
# tzdata: sin la base de zonas horarias, la TZ que pasan los docker-compose no
# se puede resolver y glibc cae a UTC ignorando incluso el /etc/localtime
# montado; con ella las horas de los logs y de las imágenes de debug son las
# de la máquina que ejecuta el contenedor. (DEBIAN_FRONTEND ya está en
# noninteractive arriba, que es lo que evita que tzdata pare a preguntar.)
RUN apt-get update && apt-get install -y \
  ros-humble-rosidl-default-generators \
  ros-humble-rosidl-default-runtime \
  python3-opencv \
  python3-numpy \
  python3-serial \
  tzdata \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

# Setup automático del entorno de ROS
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
