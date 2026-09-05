FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Lo mismo que la imagen de la arquitectura (python/Dockerfile) mas el plugin
# mcap de rosbag2 (para leer los bags) y pandas, que es con lo que trabajan el
# parser de logs y las figuras
RUN apt-get update && apt-get install -y \
  ros-humble-rosidl-default-generators \
  ros-humble-rosidl-default-runtime \
  ros-humble-rosbag2-storage-mcap \
  python3-numpy \
  python3-pandas \
  python3-pip \
  && rm -rf /var/lib/apt/lists/*

# - mcap / mcap-ros2-support: el plan B de lectura para bags grabados con
#   definiciones de mensaje que luego cambiaron en el codigo (el mcap lleva
#   embebida la definicion original y con ella siempre se puede decodificar)
# - plotly: todas las figuras del dashboard (figuras.py)
# - kaleido: exportacion de las figuras a PDF vectorial en el servidor.
#   Fijado a 0.2.1 a proposito: desde la 1.0 kaleido dejo de traer su propio
#   navegador y exige un Google Chrome instalado en el sistema (falla con
#   "Kaleido requires Google Chrome to be installed"). La 0.2.1 trae un
#   binario propio y exporta PDF sin instalar nada mas. Plotly avisa de que
#   es una version antigua (DeprecationWarning): es inofensivo. Si algun dia
#   plotly deja de admitirla, la alternativa es instalar Chrome en esta
#   imagen (apt + plotly_get_chrome) y quitar el pin
RUN pip3 install --no-cache-dir mcap mcap-ros2-support plotly "kaleido==0.2.1"

WORKDIR /ros2_ws
