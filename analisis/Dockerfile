FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Lo mismo que la imagen de la arquitectura (python/Dockerfile) más el plugin
# mcap de rosbag2 y las librerías del dashboard:
# - pandas: DataFrames del parser de logs (parseo_log.py)
# - opencv: montar el vídeo mp4 del visor de imágenes (fase 4)
# - matplotlib + tk: SOLO para el script antiguo generar_graficas.py; borrar
#   estas dos líneas cuando se retire
RUN apt-get update && apt-get install -y \
  ros-humble-rosidl-default-generators \
  ros-humble-rosidl-default-runtime \
  ros-humble-rosbag2-storage-mcap \
  python3-numpy \
  python3-pandas \
  python3-opencv \
  python3-matplotlib \
  python3-tk \
  python3-pip \
  && rm -rf /var/lib/apt/lists/*

# - mcap / mcap-ros2-support: el plan B de lectura para bags grabados con
#   definiciones de mensaje que luego cambiaron en el código (el mcap lleva
#   embebida la definición original y con ella siempre se puede decodificar)
# - plotly: todas las figuras del dashboard (figuras.py)
# - kaleido: exportación de las figuras a PDF vectorial en el servidor.
#   FIJADO a 0.2.1 a propósito: desde la 1.0 kaleido dejó de traer su propio
#   navegador y exige un Google Chrome instalado en el sistema (falla con
#   "Kaleido requires Google Chrome to be installed"). La 0.2.1 trae un
#   binario propio y exporta PDF sin instalar nada más. Plotly avisa de que
#   es una versión antigua (DeprecationWarning): es inofensivo. Si algún día
#   plotly deja de admitirla, la alternativa es instalar Chrome en esta
#   imagen (apt + plotly_get_chrome) y quitar el pin.
# - tornado: SOLO para el modo --vueltas del script antiguo; borrar al retirarlo
RUN pip3 install --no-cache-dir mcap mcap-ros2-support plotly \
  "kaleido==0.2.1" tornado

WORKDIR /ros2_ws
