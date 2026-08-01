#!/bin/bash

# ==============================================================================
# ZONA DE CONFIGURACIÓN — Lo único que hay que tocar para ampliar el sistema
# ==============================================================================

# 1) TOPICS A GRABAR: TODOS. La grabación usa `ros2 bag record --all`, que
#    graba todos los topics del dominio y además sigue descubriendo topics
#    NUEVOS después de arrancar (rosbag2 sondea el grafo cada 100 ms; solo se
#    perdería esa capacidad pasando --no-discovery, que aquí no se usa). Ya no
#    hay lista de topics ni fichero de QoS que mantener: rosbag2 adapta la
#    suscripción de cada topic al QoS que ofrecen sus publicadores (best-effort
#    para los sensor_data, transient_local para /finish_line_position).

# 2) MENSAJES CUSTOM. Se sincronizan automáticamente desde el proyecto
#    principal antes de grabar (si la ruta existe). Para añadir un mensaje
#    nuevo basta con definirlo en el proyecto principal: el CMakeLists de
#    record_pkg compila todos los .msg de la carpeta msg/ automáticamente.
MSG_SOURCE_DIR="/home/lois/TEMP/TFG_Ctrl_visual_distrib/python/src/image_processor_pkg/msg"
MSG_DEST_DIR="src/record_pkg/msg"

# 3) Contenedor del sistema principal (para activar el debug de las cámaras)
CEREBRO_FILTER="cerebro"

# ==============================================================================
# 1. Validación de parámetros de entrada
# ==============================================================================
if [ -z "$1" ]; then
  echo "Error: Debes indicar el nombre de la grabación."
  echo "Uso: ./arrancar_grabacion.sh <nombre_de_la_grabacion> [num_camaras]"
  echo "Ejemplo: ./arrancar_grabacion.sh prueba_02 3"
  exit 1
fi

VARIABLE_GRABACION=$1
# El nº de cámaras ya solo se usa para activar su modo debug (la grabación
# con --all no necesita saber cuántas hay)
NUM_CAMARAS=${2:-2} # Por defecto: 2 cámaras si no se indica lo contrario

echo "--- Configurando entorno para $NUM_CAMARAS cámara(s) ---"

# ==============================================================================
# 2. Sincronización de mensajes con el proyecto principal
# ==============================================================================
if [ -d "$MSG_SOURCE_DIR" ]; then
  echo "Sincronizando mensajes desde $MSG_SOURCE_DIR..."
  cp "$MSG_SOURCE_DIR"/*.msg "$MSG_DEST_DIR"/
else
  echo "Aviso: no se encontró $MSG_SOURCE_DIR, se usan los mensajes locales."
fi

# ==============================================================================
# 3. Activamos el modo debug de las cámaras en el contenedor cerebro
#    (antes de arrancar la grabación, que se queda en primer plano)
# ==============================================================================
CONTAINER_ID=$(docker ps -qf "name=$CEREBRO_FILTER")

if [ -n "$CONTAINER_ID" ]; then
  CMD_CAMARAS=""
  for ((i = 1; i <= NUM_CAMARAS; i++)); do
    ID_CAM=$(printf "%02d" $i)
    CMD_CAMARAS+="ros2 param set /camara_${ID_CAM}/image_processor camera.debug True ; "
  done

  echo "Activando modo debug de cámaras en el contenedor $CONTAINER_ID..."
  docker exec "$CONTAINER_ID" bash -c "
    source /opt/ros/humble/setup.bash ;
    source /ros2_ws/install/setup.bash ;
    $CMD_CAMARAS
  "
  echo "¡Configuración completada con éxito!"
else
  echo "Aviso: no se encontró ningún contenedor '$CEREBRO_FILTER' en ejecución."
  echo "Se grabará igualmente, pero las cámaras no publicarán imágenes de debug."
fi
echo "----------------------------------------"

# ==============================================================================
# 4. Lanzamos la grabación (Ctrl+C para detenerla)
# ==============================================================================
echo "Iniciando grabación de TODOS los topics: $VARIABLE_GRABACION..."

# Exportamos la variable para que la lea Docker Compose
export NOMBRE_GRABACION="$VARIABLE_GRABACION"

docker compose -f recorder.yml up
