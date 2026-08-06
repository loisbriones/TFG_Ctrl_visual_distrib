#!/usr/bin/env bash
# Levanta el panel de la carrera en directo y abre el navegador.
#
# Uso:
#   ./arrancar_visualizacion.sh
#
# Sirve dos paginas en el mismo puerto:
#   /          el panel de la carrera (clasificacion, plano, eventos)
#   /camaras   el mosaico de lo que ve cada camara, para ENCUADRARLAS antes de
#              empezar: se levantan solo los nodos camara, se abre esa pagina y
#              se van colocando hasta que entre todas se vea el circuito entero
#
# Se para con Ctrl+C. Para ver una carrera ya grabada, dejar esto corriendo y
# en otra terminal lanzar la reproduccion del bag en el mismo dominio DDS:
#
#   ROS_DOMAIN_ID=42 ros2 bag play GRABACIONES/bags/<nombre>
#
# (arrancar SIEMPRE este script antes que el play: la linea de meta se publica
# una sola vez, al principio de la reproduccion).
set -euo pipefail

cd "$(dirname "$0")"

PUERTO=8990   # PUERTO en VisualizacionNode.py

# El navegador se abre en segundo plano y con una espera: el contenedor tiene
# que compilar el workspace con colcon antes de que haya nada escuchando en el
# puerto, y una pestaña abierta demasiado pronto se queda en "no se puede
# conectar". El EventSource de la pagina reconecta solo, pero la primera carga
# del HTML no
(sleep 12; xdg-open "http://localhost:$PUERTO" >/dev/null 2>&1 || true) &

echo "Panel   en http://localhost:$PUERTO          (Ctrl+C para parar)"
echo "Cámaras en http://localhost:$PUERTO/camaras"
exec docker compose -f docker-compose-visualizacion.yml up
