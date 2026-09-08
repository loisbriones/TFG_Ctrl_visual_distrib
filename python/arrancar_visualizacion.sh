#!/usr/bin/env bash
# Levanta el panel de la carrera en directo y abre el navegador. Se para con
# Ctrl+C. Sirve dos paginas, / con el panel y /camaras con el mosaico para
# encuadrar las camaras antes de empezar (ver README, "El panel en directo").
#
# Uso:  ./arrancar_visualizacion.sh
set -euo pipefail

cd "$(dirname "$0")"

PUERTO=8990   # PUERTO en VisualizacionNode.py

# El navegador se abre con espera porque el contenedor tiene que compilar antes
# de que haya nada escuchando. Una pestana abierta demasiado pronto se queda en
# "no se puede conectar", y la primera carga del HTML no reintenta sola
(sleep 12; xdg-open "http://localhost:$PUERTO" >/dev/null 2>&1 || true) &

echo "Panel   en http://localhost:$PUERTO          (Ctrl+C para parar)"
echo "Cámaras en http://localhost:$PUERTO/camaras"
exec docker compose -f docker-compose-visualizacion.yml up
