#!/usr/bin/env bash
# Lanza el dashboard (analisis.py) dentro de un
# contenedor ROS2 Humble y lo sirve en http://localhost:8988
#
# Uso:
#   ./analisis/run.sh <carpeta_bag> [logs...] [opciones de analisis.py]
#
#   ./analisis/run.sh PRUEBAS/TRAYECTORIA_2_SIN_TAPAR \
#       PRUEBAS/LOGS/ALGO-MOD/circuito_pequeño/derrapesLog_CarControllerNode_camara_01.txt
#   ./analisis/run.sh GRABACIONES/bags/mi_bag PRUEBAS/LOGS/ALGO-MOD/circuito_pequeño
#   ./analisis/run.sh PRUEBAS/PRUEBA_MANUAL --sin-servidor    # solo el HTML
#
# Parar con Ctrl+C. El HTML autocontenido queda ademas escrito junto al bag (<nombre_bag>_analisis.html)
#
# Todas las rutas (bag y logs) deben estar dentro del repositorio
set -euo pipefail

if [ $# -lt 1 ]; then
    echo "Uso: $0 <carpeta_bag> [logs...] [opciones]" >&2
    exit 1
fi

REPO="$(cd "$(dirname "$0")/.." && pwd)"
IMAGEN=tfg-analisis

# Workspace persistente en el host para no recompilar los mensajes en cada ejecucion
WS="$REPO/analisis/.ws"
mkdir -p "$WS"

# Se construye siempre. Si el Dockerfile no cambio, la cache de Docker lo
# hace instantaneo; si cambio, asi la imagen se actualiza sola sin tener
# que borrarla a mano. La salida se silencia salvo error
docker build -t "$IMAGEN" "$REPO/analisis" >/dev/null

ARGS=()
for a in "$@"; do
    if [ -e "$a" ]; then
        abs="$(realpath "$a")"
        case "$abs" in
        "$REPO"/*) ARGS+=("/repo/${abs#"$REPO"/}") ;;
        *)
            echo "ERROR: $a está fuera del repositorio ($REPO)" >&2
            exit 1
            ;;
        esac
    else
        ARGS+=("$a")
    fi
done

# -p 8988:8988  puerto del dashboard (PUERTO en analisis.py)
# exec python3  para que el Ctrl+C llegue al servidor y no se quede colgado
# -u            sin buffer: el resumen y la direccion del dashboard salen al
#               instante aunque la salida se este redirigiendo a un fichero
#               (si no, Python los guarda en el buffer mientras el servidor
#               sigue vivo y no se ve nada)
docker run --rm \
    --user "$(id -u):$(id -g)" \
    -e HOME=/tmp \
    -p 8988:8988 \
    -v "$WS:/ros2_ws" \
    -v "$REPO/python/src:/ros2_ws/src:ro" \
    -v "$REPO/analisis:/analisis:ro" \
    -v "$REPO:/repo" \
    -w /ros2_ws \
    "$IMAGEN" \
    bash -c "source /opt/ros/humble/setup.bash && \
             colcon build --packages-select image_processor_pkg >/dev/null && \
             source install/setup.bash && \
             cd /analisis && \
             exec python3 -u /analisis/analisis.py \"\$@\"" _ "${ARGS[@]}"
