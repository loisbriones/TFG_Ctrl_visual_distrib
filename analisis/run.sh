#!/usr/bin/env bash
# Lanza el dashboard unificado de análisis (analisis.py) dentro de un
# contenedor ROS2 Humble y lo sirve en http://localhost:8988.
#
# Uso:
#   ./analisis/run.sh <carpeta_bag> [logs...] [opciones de analisis.py]
#
#   ./analisis/run.sh PRUEBAS/TRAYECTORIA_2_SIN_TAPAR \
#       PRUEBAS/LOGS/ALGO-MOD/circuito_pequeño/derrapesLog_CarControllerNode_camara_01.txt
#   ./analisis/run.sh GRABACIONES/bags/mi_bag PRUEBAS/LOGS/ALGO-MOD/circuito_pequeño
#   ./analisis/run.sh PRUEBAS/PRUEBA_MANUAL --sin-servidor    # solo el HTML
#
# Cuando el script imprima la dirección, abrir http://localhost:8988 en el
# navegador del host y parar con Ctrl+C al terminar. El HTML autocontenido
# queda además escrito junto al bag (<nombre_bag>_analisis.html).
#
# Todas las rutas (bag y logs) deben estar DENTRO del repositorio: el
# contenedor solo monta el repo (en /repo) y traduce las rutas del host.
set -euo pipefail

if [ $# -lt 1 ]; then
    echo "Uso: $0 <carpeta_bag> [logs...] [opciones]" >&2
    exit 1
fi

REPO="$(cd "$(dirname "$0")/.." && pwd)"
IMAGEN=tfg-analisis

# Workspace persistente en el host para no recompilar los mensajes en cada
# ejecución (colcon solo reconstruye si cambian los .msg)
WS="$REPO/analisis/.ws"
mkdir -p "$WS"

# Se construye SIEMPRE: si el Dockerfile no cambió, la caché de Docker lo
# hace instantáneo; si cambió, así la imagen se actualiza sola sin tener
# que borrarla a mano. La salida se silencia salvo error.
docker build -t "$IMAGEN" "$REPO/analisis" >/dev/null

# Traducción de argumentos: los que existan como fichero/carpeta del host se
# convierten a su ruta dentro del contenedor (/repo/...); el resto (--coche,
# --sin-servidor...) pasan tal cual
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
# /repo         el repositorio completo, con escritura: el HTML de salida se
#               escribe junto al bag
# exec python3  para que el Ctrl+C llegue al servidor y no se quede colgado
# -u            sin buffer: el resumen y la dirección del dashboard salen al
#               instante aunque la salida se esté redirigiendo a un fichero
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
