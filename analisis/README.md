# Análisis de una carrera · dashboard unificado

Un solo comando que junta el **bag** (posiciones, tiempos por vuelta,
imágenes de las cámaras) con el/los **logs del algoritmo**
(`derrapesLog_*.txt`) y levanta un dashboard web con todas las gráficas.

```bash
./analisis/run.sh <carpeta_bag> [logs...] [opciones]
```

Cuando imprima la dirección, abrir **http://localhost:8988** y parar con
Ctrl+C. El HTML autocontenido queda además escrito junto al bag como
`<nombre_bag>_analisis.html` (las gráficas funcionan con doble clic, pero
las descargas y el visor de imágenes necesitan el servidor).

Ejemplos:

```bash
# dos cámaras: un log por cámara
./analisis/run.sh PRUEBAS/TRAYECTORIA_2_SIN_TAPAR \
    PRUEBAS/LOGS/ALGO-MOD/circuito_pequeño/derrapesLog_CarControllerNode_camara_01.txt \
    PRUEBAS/LOGS/ALGO-MOD/circuito_pequeño/derrapesLog_CarControllerNode_camara_02.txt

# pasando una carpeta: busca dentro los derrapesLog_*.txt
./analisis/run.sh GRABACIONES/bags/mi_bag PRUEBAS/LOGS/ALGO-MOD/circuito_pequeño

# solo generar el HTML, sin servidor (más rápido: no carga las imágenes)
./analisis/run.sh PRUEBAS/PRUEBA_MANUAL --sin-servidor
```

Opciones: `--coche car1` (por defecto), `--sin-servidor`, `--sin-imagenes`
(arranca mucho más rápido; la sección 7 queda vacía).

Las rutas deben estar **dentro del repositorio**: `run.sh` monta el repo en
el contenedor y traduce las rutas.

## Secciones

| # | Qué muestra | Slider |
|---|---|---|
| 0 | Resumen de la carrera y anomalías detectadas | — |
| 1 | Derrape 3D sobre la trayectoria (fig. 5.12 de Mario) | acumulativo (vueltas 1..v) |
| 2 | Trayectorias delantera y trasera (fig. 5.17) | una vuelta cada vez |
| 3 | Distancia perpendicular de la trasera vs celda (del log) | una vuelta cada vez |
| 3b | `dist_derrape` del bag vs tiempo de vuelta, con la cámara de cada muestra y el PWM aplicado; debajo, el resumen por vuelta (3c) | una vuelta cada vez, **también con ← →** |
| 4 | El circuito con el PWM de cada celda como número | una vuelta cada vez |
| 5 | Tabla de tiempos por vuelta + tendencia | — |
| 6 | Zonas (6a), heatmap de PWM (6b), resumen 2×2 (6c) | — |
| 7 | Visor de imágenes + vídeo mosaico descargable | por imagen del bag |
| 8 | Descarga de todas las tablas en CSV | — |

### Números de frame

Los `[FRAME n]` del log son el contador de **fotogramas procesados de esa
cámara**, el mismo número que va quemado arriba a la izquierda de sus imágenes de
debug (`camara_XX #n hora`). Por eso las etiquetas del dashboard dicen
"camara_02 frame 1234" y no solo "frame 1234": **los números de dos cámaras no
son comparables entre sí** (cada contador empieza cuando arranca su nodo); para
cruzar cámaras se usa la hora. Ojo también con el slider de la sección 7: recorre
el índice de la imagen dentro del bag, que no es el número de frame (solo se
publica imagen de debug de algunos fotogramas). Detalle completo en
`../NUMERACION_FRAMES.md`.

## Descargas

- **PDF vectorial** (para `\includegraphics` en la memoria): botón bajo cada
  gráfica. Sale el estado de la vuelta que marque el slider si se añade
  `&vuelta=N` al enlace; para llevarse 3 vueltas concretas hay que descargar
  las tres a mano, una por una.
- El panel **6c** se puede bajar completo o **cada subplot por separado**.
- **CSV** de los datos de cada gráfica (o todos juntos en la sección 8).
- **Vídeo mp4** mosaico con las cámaras en el orden que se marque en la
  sección 7.

## Estructura

| Fichero | Qué hace |
|---|---|
| `analisis.py` | CLI, ensamblado del HTML y servidor con sus endpoints |
| `parseo_log.py` | regex y parser del log del algoritmo, derivados por vuelta |
| `lectura_bag.py` | lectura del bag (rosbag2_py + respaldo mcap), imágenes, PWM aplicado |
| `figuras.py` | todas las figuras plotly, colores y offsets del plano global |

**Multicámara**: cada cámara ve el circuito en sus propios píxeles, así que
las gráficas 1, 2 y 4 las colocan en un *plano global* usando la traslación
de `OFFSETS_CAMARAS` (en `figuras.py`). Esos valores se ajustan a mano una
vez por montaje hasta que el circuito quede continuo.

`generar_graficas.py` y `analizar_log_algoritmo.py` son los scripts
anteriores (bag → PNG y log → HTML). Este dashboard los sustituye: se
pueden borrar cuando ya no hagan falta.
