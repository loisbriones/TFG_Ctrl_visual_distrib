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
| 3b | `dist_derrape` del bag vs tiempo de vuelta, con la cámara de cada muestra y el PWM aplicado; debajo, el resumen por vuelta (3c) | una vuelta cada vez, **también con ← →** |
| 4 | El circuito con las zonas de PWM: **un color y una forma por zona**, que la zona conserva mientras viva; el PWM y su subida o bajada, en la leyenda | una vuelta cada vez |
| 5 | Tabla de tiempos por vuelta + tendencia | — |
| 6 | Zonas (6a), heatmap de PWM (6b), resumen 2×2 (6c) | — |
| 7 | Visor de imágenes + vídeo mosaico descargable | por imagen del bag |
| 8 | Descarga de todas las tablas en CSV | — |

## Carreras manuales

Una carrera conducida por una persona (`docker-compose-manual.yml`) se analiza
**exactamente igual**: mismo comando, pasándole también los logs. Dos
diferencias en el resultado:

- La sección **3b no lleva la línea verde de PWM aplicado**: en manual el
  controlador no publica `/carN/pwd` (manda el gatillo de la persona), así que
  al arrancar sale un `AVISO: el bag no contiene ['/car1/pwd']`. Es normal.
- Las secciones **4, 6b y 6c no son lo que pasó, sino lo que el algoritmo
  HABRÍA hecho**: siguió corriendo en paralelo sin mandar nada y dejó en su log
  el PWM de cada fotograma. Ahí está la gracia del modo — se ve dónde el
  algoritmo va por debajo de lo que la persona se atreve a hacer. Con matiz:
  es el perfil que habría **aprendido mirando conducir al humano** (los
  derrapes que lo alimentan son los de la persona), no el que habría producido
  conduciendo él.

Sus logs se llaman `derrapesLog_CarControllerNodeManual_camara_XX.txt`, así que
no pisan los de una carrera autónoma y se distinguen a simple vista.

### Rampa de PWM

`docker-compose-rampa.yml` (el nodo `RampaPWM.py`) es una carrera manual más,
solo que en vez de conducirla una persona la conduce un script: una **escalera
de PWM** que se mantiene N vueltas en cada valor y va subiendo. Sirve para
comprobar el cronómetro — a PWM constante los tiempos de vuelta tienen que salir
casi iguales, y bajar al subir de escalón.

Su análisis se hace igual, con una diferencia a favor: **sí trae `/car1/pwd`**
(el script publica cada escalón además de mandarlo al Arduino), así que la
sección 3b recupera su línea verde y en ella se ve la escalera dibujada sobre
los tiempos.

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
  gráfica. Sale **la vuelta que marque el slider** en ese momento, con su
  número en el título; para llevarse 3 vueltas concretas hay que mover el
  slider y descargar tres veces. Lo que el PDF no respeta son los *checkboxes*
  de capas de la sección 2: su estado no llega al servidor y salen las cuatro.
- El panel **6c** se puede bajar completo o **cada subplot por separado**.
- **CSV** de los datos de cada gráfica (o todos juntos en la sección 8).
- **Vídeo mp4** mosaico con las cámaras en el orden que se marque en la
  sección 7.

## Comparar varias carreras

`comparativa.py` coge **HTML de análisis ya generados** y saca una sola página
con las vueltas de todas superpuestas: una **contrarreloj a 50 vueltas** (gana
quien menos tarda en darlas, con las salidas de pista contando), más la mejor
vuelta y lo controlado que fue el coche. No necesita ROS ni el contenedor: lee
la tabla de tiempos del propio HTML.

```bash
# todas las carreras guardadas de una carpeta (busca los *_analisis.html dentro)
ANALISIS/env/bin/python analisis/comparativa.py GRAFICAS_CARLOS/GUARDAR

# unas cuantas sueltas, renombrando una serie a mano
ANALISIS/env/bin/python analisis/comparativa.py \
    GRAFICAS_CARLOS/GUARDAR/MANUAL_LOIS_COCHE_ROJO_CIRCUITO_OVALO_1 \
    "Algoritmo final=GRAFICAS_CARLOS/GUARDAR/ALGO_..._rojo_014_vmin_70_vmax_94_d_9" \
    --salida comparativa_lois_vs_algo.html
```

Saca tres cosas: las **líneas** de tiempo por vuelta (con un ★ en la vuelta más
rápida), las **cajas** de cuartiles (caja estrecha = coche controlado) y la
**tabla** ordenada por el tiempo de la contrarreloj, con mediana y σ.

Reglas de lectura, que la propia página explica:

- Se descarta la **vuelta de calibración**: el controlador la cronometra antes
  de poner el contador a cero, así que toda carrera trae dos vueltas 1 y la
  primera es la lenta de reconocimiento.
- Las **vueltas lentas cuentan** (las que el dashboard marca como &gt;2× la
  media): si el coche se salió y hubo que volver a ponerlo, ese tiempo se perdió
  y suma, igual que en una carrera de verdad.
- Se descartan las **vueltas mal medidas** (por debajo de la mitad de la
  mediana: la meta disparó dos veces y partió una vuelta en dos), avisando
  debajo de la tabla.
- Se comparan las **50 primeras vueltas**: la carrera que dé más se corta ahí y
  la que no llegue sale sin tiempo de contrarreloj.
- **Un panel por coche** (detectado por el nombre): el coche pesa más que el
  piloto, así que mezclarlos en un eje compararía coches, no pilotos.

Por eso la mediana de la comparativa no coincide con la del pie de la tabla
del dashboard, donde entran la calibración y las anómalas.

## Estructura

| Fichero | Qué hace |
|---|---|
| `analisis.py` | CLI, ensamblado del HTML y servidor con sus endpoints |
| `comparativa.py` | Compara los tiempos de varios HTML de análisis en una página |
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
