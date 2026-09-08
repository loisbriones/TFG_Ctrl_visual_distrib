# TFG Scalextric · control visual distribuido

Sistema **distribuido** que controla coches de tipo Scalextric de forma autónoma.
Unas cámaras miran la pista, detectan por color dónde está cada coche y mandan
esa posición por la red. Un controlador decide qué potencia hay que aplicar al
coche y se la envía a un Arduino, que es quien alimenta el carril.

Lo interesante es que el sistema **aprende solo**. Da una vuelta despacio para
memorizar el trazado y a partir de ahí va subiendo la velocidad vuelta tras
vuelta. Cuando el coche derrapa en un sitio, baja la velocidad justo en ese
punto.

Está construido sobre **ROS2 Humble** y todo corre en **Docker**.

Continúa dos TFG anteriores de la Universidade da Coruña:

- **Mario López Cea**:
  [github.com/mariolopez15/control_coche_scalextric](https://github.com/mariolopez15/control_coche_scalextric)
- **Adrián Rego Criado**:
  [github.com/Rego523/GEI-TFG](https://github.com/Rego523/GEI-TFG)

Los dos eran aplicaciones que corrían enteras en un solo PC. La principal aportación
es la arquitectura distribuida.

---

## Índice

1. 🏁 [Qué hay en el repositorio](#qué-hay-en-el-repositorio)
   1. 📊 [analisis](#analisis)
   2. 🏆 [Los resultados](#los-resultados)
   3. 🎥 [GRABACIONES](#grabaciones)
   4. 📄 [MEMORIA](#memoria)
   5. 🐍 [python](#python)
2. 🔄 [Cómo funciona una carrera](#cómo-funciona-una-carrera)
3. 🚀 [Los ficheros de lanzamiento](#los-ficheros-de-lanzamiento)
4. ⚙️ [La configuración](#la-configuración)
   1. 🔧 [Parámetros básicos](#parámetros-básicos)
   2. 🎛️ [Parámetros avanzados](#parámetros-avanzados)
5. 🧩 [El código, clase a clase](#el-código-clase-a-clase)
   1. 📷 [CameraNode](#cameranode)
   2. 🎨 [ProcessImage](#processimage)
   3. 🧠 [CarControllerNode](#carcontrollernode)
   4. 📈 [AlgoritmoVelocidad](#algoritmovelocidad)
   5. 🔌 [RaceControllerNode](#racecontrollernode)
   6. ⚡ [ArduinoController](#arduinocontroller)
   7. 📺 [VisualizacionNode](#visualizacionnode)
   8. 📡 [NetProbeNode](#netprobenode)
6. ✉️ [Los mensajes](#los-mensajes)
7. 🐳 [Despliegue](#despliegue)
8. ⚠️ [Detalles que conviene saber](#detalles-que-conviene-saber)

---

## Qué hay en el repositorio

| Carpeta | Qué es |
|---|---|
| `python/` | El sistema. Todos los nodos de ROS2, su configuración y su despliegue |
| `analisis/` | Los scripts que analizan los resultados de las ejecuciones |
| `GRABACIONES/` | La herramienta que graba lo que pasa por la red |
| `MEMORIA/` | El código LaTeX de la memoria del proyecto |
| `RESULTADOS.md` | La tabla de resultados: análisis, vídeo y datos de cada ejecución |

### analisis

Los scripts que se usaron para analizar los resultados obtenidos de las
ejecuciones. Cogen un bag grabado y los logs del algoritmo, los cruzan y sacan
una página web con todas las gráficas de la sesión.

Tiene su propio README con el detalle: [`analisis/README.md`](analisis/README.md).

### Los resultados

De cada ejecución del capítulo de pruebas hay publicadas tres cosas, y
[`RESULTADOS.md`](RESULTADOS.md) las lista todas en una tabla:

- El **análisis** completo e interactivo, que se abre en el navegador desde
  [la web de resultados](https://loisbriones.github.io/TFG_Ctrl_visual_distrib_resultados).
- El **vídeo** de lo que vieron las cámaras, en YouTube.
- Los **datos**, el bag y los logs, en las
  [releases](https://github.com/loisbriones/TFG_Ctrl_visual_distrib/releases),
  para poder rehacer el análisis con la herramienta de `analisis/`.

### GRABACIONES

Los scripts y el código necesarios para lanzar la herramienta de grabación. Usa
`rosbag` para guardar toda la información que se publica por la red en ficheros
`.mcap`, que es lo que después lee la herramienta de análisis.

```bash
./GRABACIONES/arrancar_grabacion.sh <nombre_de_la_grabacion>
```

Graba **todos** los topics del dominio, incluidos los que aparezcan después de
arrancar. Los mensajes propios se copian solos desde `python/` antes de empezar,
porque el bag necesita saber cómo está hecho cada tipo de mensaje para poder
leerlo meses después.

### MEMORIA

El código LaTeX donde está recogida la memoria del proyecto.

### python

```
python/
├── Dockerfile                        La imagen, común a todos los nodos
├── docker-compose-camera.yml         Despliegue de una cámara
├── docker-compose-control.yml        Despliegue de los controladores
├── docker-compose-manual.yml         Los mismos en modo manual, sin Arduino
├── docker-compose-visualizacion.yml  El panel en directo
├── arrancar_visualizacion.sh         Atajo para levantar el panel
└── src/image_processor_pkg/
    ├── config/params.yaml            TODA la configuración
    ├── launch/                       Qué nodos levantar en cada máquina
    ├── msg/                          Los mensajes propios
    ├── web/                          Las páginas del panel
    └── image_processor_pkg/          El código de los nodos
```

---

## Cómo funciona una carrera

Cada coche lleva **dos pegatinas de color**, una delante y otra detrás. La de
delante dice dónde está el coche, porque va siempre guiada por la ranura del
carril. La de detrás dice si **derrapa**: si la cola del coche se separa de la
trayectoria más de la cuenta, es que está perdiendo agarre.

Este es el camino que recorre un frame:

```
┌────────────────┐  /car1/position  ┌───────────────────┐ /car1/pwd ┌───────────────────┐
│   CameraNode   │ ────────────────▶│ CarControllerNode │ ─────────▶│ RaceControllerNode│──▶ Arduino
│  (una cámara)  │                  │  (uno por coche)  │           │  (uno para todos) │
└────────────────┘                  └───────────────────┘           └───────────────────┘
```

Una carrera tiene **dos fases**:

1. **Calibración.** El coche rueda despacio una vuelta entera. Cada cámara va
   apuntando por dónde pasa, y con eso aprende el trazado que ella ve. Al cruzar
   la meta por segunda vez la calibración de ese coche termina sola.
2. **Carrera.** El controlador ya sabe por dónde va el circuito, así que empieza
   a decidir la velocidad y a aprender de los derrapes.

La calibración es **de cada coche**, no del sistema entero. Un coche puede
seguir calibrando mientras otro ya está corriendo.

> En todo el proyecto, "velocidad" quiere decir **valor de PWM**, un número de 0
> a 255 que fija la tensión del carril. No es una velocidad medida en m/s.

### Los cuatro modos

Cada coche tiene un modo, que se elige en `params.yaml`, y decide quién manda:

| Modo | ¿Publica PWM? | De dónde sale el PWM |
|---|---|---|
| `manual` | No, conduce una persona | Del mando físico del Scalextric |
| `incremental` | Sí | Empieza bajo y sube un escalón cada N vueltas |
| `automatico` | Sí | Lo decide el algoritmo. Es el modo normal |
| `politica` | Sí | De un JSON con un perfil ya fijado |

---

## Los ficheros de lanzamiento

Un fichero de lanzamiento (o *launch*) es la forma que tiene ROS2 de decir qué
nodos hay que levantar, con qué parámetros y con qué nombre. Están en
`python/src/image_processor_pkg/launch/`.

| Fichero | Qué levanta |
|---|---|
| `CameraLaunch.py` | Un `CameraNode`. Recibe por argumento el nombre de la cámara y qué webcam abrir |
| `ControlLaunch.py` | Un `CarControllerNode` por cada coche que NO sea manual, el `RaceControllerNode` y, si está activado, el `NetProbeNode` |
| `ManualLaunch.py` | Un `CarControllerNode` por cada coche manual. **No** levanta el puente Arduino |
| `VisualizacionLaunch.py` | El `VisualizacionNode` del panel en directo |

`ControlLaunch` y `ManualLaunch` leen `params.yaml` por su cuenta antes de
arrancar nada, porque cuántos nodos crear depende de cuántos coches haya. Cada
uno se queda con su parte de la lista según el modo, así que para una carrera
mixta se levantan los dos a la vez y no hay que tocar ninguna configuración.

Todos los nodos van con `respawn`, o sea que si uno se cae vuelve solo a los dos
segundos.

---

## La configuración

Todo está en un único fichero,
`python/src/image_processor_pkg/config/params.yaml`, y cada parámetro lleva su
comentario al lado. Aquí van los que se tocan de verdad.

### Parámetros básicos

| Parámetro | Qué hace |
|---|---|
| `coches` | Cuántos coches hay y cómo se llaman. Cada nombre necesita su bloque en `cars` |
| `cars.<coche>.stiker_front` / `stiker_back` | Los colores de sus dos pegatinas |
| `cars.<coche>.carril` | En qué carril está el coche. Tiene que ser el mismo al que está conectado en el Arduino |
| `cars.<coche>.modo` | Modo de operación: cómo se decide qué PWM aplicar |
| `camera.mode` | Resolución del frame que captura la cámara |
| `camera.device` | Qué webcam usar, el número o la ruta |
| `camera.debug` | Si la cámara publica los frames que captura, para depuración |
| `controller.minimum_speed` | PWM con el que empieza el algoritmo |
| `controller.maximum_speed` | PWM máximo que puede alcanzar el algoritmo |
| `controller.algoritmo.umbral_derrape` | Cuánto se tiene que separar la pegatina trasera para decir que derrapa. Más alto, se detecta más tarde |
| `arduino.port` | Puerto serie donde aparece el Arduino |
| `arduino.calibration_speed` | PWM al que rueda el coche durante la vuelta de calibración |

Los **colores** que entiende el sistema son `rojo`, `azul`, `verde`, `naranja`,
`amarillo`, `cian` y `violeta`. Valen tanto para las dos pegatinas de un coche
como para `camera.detection.finish_line_color`, que es el color de la meta.

Y `camera.mode` es un número del 0 al 4: `0` 160×120, `1` 320×240, `2` 640×480,
`3` 800×600 y `4` 1280×720.

### Parámetros avanzados

Los que gobiernan el aprendizaje. Van en píxeles, en celdas o en unidades de
PWM, y el valor que tiene cada uno ahora mismo está en `params.yaml`, con su
comentario al lado.

| Parámetro | Qué pasa si se cambia |
|---|---|
| `controller.distancia_nodos_trayectoria` | Separación mínima entre los puntos que se guardan de la trayectoria |
| `controller.umbral_meta` | Tolerancia del cruce de meta |
| `controller.timeout_camara_activa` | Lo que se espera antes de dar por perdida la cámara que está siguiendo al coche |
| `controller.vueltas_incremento` | Vueltas que aguanta el modo `incremental` antes de subir un escalón de PWM |
| `controller.algoritmo.max_dist_ruta` | Si la pegatina delantera se separa de la trayectoria base más de esto, se descarta el frame |
| `controller.algoritmo.margen_extremo_celdas` | Zona muerta junto a los bordes de lo que ve la cámara, donde no se abren derrapes |
| `controller.algoritmo.paso_celda` | La distancia entre los puntos de la trayectoria base con la que trabaja el algoritmo |
| `controller.algoritmo.umbral_celda_gigante` | Salto a partir del cual se da por hecho que hay un trozo de circuito que la cámara no ve |
| `controller.algoritmo.umbral_cierre` | Distancia para dar por hecho que esa cámara ve el circuito entero |
| `controller.algoritmo.incremento_vuelta` | Cuánto sube el PWM de cada zona al cruzar meta |
| `controller.algoritmo.reduccion_derrape` | Cuánto baja el PWM de una zona al derrapar en ella |
| `controller.algoritmo.retroceso_creacion` | Cuánto antes del derrape empieza a frenar el coche |
| `controller.algoritmo.retroceso_fusion` | Lo mismo, pero cuando vuelve a derrapar donde ya derrapó |
| `controller.algoritmo.margen_fusion_celdas` | Holgura para decidir que un derrape es el mismo de antes |
| `controller.algoritmo.vueltas_proteccion` | Vueltas que una zona no puede subir después de un derrape |

---

## El código, clase a clase

Está todo en `python/src/image_processor_pkg/image_processor_pkg/`.

### CameraNode

**Nodo de ROS2.** Captura frames, busca en ellos las pegatinas de cada coche con
[`ProcessImage`](#processimage) y publica la posición en `/<coche>/position`.
También busca la línea de meta al arrancar y la publica una sola vez.

Publica: `/<coche>/position`, `camara_debug`, `/finish_line_position`.

### ProcessImage

No es un nodo de ROS2. La usa [`CameraNode`](#cameranode) para crear una
instancia por cada coche de la lista `cars` de `params.yaml`.

Convierte los frames que le pasa el [`CameraNode`](#cameranode) a HSV y se queda
con los píxeles que coinciden con el color de cada pegatina. Limpia el ruido,
busca la mancha con el área más grande y devuelve su centro. La misma clase
busca también la pegatina que marca la línea de meta.

### CarControllerNode

**Nodo de ROS2.** Hay uno por coche. Recibe las posiciones que mandan **todas**
las cámaras y decide el PWM que se aplica al coche que controla.

Durante la calibración va guardando por dónde pasa el coche. Al terminarla crea
un [`AlgoritmoVelocidad`](#algoritmovelocidad) **por cada cámara**, porque cada
una tiene su propio sistema de coordenadas y no hay forma de mezclarlas.

También cuenta las vueltas y detecta el cruce de meta.

Escucha `/<coche>/position` y publica `/<coche>/pwd` más la telemetría.

### AlgoritmoVelocidad

Coge el trazado que ve esa cámara y lo parte en celdas de `paso_celda` píxeles.
Esas celdas se agrupan en zonas, y todas las celdas de una zona comparten el
mismo PWM. Al empezar hay una zona única con el valor de PWM mínimo. Hay una
instancia por cada cámara que envía su posición al
[`CarControllerNode`](#carcontrollernode). Las zonas se van creando y uniendo en
función de dónde derrape el coche: ahí se aplica un castigo para reducir el valor
de PWM, y después se incrementa en las zonas donde no hubo derrape.

En la memoria estas celdas se llaman puntos de la trayectoria base, son lo mismo.

Todo lo que hace queda escrito en `logs_carrera/derrapesLog_<nodo>_<camara>.txt`,
que es de donde la herramienta de análisis reconstruye la sesión entera.

### RaceControllerNode

**Nodo de ROS2.** Hay uno solo, en el PC que tiene el Arduino enchufado.
Escucha el `pwd` de cada coche, saca el número de carril y manda la orden por el
puerto serie con [`ArduinoController`](#arduinocontroller).

### ArduinoController

Heredada del TFG de Adrián. Habla con el Arduino por el puerto serie, mandando
órdenes del tipo `r2:87` (carril 2 a PWM 87) y esperando un `OK` de
confirmación.

### VisualizacionNode

**Nodo de ROS2** que escucha topics y muestra la información que se envía en una
página web. Sirve dos páginas en `http://localhost:8990`:

| Página | Qué es |
|---|---|
| `/` | Clasificación, plano del circuito, eventos y salud del sistema |
| `/camaras` | Mosaico con lo que ve cada cámara |

### NetProbeNode

**Nodo de ROS2** que mide la latencia de la red. Manda un echo periódico a cada
cámara, la cámara devuelve la respuesta tal cual y él cronometra lo que tarda en
volver. Crea un CSV con las mediciones de los retardos para su posterior
análisis.

Se puede desactivar con el parámetro `net_probe.enabled` de `params.yaml`.

---

## Los mensajes

Los mensajes propios están en `python/src/image_processor_pkg/msg/`. ROS2 los
compila y genera el código para leerlos y escribirlos.

| Mensaje | Para qué |
|---|---|
| `Point2D` | Un punto en píxeles |
| `Stiker` | Una pegatina detectada |
| `BoundingRect` | El rectángulo que envuelve una pegatina |
| `LineSegment` | Un segmento cualquiera |
| `CarLocation` | **El principal.** La posición de un coche vista por una cámara |
| `FinishLine` | La línea de meta y quién la ve |
| `SpeedCarril` | Una orden de PWM para un carril |
| `TimePerLap` | El tiempo de una vuelta |
| `CarControlTelemetry` | Cómo le va al controlador |
| `NetProbe` / `NetProbeEcho` | El echo de latencia de red y la respuesta de la cámara |

---

## Despliegue

Hace falta **Docker** y nada más. La imagen se construye sola la primera vez.

**1. Las cámaras.** Al arrancar hay que pasarle el `NODE_ID`, que es el nombre
con el que esa cámara se identifica en toda la red, y `CAMERA_DEV`, la webcam
que tiene que abrir:

```bash
NODE_ID=camara_01 CAMERA_DEV=0 docker compose -f docker-compose-camera.yml up
```

**2. Los controladores.** En el PC que tiene el Arduino enchufado. Levanta un
controlador por cada coche que no sea manual, el puente Arduino y el medidor de
red:

```bash
docker compose -f docker-compose-control.yml up
```

**3. Carrera manual.** Los mismos controladores pero sin el puente Arduino,
porque conduce una persona. Arranca aunque el Arduino ni siquiera esté conectado:

```bash
docker compose -f docker-compose-manual.yml up
```

**4. El panel en directo.** Abre el navegador solo:

```bash
./arrancar_visualizacion.sh
```

Los nodos se encuentran entre ellos solos. Solo tienen que estar en la misma red
y con el mismo `ROS_DOMAIN_ID`.

---

## Detalles que conviene saber

Cosas que no se ven leyendo el código y que hacen perder tiempo si no se saben.

**El formato del log del algoritmo es un contrato.** La herramienta de análisis
lee `derrapesLog_*.txt` con expresiones regulares estrictas. Cambiar el texto de
esas líneas rompe el análisis de todas las sesiones que ya estén grabadas.

**Los números de frame son de cada cámara.** Cada cámara empieza a contar desde
cero cuando arranca su nodo, así que el frame 500 de una no tiene nada que ver
con el 500 de otra. Dentro de una cámara se compara por número de frame, y entre
cámaras por marca de tiempo. Ese número es el mismo que va escrito en la imagen
de depuración.

**Las marcas de tiempo nunca cruzan dos relojes.** El tiempo de vuelta se calcula
restando dos marcas de la **misma** Raspberry, y por eso no hace falta
sincronizar los relojes de las máquinas.

**Cómo se mide la latencia de la red.** Por lo mismo de arriba, se mide el viaje
de ida y vuelta y no el de ida sola. En un viaje completo las dos marcas de
tiempo las toma el mismo reloj, así que el desfase entre las dos máquinas se
cancela al restar.

**El algoritmo corre en los cuatro modos.** No estaba pensado así al principio,
los cuatro modos salieron de necesidades que fueron apareciendo durante el
desarrollo. Se dejó que el algoritmo se ejecutara siempre para no tener dos
caminos distintos en el código y para que el análisis fuese el mismo en todos
los casos. Es algo a mejorar: los resultados de algunos modos pueden parecer
raros justo porque el algoritmo estaba corriendo por debajo.

**Las celdas del código son los puntos de la trayectoria de la memoria.** Es
solo un cambio de nombre, está explicado en
[`AlgoritmoVelocidad`](#algoritmovelocidad).

**El topic se llama `pwd` pero lo que lleva es un PWM.** Fue una errata del
principio del proyecto que se quedó y se fue arrastrando. No se ha cambiado
porque el nombre del topic está grabado dentro de todos los bags publicados, y
renombrarlo dejaría sin datos de PWM el análisis de todas las pruebas.

**Donde el código dice "speed" es el valor de PWM.** Pasa con `minimum_speed`,
`maximum_speed`, `calibration_speed`, `v_min`, `v_max` y el mensaje
`SpeedCarril`. Se quedaron así desde el principio y renombrarlos tocaría
demasiados sitios para lo que aportan.

