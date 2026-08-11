# Análisis de una carrera · dashboard

Un solo comando que junta el **bag** (posiciones, telemetría, tiempos por
vuelta) con el/los **logs del algoritmo** (`derrapesLog_*.txt`, uno por cámara)
y saca una página web con todas las gráficas.

```bash
./analisis/run.sh <carpeta_bag> [logs...] [opciones]
```

Cuando imprima la dirección, abrir **http://localhost:8988** y parar con
Ctrl+C. El HTML queda además escrito junto al bag como
`<nombre_bag>_analisis.html`, y es **autocontenido**: funciona con doble clic,
sin servidor y sin ficheros al lado.

Ejemplos:

```bash
# dos cámaras: un log por cámara
./analisis/run.sh PRUEBAS/TRAYECTORIA_2_SIN_TAPAR \
    PRUEBAS/LOGS/derrapesLog_CarControllerNode_camara_01.txt \
    PRUEBAS/LOGS/derrapesLog_CarControllerNode_camara_02.txt

# pasando una carpeta: busca dentro los derrapesLog_*.txt
./analisis/run.sh GRABACIONES/bags/mi_bag PRUEBAS/LOGS/circuito_pequeño

# solo generar el HTML, sin levantar el servidor
./analisis/run.sh PRUEBAS/PRUEBA_MANUAL --sin-servidor
```

Opciones: `--coche car1` (por defecto) y `--sin-servidor`.

Las rutas deben estar **dentro del repositorio**: `run.sh` monta el repo en el
contenedor y traduce las rutas.

---

## Qué hay en cada fichero

| Fichero | Qué hace |
|---|---|
| `analisis.py` | El guion: los argumentos, en qué orden salen las secciones y qué se hace con el resultado. No dibuja ni escribe HTML |
| `lectura_bag.py` | Abre el bag y saca las cuatro series que se usan (posiciones, telemetría, tiempos de vuelta, órdenes de PWM), más las funciones que las cruzan por tiempo |
| `parseo_log.py` | Las regex y el parser del log del algoritmo, y el estado del algoritmo reconstruido vuelta a vuelta |
| `datos.py` | Junta las dos fuentes en los dos DataFrames que alimentan todas las figuras (`df_pos` y `df_tel`) |
| `figuras.py` | Una función por figura, `grafica_<n>_<nombre>`, donde `<n>` es su sección. También la paleta y los offsets del plano global |
| `pagina.py` | Convierte las figuras en las secciones del HTML y las mete en la plantilla |
| `textos.py` | Los títulos y las explicaciones que se leen en la página, todos juntos para poder reescribirlos sin tocar código |
| `servidor.py` | El servidor (`/` y `/pdf`) y la exportación a PDF con kaleido |
| `web/plantilla.html` | El esqueleto de la página, con marcadores `{{...}}` |
| `web/estilos.css` | El estilo. Los colores no se repiten aquí: `pagina.py` mete los de `figuras.py` en las variables `--col-*` |
| `web/panel.js` | Toda la interactividad: los checkboxes de la 2, las flechas ← → de la 3, la sincronización entre las dos y la vuelta que se lleva el botón de PDF |
| `comparativa.py` | Herramienta aparte: superpone los tiempos de varios HTML **ya generados** |
| `Dockerfile` / `run.sh` | La imagen y el lanzador |

### Cómo se genera la página

```
bag ─┐                                       ┌─ web/plantilla.html
     ├─ lectura_bag.py ─┐                    ├─ web/estilos.css
     │                  ├─ datos.py ─ figuras.py ─ pagina.py ─▶ <bag>_analisis.html
logs ┴─ parseo_log.py ──┘                    └─ web/panel.js          │
                                                                servidor.py ─▶ :8988
```

El CSS, el JS y `plotly.js` se **incrustan** en el HTML, no se enlazan: por eso
la página pesa varios MB y por eso funciona con doble clic desde cualquier
sitio. `plotly.js` va una sola vez, en la primera figura del documento.

### Descargas

El único botón es el de **PDF vectorial** (para `\includegraphics` en la
memoria), y solo aparece **con el servidor levantado**: lo genera kaleido, que
está instalado dentro del contenedor. Con `--sin-servidor` la página sale
directamente sin botones, en vez de con botones que no llevan a ningún sitio.

En una sección con slider, el PDF sale con **la vuelta que marque el slider**
en ese momento y con su número en el título; para llevarse tres vueltas hay que
mover el slider y descargar tres veces. Lo que el PDF no respeta son los
*checkboxes* de capas de la sección 2: su estado se queda en el navegador y no
llega al servidor, así que salen todas las capas.

---

## Las secciones

| # | Qué muestra | Controles |
|---|---|---|
| — | Resumen: de dónde salen los datos y qué trae cada log | — |
| 1 | El derrape en 3D sobre la trayectoria (fig. 5.12 de Mario) | slider de vuelta |
| 2 | Las trayectorias de las dos pegatinas, con la base y las perpendiculares (fig. 5.17) | slider + checkboxes de capa |
| 3 | El `dist_derrape` del bag muestra a muestra, con el PWM aplicado; debajo, su resumen por vuelta | slider y **flechas ← →** |
| 4 | El circuito con las zonas de PWM: un color y una forma por zona, que conserva mientras viva | slider de vuelta |
| 5 | La tabla de tiempos y su tendencia; al lado, el derrape frente al tiempo en cajas | — |
| 6 | Cuatro paneles por vuelta: derrapes, tiempo, PWM medio y velocidad media | — |

Las secciones **4 y 6 salen del log**, así que sin logs no aparecen (y la 2 se
queda sin trayectoria base). Las secciones **1, 2, 3 y 5 salen del bag**.

### Cómo se lee la sección 3

Un derrape de verdad es una subida y bajada suave en mitad de una curva, con el
coche bien dentro del campo de una cámara. Un pico estrecho que arranca *justo
en una línea de puntos vertical* (un cambio de cámara) y decae en dos o tres
muestras **no es un derrape**: la pegatina trasera todavía va por detrás del
inicio de la cadena de la cámara que entra, así que `localizar()` la asigna a la
celda 0 y devuelve una distancia *longitudinal*, no perpendicular. La delantera
está filtrada por `max_dist_ruta`; la trasera, no.

### Cómo se lee la sección 4

Color y forma son la **identidad** de una zona: se los queda desde que nace
hasta que muere o la absorbe otra, aunque por el camino la castiguen, y solo
entonces vuelven a la reserva. Así, moviendo el slider, se ve cómo nacen,
crecen, se fusionan y desaparecen.

Dos casos que despistan si no se saben:

- Una zona que **cruza la línea de meta** sale como UNA sola (`69-46 (cruza
  meta)`) aunque el log la liste en dos tramos: la numeración de celdas empieza
  en la meta y es ahí donde el algoritmo corta la partición, pero en la pista es
  el mismo tramo.
- Una zona **partida entre dos cámaras** (una curva que ninguna ve entera, y que
  el algoritmo reparte con el mecanismo de derrame) sale con el mismo color y la
  misma forma en las dos, y la leyenda la marca con `↔`. Cada mitad conserva su
  rango de celdas y su PWM, que pueden diferir porque se ajustan por separado.

---

## Contratos: lo que no se puede cambiar a la ligera

- **El formato del log.** Las regex de `parseo_log.py` son copias de los
  `saveLogFile()` de `AlgoritmoVelocidad.py`. Si allí cambia el texto de una
  línea, los logs ya grabados dejan de leerse. Al añadir un motivo nuevo se crea
  una regex **nueva** en vez de ampliar la existente, para que los logs viejos
  sigan casando igual. La comprobación de que el contrato sigue sano es que
  `Carrera.no_reconocidas` valga 0 tanto con un log nuevo como con uno antiguo.
- **La tabla de tiempos de la sección 5.** `comparativa.py` la lee del HTML ya
  generado (busca `table.tiempos` y las filas `<tr class="..."><td>N</td>
  <td>T.TTT`). Cambiar ese marcado deja sin leer todos los análisis guardados.
- **Los ids `g-trayectorias` y `g-derrape-bag`.** `pagina.py` los pone y
  `web/panel.js` los busca por nombre; están declarados en los dos sitios.

---

## Carreras manuales

Una carrera conducida por una persona (`docker-compose-manual.yml`) se analiza
**exactamente igual**: mismo comando, pasándole también los logs. Dos
diferencias en el resultado:

- La sección **3 no lleva la línea verde de PWM aplicado**: en manual el
  controlador no publica `/carN/pwd` (manda el gatillo de la persona), así que
  al arrancar sale un `AVISO: el bag no contiene ['/car1/pwd']`. Es normal.
- Las secciones **4 y 6 no son lo que pasó, sino lo que el algoritmo HABRÍA
  hecho**: siguió corriendo en paralelo sin mandar nada y dejó en su log el PWM
  de cada fotograma. Ahí está la gracia del modo, se ve dónde el algoritmo va
  por debajo de lo que la persona se atreve a hacer. Con un matiz: es el perfil
  que habría **aprendido mirando conducir al humano** (los derrapes que lo
  alimentan son los de la persona), no el que habría producido conduciendo él.

Sus logs se llaman `derrapesLog_CarControllerManual_*.txt`, así que no pisan los
de una carrera autónoma y se distinguen a simple vista.

### Rampa de PWM

`docker-compose-rampa.yml` (el nodo `RampaPWM.py`) es una carrera manual más,
solo que en vez de conducirla una persona la conduce un script: una **escalera
de PWM** que se mantiene N vueltas en cada valor y va subiendo. Sirve para
comprobar el cronómetro — a PWM constante los tiempos de vuelta tienen que salir
casi iguales, y bajar al subir de escalón.

Su análisis se hace igual, con una diferencia a favor: **sí trae `/car1/pwd`**
(el script publica cada escalón además de mandarlo al Arduino), así que la
sección 3 recupera su línea verde y en ella se ve la escalera dibujada sobre
los tiempos.

### Números de frame

Los `[FRAME n]` del log son el contador de **fotogramas procesados de esa
cámara**, el mismo número que va quemado arriba a la izquierda de sus imágenes
de debug (`camara_XX #n hora`). Por eso las etiquetas del dashboard dicen
"camara_02 frame 1234" y no solo "frame 1234": **los números de dos cámaras no
son comparables entre sí** (cada contador empieza cuando arranca su nodo); para
cruzar cámaras se usa la hora. Detalle completo en `../NUMERACION_FRAMES.md`.

### Multicámara

Cada cámara ve el circuito en sus propios píxeles, así que las gráficas 1, 2 y 4
las colocan en un *plano global* usando la traslación de `OFFSETS_CAMARAS` (en
`figuras.py`). Esos valores se ajustan a mano una vez por montaje hasta que el
circuito quede continuo. El panel de directo (`VisualizacionNode.py`) tiene un
botón "copiar offsets" que los vuelca ya con este formato, para pegarlos ahí.

---

## Comparar varias carreras

`comparativa.py` coge **HTML de análisis ya generados** y saca una sola página
con las vueltas de todas superpuestas: una **contrarreloj a 50 vueltas** (gana
quien menos tarda en darlas, con las salidas de pista contando), más la mejor
vuelta y lo controlado que fue el coche. No necesita ROS ni el contenedor: lee
la tabla de tiempos del propio HTML, así que también lee los análisis antiguos.

```bash
# todas las carreras guardadas de una carpeta (busca los *_analisis.html dentro)
analisis/env/bin/python analisis/comparativa.py BAGS_GUARDAR

# unas cuantas sueltas, renombrando una serie a mano
analisis/env/bin/python analisis/comparativa.py \
    BAGS_GUARDAR/MANUAL_LOIS_COCHE_ROJO \
    "Algoritmo final=BAGS_GUARDAR/ALGO_..._rojo_014_vmin_70_vmax_94_d_9" \
    --salida comparativa_lois_vs_algo.html
```

Saca tres cosas: las **líneas** de tiempo por vuelta (con un ★ en la vuelta más
rápida), las **cajas** de cuartiles (caja estrecha = coche controlado) y la
**tabla** ordenada por el tiempo de la contrarreloj, con mediana y σ.

Reglas de lectura, que la propia página explica:

- Se descarta la **vuelta de calibración**: el controlador la cronometra antes
  de poner el contador a cero, así que toda carrera trae dos vueltas 1 y la
  primera es la lenta de reconocimiento.
- Las **vueltas lentas cuentan** (las que el dashboard marca como >2× la media):
  si el coche se salió y hubo que volver a ponerlo, ese tiempo se perdió y suma,
  igual que en una carrera de verdad.
- Se descartan las **vueltas mal medidas** (por debajo de la mitad de la
  mediana: la meta disparó dos veces y partió una vuelta en dos), avisando
  debajo de la tabla.
- Se comparan las **50 primeras vueltas**: la carrera que dé más se corta ahí y
  la que no llegue sale sin tiempo de contrarreloj.
- **Un panel por coche** (detectado por el nombre): el coche pesa más que el
  piloto, así que mezclarlos en un eje compararía coches, no pilotos.

Por eso la mediana de la comparativa no coincide con la del pie de la tabla del
dashboard, donde entran la calibración y las anómalas.
