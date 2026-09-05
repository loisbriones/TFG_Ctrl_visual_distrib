# Análisis de una carrera · dashboard

Junta el **bag** de una carrera (posiciones, telemetría, tiempos por vuelta)
con el/los **logs del algoritmo** (`derrapesLog_*.txt`, uno por cámara) y genera
un dashboard con las gráficas para analizar la carrera.

```bash
./analisis/run.sh <carpeta_bag> [logs...] [opciones]
```

Una vez lanzado, para verlo hay que abrir **http://localhost:8988**, y se para
con Ctrl+C. El HTML queda además escrito junto al bag como
`<nombre_bag>_analisis.html`, y es **autocontenido**.

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

**Opciones disponibles:**

- `--coche carN`: coche que se analiza. Por defecto `car1`.
- `--sin-servidor`: genera el HTML pero no levanta el servidor.

Las rutas deben estar **dentro del repositorio**: `run.sh` monta el repo en el
contenedor y traduce las rutas.

---

## Qué hay en cada fichero

| Fichero | Qué hace |
|---|---|
| `analisis.py` | Recibe los argumentos y va llamando al resto en orden hasta tener el HTML |
| `lectura_bag.py` | Abre el bag y saca las cuatro series que se usan (posiciones, telemetría, tiempos de vuelta, órdenes de PWM) |
| `parseo_log.py` | Lee el log y, con las regex, reconstruye el estado del algoritmo vuelta a vuelta |
| `datos.py` | Junta las dos fuentes en los dos DataFrames que alimentan todas las figuras (`df_pos` y `df_tel`) |
| `figuras.py` | Una función por figura, `grafica_<n>_<nombre>`, donde `<n>` es su sección |
| `pagina.py` | Convierte las figuras en las secciones del HTML y las mete en la plantilla |
| `textos.py` | Los títulos y las explicaciones que se leen en la página, todos juntos para poder reescribirlos sin tocar código |
| `servidor.py` | Sirve el HTML en el puerto 8988 y exporta las figuras a PDF |
| `web/plantilla.html` | El esqueleto de la página, con marcadores `{{...}}` |
| `web/estilos.css` | Recoge el estilo que usa la plantilla |
| `web/panel.js` | Toda la interactividad de la página, los sliders, las flechas y la vuelta que se lleva el botón de PDF |
| `Dockerfile` / `run.sh` | La imagen y el script de lanzamiento |

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

Las gráficas se descargan en PDF, lo genera kaleido y va instalado dentro del
contenedor, así que hace falta tener el servidor levantado. Con la opción
`--sin-servidor` la página sale sin botones.

En una sección con slider, el PDF sale con **la vuelta que marque el slider** en
ese momento y con su número en el título.

---

## Las secciones

| # | Qué muestra | Controles |
|---|---|---|
| — | Resumen: de dónde salen los datos y qué trae cada log | — |
| 1 | El derrape en 3D sobre la trayectoria | slider de vuelta |
| 2 | Las trayectorias de las dos pegatinas, con la base y la distancia de derrape dibujada | slider + checkboxes de capa |
| 3 | El `dist_derrape` del bag muestra a muestra, con el PWM aplicado; debajo, su resumen por vuelta | slider y **flechas ← →** |
| 4 | El circuito con las zonas de PWM: un color y una forma por zona, que conserva mientras viva | slider de vuelta |
| 5 | La tabla de tiempos y una gráfica; al lado, el derrape frente al tiempo en cajas | — |
| 6 | Cuatro paneles por vuelta: derrapes, tiempo, PWM medio y velocidad media | — |

Las secciones **4 y 6 salen del log**, así que sin logs no aparecen (y la 2 se
queda sin trayectoria base). Las secciones **1, 2, 3 y 5 salen del bag**.

---

## Contratos: lo que no se puede cambiar a la ligera

- **El formato del log.** Las regex de `parseo_log.py` son copias de los
  `saveLogFile()` de `AlgoritmoVelocidad.py`. Si allí cambia el texto de una
  línea, los logs ya grabados dejan de leerse. Al añadir un motivo nuevo se crea
  una regex **nueva** en vez de ampliar la existente, para que los logs viejos
  sigan casando igual. La comprobación de que el contrato sigue sano es que
  `Carrera.no_reconocidas` valga 0 tanto con un log nuevo como con uno antiguo.
- **Los ids `g-trayectorias` y `g-derrape-bag`.** Están escritos en `pagina.py`
  y en `panel.js`, así que hay que cambiarlos en los dos sitios.

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
  de cada fotograma.

Sus logs se llaman `derrapesLog_CarControllerManual_*.txt`.

### Números de frame

Los `[FRAME n]` del log son el contador de **fotogramas procesados de esa
cámara**, el mismo número que va quemado arriba a la izquierda de sus imágenes
de debug (`camara_XX #n hora`). Por eso las etiquetas del dashboard dicen
"camara_02 frame 1234" y no solo "frame 1234": **los números de dos cámaras no
son comparables entre sí** (cada contador empieza cuando arranca su nodo); para
cruzar cámaras se usa la hora.

### Multicámara

Cada cámara ve el circuito en sus propios píxeles, así que las gráficas 1, 2 y 4
las colocan en un *plano global* usando la traslación de `OFFSETS_CAMARAS` (en
`figuras.py`). Hay que ajustarlos a mano.
