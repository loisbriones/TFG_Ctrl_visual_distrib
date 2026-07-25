#!/usr/bin/env python3
"""
Dashboard unificado de análisis de una carrera: UN comando que junta el bag
(posiciones, tiempos por vuelta, imágenes) con el/los logs del algoritmo
(derrapesLog_*.txt, uno por cámara) y genera un HTML interactivo que además
se sirve por web desde el contenedor (puerto 8988).

Sustituye a generar_graficas.py (bag -> PNG matplotlib) y a
analizar_log_algoritmo.py (log -> HTML plotly): reutiliza el parser de logs
(parseo_log.py), la lectura del bag (lectura_bag.py) y las figuras plotly
(figuras.py) de aquellos scripts.

Uso (dentro del contenedor; ver run.sh, que lo lanza con Docker):

  python3 analisis.py <carpeta_bag> [log1.txt log2.txt ...] [--coche car1]

  - <carpeta_bag>: carpeta del bag (metadata.yaml + .mcap) o una carpeta que
    lo contenga (se busca con rglob).
  - logs: ficheros derrapesLog_*.txt (uno por cámara). Si se pasa una
    carpeta se buscan dentro; si no se pasa nada se buscan en <carpeta_bag>.

El HTML se escribe junto a los datos como <nombre_bag>_analisis.html (las
gráficas funcionan abriéndolo con doble clic; los botones de PDF/CSV y el
visor de imágenes necesitan el servidor) y se sirve en http://localhost:8988.

Todos los sliders por vuelta son INSTANTÁNEOS (cada paso enseña SOLO su
vuelta) y con los ejes fijos, para poder comparar vueltas entre sí. Los de la
2 y la 3b van además sincronizados entre ellos.

Secciones del dashboard:
  0   Resumen de la carrera + anomalías detectadas
  1   Derrape 3D sobre la trayectoria
  2   Trayectorias de las dos pegatinas, con la trayectoria base y las
      perpendiculares de derrape (checkboxes para quitar/poner cada capa)
  3b  Distancia de derrape del bag, vuelta a vuelta; 3c su resumen
  4   El circuito con el PWM de cada celda por color (todas las cámaras en
      un plano): un bloque de color es una zona
  5   Tabla de tiempos por vuelta + tendencia y, debajo, la comparativa
      derrape frente a tiempo en cajas por vuelta (5c)
  6   Análisis del algoritmo: heatmap de PWM (6b), resumen por vuelta (6c)
  7   Visor de imágenes de las cámaras + vídeo mosaico descargable
  8   Descarga de todas las tablas en CSV

Endpoints del servidor (los usan los botones del HTML):
  /pdf?fig=<clave>[&vuelta=N]  PDF vectorial de una figura, en el estado de
                               la vuelta indicada (para meter en la memoria)
  /csv?tabla=<clave>           los datos que alimentan una figura
  /frames/<camara>/<i>.jpg     un fotograma del bag
  /video?orden=cam1,cam2       mp4 mosaico con las cámaras en ese orden
"""

import argparse
import html
import io
import re
import sys
import tempfile
import urllib.parse
import warnings
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np
import pandas as pd
import plotly.graph_objects as go

import figuras
from figuras import (
    COL_GRID, COL_SERIE_1, COL_SUPERFICIE, COL_TINTA, COL_TINTA_2, FUENTE,
    PANELES_6C, _layout_base, grafica_1_derrape3d, grafica_2_trayectorias,
    grafica_3b_derrape_bag, grafica_3c_resumen_derrape,
    grafica_4_circuito, grafica_5c_cajas_derrape, grafica_6b_heatmap,
    grafica_6c_resumen, grafica_6c_subplot,
)
from lectura_bag import (
    emparejar_telemetria, encontrar_bag, leer_bag, pwm_en_instantes,
    repartir_por_vueltas,
)
from parseo_log import (
    camara_del_log, derivar_por_vuelta, detectar_anomalias, parsear_log,
    tabla_vueltas,
)

# Puerto del servidor web dentro del contenedor (run.sh lo publica tal cual
# en el host: http://localhost:8988)
PUERTO = 8988

# Una vuelta se marca como anómala en la tabla de tiempos si supera este
# múltiplo de la media (paradas, salidas de pista, relocalizaciones largas)
FACTOR_VUELTA_ANOMALA = 2.0

# Umbral de derrape que dibuja la sección 3b cuando no hay ningún log del
# algoritmo del que leerlo (es el valor por defecto de EstrategiaPerfil)
UMBRAL_DERRAPE_DEF = 8.0

# Ídem para max_dist_ruta: la distancia a la ruta por encima de la cual el
# algoritmo DESCARTA el fotograma entero (detección falsa de la delantera).
# La sección 2 lo usa para no dibujar perpendiculares de frames que el
# controlador ni llegó a mirar.
MAX_DIST_RUTA_DEF = 80.0

# id del <div> de la gráfica 3b: lo necesita el script que mueve la vuelta
# con las flechas del teclado (ver seccion_3b_derrape_bag)
ID_GRAFICA_3B = "g-derrape-bag"

# id del <div> de la gráfica 2: lo necesitan los checkboxes que quitan/ponen
# cada capa (base, delantera, trasera, perpendiculares; ver
# seccion_2_trayectorias) y el script que mantiene su vuelta sincronizada con
# la de la 3b
ID_GRAFICA_2 = "g-trayectorias"

# fps del vídeo mosaico de la sección 7 (--vídeo, no --datos--: es solo para
# revisar las cámaras, no hace falta que sea exacto). Las cámaras no están
# sincronizadas por timestamp en el mosaico: se combinan por ÍNDICE de
# imagen (i-ésima imagen de cada cámara), que es una aproximación razonable
# porque todas capturan a un ritmo similar (~30 Hz); si hace falta más
# precisión, cambiar aquí a un montaje por timestamp más cercano.
# Ese índice NO es el número de frame de la cámara: solo se publica imagen de
# debug de algunos fotogramas. El número de frame va quemado dentro de cada
# imagen y es el que casa con el log del algoritmo (ver NUMERACION_FRAMES.md).
FPS_VIDEO = 20.0
ALTO_VIDEO = 480  # px: todas las cámaras se reescalan a este alto para el mosaico

# Registros globales que rellena main() y que usan los endpoints del
# servidor (/pdf, /csv, /frames, /video): así el servidor no tiene que
# recalcular nada, solo servir lo que ya se generó para el HTML.
FIGURAS = {}   # clave "num" o "num:camara" -> go.Figure (o make_subplots)
TABLAS = {}    # clave "nombre" o "nombre:camara" -> pandas.DataFrame
IMAGENES = {}  # {camara: [(t_ns, bytes jpeg)]}, ordenado por tiempo


# ---------------------------------------------------------------------------
# Carga de los logs del algoritmo (uno por cámara)
# ---------------------------------------------------------------------------
def cargar_logs(rutas):
    """Parsea cada log y calcula sus derivados por vuelta.

    Devuelve {camara: {"c": Carrera, "vueltas": [...], "perfil_vuelta": {...},
    "zonas_ini_vuelta": {...}, "zonas_fin_vuelta": {...}, "tabla": DataFrame,
    "ruta": Path}} ordenado por nombre de cámara."""
    datos = {}
    for ruta in rutas:
        c = parsear_log(ruta)
        if c.frames is None or c.frames.empty:
            print(f"AVISO: {ruta} no contiene líneas [FRAME] válidas (¿formato "
                  "antiguo por tramos/arco?); se ignora este log")
            continue
        vueltas, perfil_vuelta, zonas_ini, zonas_fin = derivar_por_vuelta(c)
        camara = camara_del_log(ruta)
        datos[camara] = {
            "c": c,
            "vueltas": vueltas,
            "perfil_vuelta": perfil_vuelta,
            "zonas_ini_vuelta": zonas_ini,
            "zonas_fin_vuelta": zonas_fin,
            "tabla": tabla_vueltas(c, vueltas, perfil_vuelta),
            "ruta": ruta,
        }
    return dict(sorted(datos.items()))


def buscar_logs(argumentos, carpeta_bag):
    """Convierte los argumentos de logs en una lista de ficheros: cada
    argumento puede ser un .txt o una carpeta donde buscarlos; sin
    argumentos se buscan en la carpeta del bag (y sus padres inmediatos no:
    solo hacia abajo, con rglob)."""
    if not argumentos:
        return sorted(carpeta_bag.rglob("derrapesLog_*.txt"))
    rutas = []
    for arg in argumentos:
        p = Path(arg)
        if p.is_dir():
            rutas += sorted(p.rglob("derrapesLog_*.txt"))
        elif p.is_file():
            rutas.append(p)
        else:
            sys.exit(f"ERROR: no existe {p}")
    return rutas


# ---------------------------------------------------------------------------
# Sección 2: trayectorias, con los checkboxes que enseñan/quitan cada capa
# ---------------------------------------------------------------------------
# La figura enseña a la vez (por defecto) tres capas: la TRAYECTORIA BASE
# aprendida (las celdas del log, la referencia contra la que el algoritmo mide),
# la pegatina DELANTERA y la pegatina TRASERA. Un bloque de checkboxes quita o
# pone cada una para poder aislar lo que interese. Son checkboxes HTML y no un
# `updatemenus` de plotly porque los dos controles tocan lo mismo (`visible`) y
# se pisan: el slider de vueltas volvería a encender una capa que el checkbox
# acaba de apagar. Con JS se RE-APLICA el estado de los checkboxes después de
# cada cambio de vuelta, y ese mismo `gd.__reaplicar` lo reutiliza el script de
# sincronización con la 3b.
#
# Los índices de traza que el JS necesita viajan en `layout.meta` (los pone
# grafica_2_trayectorias): así el HTML no tiene que saber cómo está montada
# la figura. Cada categoría lleva su lista de índices por vuelta (o el índice
# de la estática, para la base) y el de su fantasma de leyenda.
# ---------------------------------------------------------------------------
def seccion_2_trayectorias(fig, hay_base):
    cuerpo = fig.to_html(
        full_html=False, include_plotlyjs=False, div_id=ID_GRAFICA_2,
        config={"displaylogo": False, "responsive": True},
    )
    # Un checkbox por capa. La delantera y la trasera siempre están; la base y
    # sus perpendiculares solo si hay logs (si no, no hay trayectoria base).
    def check(serie, etiqueta):
        return (f'<label><input type="checkbox" data-serie="{serie}" checked> '
                f"{etiqueta}</label>")
    checks = [check("delantera", "Etiq. delantera"),
              check("trasera", "Etiq. trasera")]
    if hay_base:
        checks.insert(0, check("base", "Trayectoria base"))
        checks.append(check("perp", "Dist. derrape"))
    checks_html = (f'<div class="checks-tray" id="checks-tray-2">'
                   f'{"".join(checks)}</div>')

    explica_base = (
        "<p>Las tres capas salen a la vez y los <strong>checkboxes</strong> "
        "quitan o ponen cada una. La <strong>trayectoria base</strong> "
        "(azul) es la aprendida en calibración (las celdas del log), la "
        "referencia contra la que el algoritmo mide de verdad. Los segmentos "
        "rojos (<strong>Dist. derrape</strong>) son esa medida: van de la "
        "trasera al pie de su perpendicular sobre la base, calculados igual que "
        "en <code>localizar()</code>. El valor sale al <strong>pasar el ratón "
        "por la recta</strong> (o por sus extremos), junto al "
        "<code>dist_derrape</code> que el controlador publicó en el bag, que es "
        "con quien hay que contrastarlo; es una <strong>magnitud, sin signo</"
        "strong> (el algoritmo mide cuánto se aparta la trasera, no de qué "
        "lado). Los fotogramas que el algoritmo <strong>descarta</strong> (la "
        "delantera a más de <code>max_dist_ruta</code> de la ruta) no llevan "
        "perpendicular, porque tampoco llegan a producir un "
        "<code>dist_derrape</code>.</p>"
        if hay_base else
        "<p>Sin logs del algoritmo no hay trayectoria base que dibujar, así que "
        "esta vez no salen ni su checkbox ni las perpendiculares de derrape.</p>"
    )
    script = f"""
<script>
(function() {{
  const gd = document.getElementById("{ID_GRAFICA_2}");
  const cont = document.getElementById("checks-tray-2");
  if (!gd || !cont) return;
  function marcado(serie) {{
    const el = cont.querySelector('input[data-serie="' + serie + '"]');
    return el ? el.checked : true;  // capa sin checkbox: se deja como esté
  }}
  // Aplica el estado de los checkboxes a la vuelta ACTIVA. Las capas por
  // vuelta (delantera/trasera/perp) solo tocan la traza de esa vuelta; las
  // demás ya están apagadas por el paso del slider. La base es una estática.
  function reaplicar() {{
    const meta = gd.layout.meta || {{}};
    const slider = (gd.layout.sliders || [])[0];
    const k = (slider && slider.active) || 0;
    const on = [], off = [];
    function aplica(visible, idx) {{
      if (idx === undefined || idx === null || idx < 0) return;
      (visible ? on : off).push(idx);
    }}
    aplica(marcado("base"), meta.idx_base);
    aplica(marcado("base"), meta.idx_fantasma_base);
    aplica(marcado("delantera"), (meta.idx_delantera || [])[k]);
    aplica(marcado("delantera"), meta.idx_fantasma_delantera);
    aplica(marcado("trasera"), (meta.idx_trasera || [])[k]);
    aplica(marcado("trasera"), meta.idx_fantasma_trasera);
    aplica(marcado("perp"), (meta.idx_perp || [])[k]);
    aplica(marcado("perp"), meta.idx_fantasma_perp);
    if (off.length) Plotly.restyle(gd, {{visible: false}}, off);
    if (on.length) Plotly.restyle(gd, {{visible: true}}, on);
  }}
  // El script de sincronización con la 3b lo llama tras mover la vuelta
  gd.__reaplicar = reaplicar;
  cont.querySelectorAll('input[type="checkbox"]').forEach(function(el) {{
    el.addEventListener("change", reaplicar);
  }});
  // Tras cambiar de vuelta el slider reescribe las visibilidades (reactiva
  // todas las capas de la nueva vuelta): hay que volver a imponer lo marcado,
  // en un tick aparte, cuando plotly ya terminó
  gd.on("plotly_sliderchange", function() {{ setTimeout(reaplicar, 0); }});
  reaplicar();
}})();
</script>
"""
    return f"""
<section><h2>2 · Trayectorias de la etiqueta delantera y trasera</h2>
<p>Las pegatinas en el plano global, <strong>una vuelta cada vez</strong>: al
mover el slider se ve cómo cambia el trazado de una vuelta a la siguiente. Los
<strong>ejes están fijos</strong> (mismo rango y escala 1:1 en todas las
vueltas), así que dos vueltas se pueden comparar directamente. Cada punto real
va con su marcador además de la línea: son los puntos con los que trabaja el
controlador. La <strong>estrella ámbar</strong> marca el inicio de la vuelta,
justo después del cruce de meta, y la <strong>flecha</strong> de su lado, el
sentido de la marcha.</p>
{explica_base}
<div class="descargas">{checks_html}</div>
{cuerpo}
{script}
<div class="descargas">{boton_pdf("2", "Descargar gráfica (PDF)")}
{boton_csv("posiciones", "Descargar posiciones (CSV)")}</div>
</section>
"""


# ---------------------------------------------------------------------------
# Sincronización de la vuelta entre la gráfica 2 y la 3b
# ---------------------------------------------------------------------------
# Las dos tienen un slider por vuelta y se miran juntas ("veo algo raro en el
# trazado de la vuelta 12, quiero su distancia de derrape"). Este script las
# ata: mover una mueve la otra al paso con la MISMA etiqueta de vuelta (las
# dos usan el número de vuelta como etiqueta, así que casan directamente).
#
# Va en su propia parte, al final del documento, porque necesita que los dos
# <div> existan ya: un <script> dentro de la sección 2 se ejecutaría cuando el
# de la 3b todavía no se ha creado.
# ---------------------------------------------------------------------------
def script_sincronizar_vueltas():
    return f"""
<script>
(function() {{
  const g2 = document.getElementById("{ID_GRAFICA_2}");
  const g3 = document.getElementById("{ID_GRAFICA_3B}");
  if (!g2 || !g3) return;
  let sincronizando = false;
  function pasoConEtiqueta(gd, etiqueta) {{
    const s = (gd.layout.sliders || [])[0];
    if (!s) return -1;
    for (let i = 0; i < s.steps.length; i++)
      if (String(s.steps[i].label) === String(etiqueta)) return i;
    return -1;
  }}
  function enlazar(origen, destino) {{
    origen.on("plotly_sliderchange", function(ev) {{
      if (sincronizando || !ev.step) return;
      const k = pasoConEtiqueta(destino, ev.step.label);
      const s = (destino.layout.sliders || [])[0];
      if (k < 0 || !s || k === (s.active || 0)) return;
      sincronizando = true;
      // El paso ya lleva su array de visibilidad: se aplica igual que si se
      // hubiera pinchado en él, y además se mueve el propio slider
      Plotly.update(destino, s.steps[k].args[0], {{"sliders[0].active": k}})
        .then(function() {{
          if (destino.__reaplicar) destino.__reaplicar();
          sincronizando = false;
        }});
    }});
  }}
  enlazar(g2, g3);
  enlazar(g3, g2);
}})();
</script>
"""


# ---------------------------------------------------------------------------
# Sección 3b: la distancia de derrape tal como quedó GRABADA en el bag
# ---------------------------------------------------------------------------
# La sección 3 cuenta la distancia perpendicular desde el log del algoritmo
# (por cámara, contra la celda); esta la cuenta desde el bag y contra el
# tiempo de la vuelta, que es lo que hace falta para responder a "¿por qué
# esta vuelta tiene un pico de 60 px si el coche no derrapó?".
# ---------------------------------------------------------------------------
# Cuántas muestras seguidas se consideran "recién entradas" en una cámara.
# Mientras la pegatina TRASERA va todavía por detrás del inicio de la cadena
# de la cámara que acaba de coger el coche, localizar() devuelve la celda más
# cercana (la 0) y una distancia que ya no es perpendicular sino
# LONGITUDINAL: un pico falso que puede abrir un derrape donde no lo hay.
# Con ~30 Hz y los circuitos de las pruebas, la cola termina de entrar en 3
# fotogramas; subirlo marca más muestras como sospechosas.
MUESTRAS_TRAS_CAMBIO_CAMARA = 3


def marcar_tras_cambio_camara(df_tel):
    """Serie booleana paralela a df_tel: True en las primeras
    MUESTRAS_TRAS_CAMBIO_CAMARA muestras de cada tramo de cámara nueva."""
    marca = []
    cam_anterior = None
    desde_cambio = 10 ** 6  # arranque: nadie viene de un cambio
    for fila in df_tel.itertuples():
        if fila.camara and fila.camara != cam_anterior:
            desde_cambio = 0
            cam_anterior = fila.camara
        marca.append(desde_cambio < MUESTRAS_TRAS_CAMBIO_CAMARA)
        desde_cambio += 1
    return pd.Series(marca, index=df_tel.index)


def anomalias_derrape_bag(df_tel, umbral):
    """Avisos sobre la serie dist_derrape del bag para el resumen de la
    cabecera. De momento uno solo, el que explica los picos que no se
    corresponden con ningún derrape real: los que salen justo después de un
    cambio de cámara."""
    sobre = df_tel["dist"] > umbral
    n_sobre = int(sobre.sum())
    if not n_sobre:
        return []
    recien = sobre & marcar_tras_cambio_camara(df_tel)
    n_recien = int(recien.sum())
    if not n_recien:
        return []
    return [
        f"3b: {n_recien} de las {n_sobre} muestras que pasan el umbral de "
        f"derrape ({umbral:.0f} px) están en los {MUESTRAS_TRAS_CAMBIO_CAMARA} "
        f"primeros frames tras un cambio de cámara, y llegan a "
        f"{df_tel.loc[recien, 'dist'].max():.0f} px. Ahí la pegatina trasera "
        f"todavía va por detrás del inicio de la cadena de la cámara que "
        f"entra, así que localizar() devuelve la celda 0 y una distancia "
        f"LONGITUDINAL, no perpendicular: son picos falsos."
    ]


def tabla_derrape_por_vuelta(df_tel, umbral):
    """Un resumen por vuelta de la serie dist_derrape del bag: máximo,
    percentil 95, mediana, cuántas muestras pasan el umbral, cuántas tenían
    derrape abierto y el PWM medio aplicado. Alimenta la figura 3c."""
    filas = []
    for v, g in df_tel.groupby("vuelta"):
        filas.append({
            "vuelta": int(v),
            "n_muestras": len(g),
            "dist_max": g["dist"].max(),
            "dist_p95": g["dist"].quantile(0.95),
            "dist_mediana": g["dist"].median(),
            "n_sobre_umbral": int((g["dist"] > umbral).sum()),
            "n_derrapando": int(g["derrapando"].sum()),
            "pwm_medio": g["pwm"].mean(),
        })
    return pd.DataFrame(filas)


def seccion_3b_derrape_bag(fig_detalle, fig_resumen, primera):
    """El HTML de la sección: la gráfica por vueltas (con el script de las
    flechas) y debajo el resumen de toda la carrera.

    Las flechas ←/→ solo actúan mientras el ratón está ENCIMA de la gráfica:
    así no se roban las teclas al resto de la página (que se recorre con las
    flechas como en cualquier sitio) y no hay que hacer clic en ningún sitio
    para que funcionen."""
    detalle = fig_detalle.to_html(
        full_html=False, include_plotlyjs=primera, div_id=ID_GRAFICA_3B,
        config={"displaylogo": False, "responsive": True},
    )
    resumen = fig_resumen.to_html(
        full_html=False, include_plotlyjs=False,
        config={"displaylogo": False, "responsive": True},
    )
    return f"""
<section><h2>3b · Distancia de derrape del bag, vuelta a vuelta</h2>
<p>El <code>dist_derrape</code> que publicó el controlador
(<code>/telemetria/&lt;coche&gt;/car_control</code>), muestra a muestra, a lo
largo de una vuelta. En gris el fondo con todas las vueltas; cada punto de la
vuelta activa va del color de la <strong>cámara</strong> que envió la posición
que lo originó y las líneas de puntos verticales marcan los cambios de cámara.
El eje derecho (verde) es el PWM que se estaba aplicando en ese instante
(<code>/&lt;coche&gt;/pwd</code>). El nombre de la vuelta en la leyenda lleva su
máximo, el rango de PWM y cuántas muestras pasaron el umbral.</p>
<p>Con el <strong>ratón encima de la gráfica</strong>, las flechas
<strong>←</strong> y <strong>→</strong> cambian de vuelta (también se puede
arrastrar el slider). El eje vertical está fijo para todas las vueltas: así se
comparan de una a otra sin que el autoescalado engañe.</p>
<p><strong>Qué mirar:</strong> un derrape de verdad es una subida y bajada
suave en mitad de una curva, con el coche bien dentro del campo de una cámara.
Un pico estrecho que arranca <em>justo en una línea de puntos</em> (cambio de
cámara) y decae en dos o tres muestras no es un derrape: la pegatina trasera
todavía va por detrás del inicio de la cadena de la cámara que entra, así que
<code>localizar()</code> la asigna a la celda 0 y devuelve una distancia
<em>longitudinal</em>, no perpendicular. La delantera está filtrada por
<code>max_dist_ruta</code>, pero la trasera no.</p>
{detalle}
<script>
(function() {{
  const gd = document.getElementById("{ID_GRAFICA_3B}");
  if (!gd) return;
  let encima = false;
  gd.addEventListener("mouseenter", function() {{ encima = true; }});
  gd.addEventListener("mouseleave", function() {{ encima = false; }});
  document.addEventListener("keydown", function(ev) {{
    if (!encima) return;
    let paso = 0;
    if (ev.key === "ArrowRight") paso = 1;
    else if (ev.key === "ArrowLeft") paso = -1;
    else return;
    ev.preventDefault();  // que la página no se desplace con las flechas
    const slider = gd.layout.sliders[0];
    const actual = slider.active || 0;
    const k = Math.min(slider.steps.length - 1, Math.max(0, actual + paso));
    if (k === actual) return;
    // El paso del slider ya lleva el array de visibilidad de esa vuelta:
    // se aplica igual que si se hubiera pinchado en él
    Plotly.update(gd, slider.steps[k].args[0], {{"sliders[0].active": k}});
  }});
}})();
</script>
<div class="descargas">{boton_pdf("3b", "Descargar gráfica (PDF)")}
{boton_csv("derrape_bag", "Descargar muestras (CSV)")}</div>
{resumen}
<div class="descargas">{boton_pdf("3c", "Descargar resumen (PDF)")}
{boton_csv("derrape_por_vuelta", "Descargar resumen (CSV)")}</div>
</section>
"""


# ---------------------------------------------------------------------------
# Sección 5: tabla de tiempos por vuelta + mini-gráfica de tendencia
# ---------------------------------------------------------------------------
def seccion_5_tiempos(vueltas, df_tel=None, umbral=None):
    """Tabla HTML con todas las vueltas y su tiempo (mejor vuelta resaltada,
    anómalas marcadas) + pie con mejor/media/mediana/peor + mini-gráfica de
    línea al lado para ver la tendencia. Si hay telemetría del bag, debajo de
    esa tendencia va la figura que compara el derrape con el tiempo (5c).
    Devuelve el HTML de la sección completa."""
    if not vueltas:
        return ("<section><h2>5 · Tiempos por vuelta</h2>"
                "<p>El bag no contiene mensajes de time_per_lap.</p></section>")

    tiempos = [v["tiempo"] for v in vueltas]
    mejor = min(tiempos)
    media = sum(tiempos) / len(tiempos)
    peor = max(tiempos)
    # La MEDIANA es la referencia de la gráfica: la media se la lleva cualquier
    # vuelta anómala (una parada de 30 s sube la referencia de todas), mientras
    # que la mediana dice cómo iba el coche de verdad. La media se queda en la
    # tabla, para poder comparar las dos y ver de un vistazo si hay anómalas.
    ordenados = sorted(tiempos)
    mitad = len(ordenados) // 2
    mediana = (ordenados[mitad] if len(ordenados) % 2
               else (ordenados[mitad - 1] + ordenados[mitad]) / 2)

    filas = []
    for v in vueltas:
        clases = []
        nota = ""
        if v["tiempo"] == mejor:
            clases.append("mejor")
            nota = " ★ mejor"
        if v["tiempo"] > FACTOR_VUELTA_ANOMALA * media:
            clases.append("anomala")
            nota = f" ⚠ &gt;{FACTOR_VUELTA_ANOMALA:.0f}× media"
        filas.append(
            f'<tr class="{" ".join(clases)}"><td>{v["numero"]}</td>'
            f'<td>{v["tiempo"]:.3f}{nota}</td></tr>'
        )
    tabla_html = (
        '<table class="tiempos"><thead><tr><th>vuelta</th><th>tiempo (s)</th>'
        "</tr></thead><tbody>" + "".join(filas) + "</tbody>"
        f'<tfoot><tr><td>mejor</td><td>{mejor:.3f}</td></tr>'
        f"<tr><td>media</td><td>{media:.3f}</td></tr>"
        f"<tr><td>mediana</td><td>{mediana:.3f}</td></tr>"
        f"<tr><td>peor</td><td>{peor:.3f}</td></tr></tfoot></table>"
    )

    fig = go.Figure(
        go.Scatter(
            x=[v["numero"] for v in vueltas], y=tiempos, mode="lines+markers",
            line=dict(color=COL_SERIE_1, width=2), marker=dict(size=6),
            hovertemplate="v%{x}: %{y:.3f} s<extra></extra>",
        )
    )
    fig.add_hline(y=mediana, line=dict(color=COL_GRID, width=1, dash="dash"),
                  annotation_text=f"mediana {mediana:.2f} s",
                  annotation_font_color=COL_TINTA_2)
    fig.update_xaxes(title_text="vuelta", dtick=5)
    # Sin rangemode="tozero" (como el panel de tiempos de la 6c): arrancando en
    # 0 un circuito de ~6 s sale como una línea plana y no se aprecia que una
    # vuelta suba o baje unas décimas, que es justo lo que se viene a mirar
    fig.update_yaxes(title_text="tiempo (s)")
    _layout_base(fig, "Tendencia del tiempo por vuelta", 420)
    fig.update_layout(margin=dict(l=60, r=20, t=60, b=50))
    mini = fig.to_html(full_html=False, include_plotlyjs=False,
                       config={"displaylogo": False, "responsive": True})

    # La mini-gráfica también se puede sacar a PDF y los tiempos a CSV
    FIGURAS["5"] = fig
    TABLAS["tiempos"] = pd.DataFrame(vueltas)

    # --- Comparativa derrape <-> tiempo (5c) -------------------------------
    # Solo si el bag trae telemetría del controlador; sin ella no hay
    # dist_derrape que comparar (mismo criterio que la sección 3b). Va JUSTO
    # DEBAJO de la gráfica de tendencia, en la misma columna: las dos tienen
    # el mismo eje X (la vuelta) y se leen una encima de otra, comparando el
    # tiempo con el derrape de esa misma vuelta sin mover la vista.
    comparativa = ""
    if df_tel is not None and len(df_tel):
        tiempos_por_vuelta = {v["numero"]: v["tiempo"] for v in vueltas}
        FIGURAS["5c"] = grafica_5c_cajas_derrape(
            df_tel, tiempos_por_vuelta, umbral)
        comparativa = (
            "<p><strong>Derrape frente a tiempo.</strong> Enseña el "
            "compromiso del algoritmo: al subir el PWM el coche se aparta más "
            "de la trayectoria (más <code>dist_derrape</code>) y el tiempo "
            "baja, hasta que el derrape pasa del umbral y el tiempo vuelve a "
            "<em>empeorar</em>. Cada caja son los cuartiles del derrape de esa "
            "vuelta (los valores atípicos van sueltos en rojo) y la línea "
            "naranja, su tiempo en el eje derecho.</p>"
            + FIGURAS["5c"].to_html(
                full_html=False, include_plotlyjs=False,
                config={"displaylogo": False, "responsive": True})
            + f'<div class="descargas">{boton_pdf("5c")}</div>')

    return (
        "<section><h2>5 · Tiempos por vuelta</h2>"
        "<p>Tiempos medidos por el controlador al cruzar la línea de meta "
        "(topic time_per_lap del bag). La mejor vuelta va resaltada; una "
        f"vuelta que supere {FACTOR_VUELTA_ANOMALA:.0f}× la media se marca "
        "como anómala (parada, salida de pista, relocalización...). La línea "
        "de puntos de la gráfica es la <strong>mediana</strong>, no la media: "
        "una sola vuelta anómala desplaza la media y deja de servir de "
        "referencia (las dos están en el pie de la tabla).</p>"
        f'<div class="fila-tiempos"><div>{tabla_html}</div>'
        f'<div class="mini-grafica">{mini}'
        f'<div class="descargas">{boton_pdf("5", "Descargar gráfica (PDF)")}'
        f'{boton_csv("tiempos", "Descargar tiempos (CSV)")}</div>'
        f"{comparativa}</div></div></section>"
    )


# ---------------------------------------------------------------------------
# Ensamblado del HTML
# ---------------------------------------------------------------------------
def boton_pdf(clave, etiqueta="Descargar PDF"):
    """Enlace al endpoint /pdf del servidor para una figura ya registrada en
    FIGURAS. El PDF se genera con kaleido (vectorial, apto para
    \\includegraphics). Si el HTML se abre con doble clic (sin servidor) el
    enlace no lleva a ningún sitio: por eso lleva la clase 'solo-servidor',
    que el aviso de la cabecera explica."""
    return (f'<a class="boton solo-servidor" href="/pdf?fig='
            f'{urllib.parse.quote(clave)}" download>{html.escape(etiqueta)}</a>')


def boton_csv(clave, etiqueta="Descargar CSV"):
    """Igual que boton_pdf pero para los datos que alimentan la figura."""
    return (f'<a class="boton solo-servidor" href="/csv?tabla='
            f'{urllib.parse.quote(clave)}" download>{html.escape(etiqueta)}</a>')


def seccion(titulo, parrafo, figs, primera=False, claves=None, csv=None):
    """Una sección del HTML: título, párrafo explicativo y una o varias
    figuras plotly (una por cámara en las secciones por-cámara), cada una
    con su botón de descarga a PDF. Solo la primera figura del documento
    embebe plotly.js (~3 MB); el resto lo reutiliza.

    claves: lista paralela a figs con la clave con la que cada figura quedó
    registrada en FIGURAS (la que usa el endpoint /pdf).
    csv: clave de TABLAS con los datos de la sección (botón de CSV)."""
    if not isinstance(figs, list):
        figs = [figs]
    if claves is None:
        claves = [None] * len(figs)
    cuerpos = []
    for i, (fig, clave) in enumerate(zip(figs, claves)):
        cuerpos.append(fig.to_html(
            full_html=False, include_plotlyjs=primera and i == 0,
            config={"displaylogo": False, "responsive": True},
        ))
        if clave:
            cuerpos.append(f'<div class="descargas">{boton_pdf(clave)}</div>')
    descarga_csv = (f'<div class="descargas">{boton_csv(csv, "Descargar datos (CSV)")}'
                    f"</div>" if csv else "")
    return (
        f"<section><h2>{html.escape(titulo)}</h2>"
        f"<p>{parrafo}</p>{''.join(cuerpos)}{descarga_csv}</section>"
    )


# ---------------------------------------------------------------------------
# Sección 7: visor de imágenes de las cámaras
# ---------------------------------------------------------------------------
# Los JPEG NO se embeben en el HTML (son cientos de MB): el visor los pide
# al servidor uno a uno (/frames/<camara>/<indice>.jpg) según se mueve el
# slider. Con varias cámaras se muestran en una rejilla; el ORDEN de la
# rejilla lo elige el usuario con los campos de la interfaz y es el que se
# usa para montar el vídeo mp4 descargable (/video?orden=...).
# ---------------------------------------------------------------------------
def seccion_7_visor(imagenes, t0_ns):
    if not imagenes:
        return ("<section><h2>7 · Visor de imágenes de las cámaras</h2>"
                "<p>El bag no contiene imágenes de debug "
                "(<code>/camara_XX/camara_debug</code>). Se graban solo si el "
                "parámetro <code>camera.debug</code> estaba activo: "
                "<code>arrancar_grabacion.sh</code> lo activa solo.</p>"
                "</section>")

    camaras = sorted(imagenes)
    n_frames = {cam: len(fs) for cam, fs in imagenes.items()}
    # El slider recorre el índice de frame; el máximo es el de la cámara con
    # más frames (las que tengan menos se quedan en su último frame)
    n_max = max(n_frames.values())

    # Tiempos (s desde el inicio del bag) de cada frame, para la etiqueta
    tiempos_js = "{" + ",".join(
        f'"{cam}":[' + ",".join(f"{(t - t0_ns) / 1e9:.2f}" for t, _ in imagenes[cam])
        + "]" for cam in camaras
    ) + "}"

    celdas = "".join(
        f'<figure class="visor-camara" data-camara="{html.escape(cam)}">'
        f'<img id="img-{html.escape(cam)}" alt="{html.escape(cam)}">'
        f'<figcaption>{html.escape(cam)} · '
        f'<span id="cap-{html.escape(cam)}"></span></figcaption></figure>'
        for cam in camaras
    )
    opciones_orden = "".join(
        f'<label><input type="checkbox" class="orden-cam" value="{html.escape(cam)}" '
        f"checked> {html.escape(cam)}</label>" for cam in camaras
    )

    return f"""
<section><h2>7 · Visor de imágenes de las cámaras</h2>
<p>Los fotogramas de depuración que grabó cada cámara en el bag
({', '.join(f'{html.escape(c)}: {n_frames[c]}' for c in camaras)} imágenes).
Las imágenes se piden al servidor según se mueve el slider (no van dentro
del HTML: pesarían cientos de MB). Las cámaras se combinan por índice de
imagen, no por timestamp exacto.</p>
<p><b>El índice del slider NO es el número de frame.</b> Cada cámara solo
publica imagen de debug de algunos de los fotogramas que procesa, así que la
i-ésima imagen del bag no es el frame i. El número bueno es el que va quemado
arriba a la izquierda de cada imagen (<code>camara_XX #N hora</code>): ese es el
que aparece en las líneas <code>[FRAME N]</code> del log de esa cámara. Los
números de dos cámaras distintas no son comparables entre sí: para cruzarlas se
usa la hora.</p>
<div class="visor">
  <div class="visor-controles">
    <button id="visor-ant">◀ anterior</button>
    <input type="range" id="visor-slider" min="0" max="{n_max - 1}" value="0">
    <button id="visor-sig">siguiente ▶</button>
    <span id="visor-pos">imagen 0 / {n_max - 1}</span>
  </div>
  <div class="visor-rejilla">{celdas}</div>
  <div class="visor-descargas">
    <span>Vídeo mosaico · cámaras y orden:</span>
    {opciones_orden}
    <a class="boton solo-servidor" id="visor-video" href="/video" download>
      Descargar vídeo (mp4)</a>
    <a class="boton solo-servidor" id="visor-frame" href="#" download>
      Descargar este frame</a>
  </div>
</div>
<script>
(function() {{
  const tiempos = {tiempos_js};
  const camaras = {list(camaras)!r}.map(String);
  const slider = document.getElementById("visor-slider");
  const pos = document.getElementById("visor-pos");
  function pintar() {{
    const i = parseInt(slider.value, 10);
    pos.textContent = "imagen " + i + " / " + slider.max;
    for (const cam of camaras) {{
      // Cada cámara tiene su propio número de imágenes: si esta se queda
      // corta se muestra su última imagen disponible. Es un índice dentro
      // del bag, no el nº de frame (ese va quemado en la propia imagen)
      const ult = tiempos[cam].length - 1;
      const j = Math.min(i, ult);
      document.getElementById("img-" + cam).src =
        "/frames/" + encodeURIComponent(cam) + "/" + j + ".jpg";
      document.getElementById("cap-" + cam).textContent =
        "imagen " + j + " · t = " + tiempos[cam][j] + " s";
    }}
    // La descarga del frame actual usa la primera cámara marcada
    const marcada = document.querySelector(".orden-cam:checked");
    if (marcada) {{
      const ult = tiempos[marcada.value].length - 1;
      document.getElementById("visor-frame").href =
        "/frames/" + encodeURIComponent(marcada.value) + "/" +
        Math.min(i, ult) + ".jpg";
    }}
  }}
  slider.addEventListener("input", pintar);
  document.getElementById("visor-ant").addEventListener("click", () => {{
    slider.value = Math.max(0, parseInt(slider.value, 10) - 1); pintar();
  }});
  document.getElementById("visor-sig").addEventListener("click", () => {{
    slider.value = Math.min(parseInt(slider.max, 10),
                            parseInt(slider.value, 10) + 1); pintar();
  }});
  // El enlace del vídeo lleva el orden de cámaras marcado en ese momento
  function actualizarVideo() {{
    const orden = Array.from(document.querySelectorAll(".orden-cam:checked"))
                       .map(c => c.value);
    document.getElementById("visor-video").href =
      "/video?orden=" + encodeURIComponent(orden.join(","));
  }}
  document.querySelectorAll(".orden-cam").forEach(
    c => c.addEventListener("change", () => {{ actualizarVideo(); pintar(); }}));
  actualizarVideo();
  pintar();
}})();
</script>
</section>
"""


def etiquetar_camara(fig, camara, varias):
    """Con varias cámaras, añade el nombre de la cámara al título de su
    figura (las secciones 3 y 6 llevan una figura por cámara)."""
    if varias:
        fig.update_layout(title_text=fig.layout.title.text + f" — {camara}")
    return fig


def ensamblar_html(nombre, resumen_html, partes):
    """La página completa, con el mismo estilo que el HTML del analizador de
    logs (superficie clara, tinta oscura, secciones numeradas)."""
    return f"""<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="utf-8">
<title>Análisis {html.escape(nombre)}</title>
<style>
  :root {{ color-scheme: light; }}
  body {{
    font-family: {FUENTE};
    background: {COL_SUPERFICIE}; color: {COL_TINTA};
    max-width: 1250px; margin: 0 auto; padding: 24px;
  }}
  h1 {{ font-size: 22px; }}
  h2 {{ font-size: 17px; margin-top: 40px; border-bottom: 1px solid {COL_GRID};
       padding-bottom: 6px; }}
  h3 {{ font-size: 14px; color: {COL_TINTA_2}; }}
  p  {{ color: {COL_TINTA_2}; font-size: 13px; line-height: 1.5; }}
  ul {{ color: {COL_TINTA_2}; font-size: 13px; }}
  ul.avisos li {{ margin-bottom: 6px; }}
  code {{ background: {COL_GRID}; padding: 1px 5px; border-radius: 3px; }}
  table.tiempos {{ border-collapse: collapse; font-size: 13px; }}
  table.tiempos th, table.tiempos td {{
    border: 1px solid {COL_GRID}; padding: 3px 12px; text-align: right; }}
  table.tiempos tfoot td {{ font-weight: 600; }}
  tr.mejor td {{ background: #e4f2e4; font-weight: 600; }}
  tr.anomala td {{ background: #f9e9e2; }}
  .fila-tiempos {{ display: flex; gap: 32px; align-items: flex-start;
                   flex-wrap: wrap; }}
  .fila-tiempos .mini-grafica {{ flex: 1 1 500px; min-width: 380px; }}
  a.boton {{
    display: inline-block; font-size: 12px; text-decoration: none;
    color: {COL_TINTA}; background: {COL_GRID}; border-radius: 4px;
    padding: 4px 12px; margin: 4px 8px 4px 0; }}
  a.boton:hover {{ background: #d2d1c8; }}
  .checks-tray {{
    display: inline-flex; flex-wrap: wrap; gap: 6px 14px; align-items: center;
    font-size: 12px; color: {COL_TINTA}; }}
  .checks-tray label {{
    display: inline-flex; align-items: center; gap: 5px; cursor: pointer;
    background: {COL_GRID}; border: 1px solid #d2d1c8; border-radius: 4px;
    padding: 3px 10px; }}
  .checks-tray label:hover {{ background: #d2d1c8; }}
  .checks-tray input[type=checkbox] {{ cursor: pointer; margin: 0; }}
  .descargas {{ margin: 4px 0 18px; }}
  .aviso-servidor {{
    background: #fdf6e3; border-left: 3px solid {COL_TINTA_2};
    padding: 8px 14px; font-size: 12px; color: {COL_TINTA_2}; }}
  .visor-controles {{ display: flex; gap: 12px; align-items: center;
                      margin-bottom: 12px; font-size: 12px; }}
  .visor-controles input[type=range] {{ flex: 1; }}
  .visor-rejilla {{ display: flex; flex-wrap: wrap; gap: 12px; }}
  .visor-camara {{ margin: 0; flex: 1 1 400px; }}
  .visor-camara img {{ width: 100%; background: {COL_GRID};
                       border: 1px solid {COL_GRID}; }}
  .visor-camara figcaption {{ font-size: 12px; color: {COL_TINTA_2};
                              margin-top: 4px; }}
  .visor-descargas {{ margin-top: 14px; font-size: 12px; color: {COL_TINTA_2};
                      display: flex; gap: 12px; align-items: center;
                      flex-wrap: wrap; }}
  ul.lista-csv {{ list-style: none; padding: 0; display: flex;
                  flex-wrap: wrap; gap: 4px; }}
</style>
</head>
<body>
<h1>Análisis de la carrera · {html.escape(nombre)}</h1>
<p class="aviso-servidor">Las gráficas funcionan abriendo este fichero con
doble clic, pero los botones de <strong>descarga (PDF / CSV)</strong> y el
<strong>visor de imágenes</strong> necesitan el servidor: lanzar
<code>./analisis/run.sh &lt;bag&gt; &lt;logs&gt;</code> y abrir
<code>http://localhost:{PUERTO}</code>.</p>
{resumen_html}
{''.join(partes)}
</body>
</html>
"""


# ---------------------------------------------------------------------------
# Exportación a PDF (kaleido) y a CSV
# ---------------------------------------------------------------------------
def figura_a_pdf(fig):
    """PDF vectorial de una figura, listo para \\includegraphics.

    Se quita el slider antes de exportar: kaleido dibuja el estado ACTIVO de
    la figura, y el slider solo estorbaría en el papel. Para elegir qué
    vuelta sale en el PDF se pasa &vuelta=N al endpoint, que activa las
    trazas de esa vuelta antes de llamar aquí (ver _activar_vuelta)."""
    copia = go.Figure(fig)
    # Fuera el slider y los menús: kaleido dibuja el estado ACTIVO de la
    # figura y esos controles solo estorbarían en el papel. Se asignan
    # directamente sobre layout porque update_layout(sliders=[]) no vacía la
    # tupla que ya tiene la figura.
    copia.layout.sliders = ()
    copia.layout.updatemenus = ()
    # El tamaño en puntos del PDF sale del width/height del layout; las
    # figuras solo fijan el alto (_layout_base), así que se da un ancho fijo
    # para que kaleido no use su valor por defecto (700 px, muy estrecho)
    if copia.layout.width is None:
        copia.layout.width = 1100
    with warnings.catch_warnings():
        # El Dockerfile fija kaleido a la 0.2.1 a propósito (la 1.x exige un
        # Chrome instalado): plotly avisa de que es antigua en CADA
        # exportación y llenaría la consola del servidor
        warnings.simplefilter("ignore", DeprecationWarning)
        return copia.to_image(format="pdf")


def _activar_vuelta(fig, vuelta):
    """Deja la figura en el estado de la vuelta pedida aplicando el paso del
    slider correspondiente (los pasos llevan el array de visibilidad; ver
    los sliders de figuras.py). Devuelve una copia; si la figura no tiene
    slider o la vuelta no existe, la copia va tal cual."""
    copia = go.Figure(fig)
    if not copia.layout.sliders:
        return copia
    pasos = copia.layout.sliders[0].steps
    for paso in pasos:
        if paso.label != str(vuelta):
            continue
        # Los pasos "update" llevan {"visible": [...]}; los "animate" (que
        # usa la figura por frames) no se pueden aplicar sin ejecutar JS, y
        # esas figuras ya no se usan en el dashboard
        args = paso.args or []
        if args and isinstance(args[0], dict) and "visible" in args[0]:
            for traza, visible in zip(copia.data, args[0]["visible"]):
                traza.visible = visible
        break
    return copia


def tabla_a_csv(df: pd.DataFrame) -> bytes:
    """CSV UTF-8 con BOM: así LibreOffice y Excel abren bien los acentos."""
    buffer = io.StringIO()
    df.to_csv(buffer, index=False)
    return buffer.getvalue().encode("utf-8-sig")


def montar_video(camaras_orden):
    """Monta un mp4 con los frames de las cámaras indicadas, una al lado de
    otra (mosaico horizontal), combinándolas por índice de frame.

    Devuelve los bytes del mp4. Se usa OpenCV: cada JPEG se decodifica, se
    reescala a ALTO_VIDEO px de alto (manteniendo proporción) y se pegan
    horizontalmente. Una cámara que se quede sin frames repite el último."""
    import cv2  # import local: solo hace falta al pedir el vídeo

    if not camaras_orden:
        raise ValueError("no se indicó ninguna cámara")
    listas = [IMAGENES[cam] for cam in camaras_orden]
    n = max(len(lista) for lista in listas)

    def frame_mosaico(i):
        trozos = []
        for lista in listas:
            _, jpeg = lista[min(i, len(lista) - 1)]
            img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                continue
            alto, ancho = img.shape[:2]
            escala = ALTO_VIDEO / alto
            trozos.append(cv2.resize(img, (int(ancho * escala), ALTO_VIDEO)))
        return cv2.hconcat(trozos) if trozos else None

    primero = frame_mosaico(0)
    if primero is None:
        raise ValueError("no se pudo decodificar ningún frame")
    alto, ancho = primero.shape[:2]

    # VideoWriter necesita escribir a un fichero: se usa uno temporal y se
    # devuelven sus bytes (los vídeos de una prueba caben de sobra en disco)
    with tempfile.NamedTemporaryFile(suffix=".mp4") as tmp:
        escritor = cv2.VideoWriter(
            tmp.name, cv2.VideoWriter_fourcc(*"mp4v"), FPS_VIDEO, (ancho, alto))
        for i in range(n):
            fotograma = frame_mosaico(i)
            if fotograma is not None:
                escritor.write(fotograma)
        escritor.release()
        return Path(tmp.name).read_bytes()


# ---------------------------------------------------------------------------
# Servidor web: sirve el dashboard y sus descargas en el puerto 8988.
#   /                        la página del dashboard
#   /pdf?fig=<clave>[&vuelta=N]   PDF vectorial de una figura
#   /csv?tabla=<clave>       datos que alimentan una figura
#   /frames/<camara>/<i>.jpg un fotograma del bag (visor de la sección 7)
#   /video?orden=cam1,cam2   mp4 mosaico con las cámaras en ese orden
# ---------------------------------------------------------------------------
def servir(pagina_bytes):
    class Manejador(BaseHTTPRequestHandler):
        def _enviar(self, cuerpo, tipo, descarga=None):
            self.send_response(200)
            self.send_header("Content-Type", tipo)
            self.send_header("Content-Length", str(len(cuerpo)))
            if descarga:
                self.send_header("Content-Disposition",
                                 f'attachment; filename="{descarga}"')
            self.end_headers()
            self.wfile.write(cuerpo)

        def do_GET(self):
            partes = urllib.parse.urlparse(self.path)
            consulta = urllib.parse.parse_qs(partes.query)
            ruta = urllib.parse.unquote(partes.path)
            try:
                if ruta in ("/", "/index.html"):
                    self._enviar(pagina_bytes, "text/html; charset=utf-8")
                elif ruta == "/pdf":
                    self._pdf(consulta)
                elif ruta == "/csv":
                    self._csv(consulta)
                elif ruta == "/video":
                    self._video(consulta)
                elif ruta.startswith("/frames/"):
                    self._frame(ruta)
                else:
                    self.send_error(404, f"ruta desconocida: {ruta}")
            except BrokenPipeError:
                pass  # el navegador canceló la descarga: no es un error
            except Exception as e:  # noqa: BLE001 - el servidor nunca debe caerse
                self.send_error(500, f"{type(e).__name__}: {e}")

        def _pdf(self, consulta):
            clave = consulta.get("fig", [""])[0]
            if clave not in FIGURAS:
                self.send_error(404, f"figura desconocida: {clave} "
                                     f"(hay: {', '.join(sorted(FIGURAS))})")
                return
            fig = FIGURAS[clave]
            nombre = clave.replace(":", "_")
            vuelta = consulta.get("vuelta", [None])[0]
            if vuelta:
                fig = _activar_vuelta(fig, vuelta)
                nombre += f"_v{vuelta}"
            self._enviar(figura_a_pdf(fig), "application/pdf", f"{nombre}.pdf")

        def _csv(self, consulta):
            clave = consulta.get("tabla", [""])[0]
            if clave not in TABLAS:
                self.send_error(404, f"tabla desconocida: {clave} "
                                     f"(hay: {', '.join(sorted(TABLAS))})")
                return
            self._enviar(tabla_a_csv(TABLAS[clave]), "text/csv; charset=utf-8",
                         f"{clave.replace(':', '_')}.csv")

        def _frame(self, ruta):
            m = re.fullmatch(r"/frames/([^/]+)/(\d+)\.jpg", ruta)
            if not m or m.group(1) not in IMAGENES:
                self.send_error(404, f"fotograma no encontrado: {ruta}")
                return
            lista = IMAGENES[m.group(1)]
            i = int(m.group(2))
            if not 0 <= i < len(lista):
                self.send_error(404, f"índice {i} fuera de rango "
                                     f"(0..{len(lista) - 1})")
                return
            self._enviar(lista[i][1], "image/jpeg")

        def _video(self, consulta):
            orden = [c for c in consulta.get("orden", [""])[0].split(",")
                     if c in IMAGENES]
            if not orden:
                orden = sorted(IMAGENES)
            if not orden:
                self.send_error(404, "el bag no contiene imágenes")
                return
            print(f"  montando vídeo de {', '.join(orden)} ...")
            self._enviar(montar_video(orden), "video/mp4",
                         f"mosaico_{'_'.join(orden)}.mp4")

        def log_message(self, formato, *args):
            pass  # sin ruido de peticiones por consola

    servidor = ThreadingHTTPServer(("0.0.0.0", PUERTO), Manejador)
    print(f"\nDashboard servido en http://localhost:{PUERTO}  (Ctrl+C para parar)")
    try:
        servidor.serve_forever()
    except KeyboardInterrupt:
        print("\nServidor parado.")


# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("carpeta_bag", type=Path,
                        help="Carpeta del bag (o que contiene el bag)")
    parser.add_argument("logs", nargs="*",
                        help="Logs del algoritmo (ficheros .txt o carpetas donde "
                             "buscarlos); por defecto se buscan en la carpeta del bag")
    parser.add_argument("--coche", default="car1", help="Nombre del coche (def: car1)")
    parser.add_argument("--sin-servidor", action="store_true",
                        help="Solo generar el HTML, sin servirlo")
    parser.add_argument("--sin-imagenes", action="store_true",
                        help="No cargar las imágenes del bag (arranca mucho más "
                             "rápido; la sección 7 queda vacía)")
    args = parser.parse_args()

    carpeta = args.carpeta_bag.resolve()
    if not carpeta.is_dir():
        sys.exit(f"ERROR: {carpeta} no es una carpeta")
    bag_dir = encontrar_bag(carpeta)
    print(f"Bag: {bag_dir}")

    # --- Logs del algoritmo ------------------------------------------------
    rutas_logs = buscar_logs(args.logs, carpeta)
    if not rutas_logs:
        print("AVISO: sin logs del algoritmo; no saldrán las secciones 4 y 6 ni "
              "la trayectoria base de la 2")
    logs = cargar_logs(rutas_logs)
    varias = len(logs) > 1
    for camara, d in logs.items():
        print(f"Log {camara}: {d['ruta'].name} · {len(d['vueltas'])} vueltas · "
              f"{d['c'].n_celdas} celdas · {len(d['c'].derrapes)} derrapes")

    # --- Bag ---------------------------------------------------------------
    # Las imágenes solo se cargan si se va a servir el dashboard: sin
    # servidor no hay visor que las use y ocupan cientos de MB en memoria
    con_imagenes = not args.sin_imagenes and not args.sin_servidor
    bag = leer_bag(bag_dir, args.coche, con_imagenes=con_imagenes)
    posiciones, telemetria, vueltas = bag["posiciones"], bag["telemetria"], bag["vueltas"]
    IMAGENES.update({cam: sorted(fs) for cam, fs in bag["imagenes"].items()})
    print(f"Mensajes: {len(posiciones)} posiciones, {len(telemetria)} telemetrías, "
          f"{len(vueltas)} vueltas cronometradas")
    if IMAGENES:
        print("Imágenes: " + ", ".join(f"{c} ({len(f)} frames)"
                                       for c, f in sorted(IMAGENES.items())))
    # Instante del primer mensaje del bag: los tiempos del visor de imágenes
    # se muestran relativos a él (segundos desde el inicio de la grabación)
    primeros = [p["t"] for p in posiciones[:1]]
    primeros += [frames[0][0] for frames in IMAGENES.values() if frames]
    t0_bag = min(primeros) if primeros else 0
    if vueltas:
        tiempos = [v["tiempo"] for v in vueltas]
        print(f"Tiempos: mejor {min(tiempos):.3f} s | media "
              f"{sum(tiempos) / len(tiempos):.3f} s | peor {max(tiempos):.3f} s")

    # El DataFrame que alimenta las figuras 1 y 2: una fila por posición
    # válida, con su vuelta (ventanas de time_per_lap) y el dist_derrape de
    # la telemetría que originó (emparejada por tiempo de grabación)
    pos_validas = [p for p in posiciones
                   if (p["fx"], p["fy"]) != (0, 0) and (p["bx"], p["by"]) != (0, 0)]
    indices_tel = emparejar_telemetria(telemetria, pos_validas)
    vuelta_de_pos = repartir_por_vueltas([p["t"] for p in pos_validas], vueltas)
    dist_de_pos = [np.nan] * len(pos_validas)
    for i_tel, i_pos in enumerate(indices_tel):
        if i_pos is not None:
            dist_de_pos[i_pos] = telemetria[i_tel]["dist"]
    df_pos = pd.DataFrame(pos_validas)
    if len(df_pos):
        df_pos["vuelta"] = vuelta_de_pos
        df_pos["dist"] = dist_de_pos
    print(f"Posiciones válidas: {len(pos_validas)} "
          f"({sum(1 for v in vuelta_de_pos if v is not None)} dentro de vueltas); "
          f"telemetrías emparejadas: {sum(1 for i in indices_tel if i is not None)}"
          f"/{len(telemetria)}")

    # El DataFrame de la sección 3b: una fila por TELEMETRÍA caída dentro de
    # una vuelta, con el instante relativo al cruce de meta que la abrió, la
    # cámara y la pegatina trasera que la originaron (emparejadas arriba) y
    # el PWM que estaba en vigor en ese momento
    umbral_derrape = next(
        (d["c"].params["umbral_derrape"] for d in logs.values()
         if "umbral_derrape" in d["c"].params), UMBRAL_DERRAPE_DEF)
    max_dist_ruta = next(
        (d["c"].params["max_dist_ruta"] for d in logs.values()
         if "max_dist_ruta" in d["c"].params), MAX_DIST_RUTA_DEF)
    vuelta_de_tel = repartir_por_vueltas([m["t"] for m in telemetria], vueltas)
    pwm_de_tel = pwm_en_instantes([m["t"] for m in telemetria], bag["pwm"])
    # Inicio de cada vuelta: el mensaje time_per_lap se publica al CERRARLA,
    # así que la vuelta empezó lap_time segundos antes de ese instante
    inicio_vuelta = {v["numero"]: v["t"] - int(v["tiempo"] * 1e9) for v in vueltas}
    filas_tel = []
    for m, i_pos, v, pwm in zip(telemetria, indices_tel, vuelta_de_tel, pwm_de_tel):
        if v is None:
            continue  # calibración o hueco entre vueltas: no va en la figura
        p = pos_validas[i_pos] if i_pos is not None else None
        filas_tel.append({
            "vuelta": v,
            "t_vuelta": (m["t"] - inicio_vuelta[v]) / 1e9,
            "dist": m["dist"],
            "derrapando": m["derrapando"],
            "camara": p["camara"] if p else None,
            "bx": p["bx"] if p else np.nan,
            "by": p["by"] if p else np.nan,
            "pwm": pwm,
        })
    df_tel = pd.DataFrame(filas_tel)
    print(f"Telemetrías dentro de vueltas: {len(df_tel)}"
          + (f" · dist máx {df_tel['dist'].max():.1f} px · "
             f"{int((df_tel['dist'] > umbral_derrape).sum())} muestras sobre el "
             f"umbral ({umbral_derrape:.0f} px)" if len(df_tel) else ""))

    # --- Resumen + anomalías -----------------------------------------------
    avisos = []
    for camara, d in logs.items():
        avisos += [f"[{camara}] {a}" if varias else a
                   for a in detectar_anomalias(d["c"], d["zonas_fin_vuelta"],
                                               d["vueltas"])]
    if len(df_tel):
        avisos += anomalias_derrape_bag(df_tel, umbral_derrape)
    lista_avisos = "".join(f"<li>{html.escape(a)}</li>" for a in avisos) or \
        "<li>Sin anomalías detectadas.</li>"
    resumen_logs = "".join(
        f"<li><code>{html.escape(d['ruta'].name)}</code> ({camara}): "
        f"{len(d['vueltas'])} vueltas, cadena de {d['c'].n_celdas} celdas "
        f"({'cerrada' if d['c'].cerrada else 'abierta'}), "
        f"{len(d['c'].derrapes)} derrapes</li>"
        for camara, d in logs.items()
    ) or "<li>sin logs del algoritmo</li>"
    resumen_html = f"""
<section>
<h2>Resumen de la carrera</h2>
<p>Bag: <code>{html.escape(bag_dir.name)}</code> · {len(posiciones)} posiciones
({len(pos_validas)} válidas) · {len(vueltas)} vueltas cronometradas ·
{len(telemetria)} telemetrías.</p>
<h3>Logs del algoritmo</h3>
<ul>{resumen_logs}</ul>
<h3>Anomalías detectadas</h3>
<ul class="avisos">{lista_avisos}</ul>
</section>
"""

    # --- Figuras -----------------------------------------------------------
    print("\nGenerando figuras ...")
    partes = []
    primera = True

    hay_vueltas_pos = len(df_pos) and df_pos["vuelta"].notna().any()
    if hay_vueltas_pos:
        # Los datos del bag que alimentan las figuras 1 y 2 (mismo DataFrame)
        TABLAS["posiciones"] = df_pos
        FIGURAS["1"] = grafica_1_derrape3d(df_pos)
        partes.append(seccion(
            "1 · Valor de derrape en cada punto del recorrido",
            "La trayectoria de la pegatina delantera como línea base (violeta, "
            "z=0) y, como altura y color, la distancia de derrape de la "
            "trasera en cada punto. El slider enseña UNA vuelta cada vez, y la "
            "caja tiene los ejes fijos: el encuadre no cambia al pasar de "
            "vuelta, así que dos vueltas se comparan tal cual. El rombo ámbar "
            "es el inicio de meta y la flecha de su lado, el sentido de la "
            "marcha. La vista se rota "
            "arrastrando con el ratón. Para sacar una vuelta concreta a PDF: "
            "mover el slider y añadir <code>&amp;vuelta=N</code> al enlace, o "
            "descargar y repetir.",
            FIGURAS["1"], primera=primera, claves=["1"], csv="posiciones",
        ))
        primera = False
        # La trayectoria BASE y las perpendiculares de derrape salen de las
        # celdas del log: sin logs la figura se queda solo con las dos
        # pegatinas (ni checkbox de base ni perpendiculares)
        celdas_por_camara = {cam: d["c"].celdas for cam, d in logs.items()}
        cerradas = {cam: d["c"].cerrada for cam, d in logs.items()}
        FIGURAS["2"] = grafica_2_trayectorias(
            df_pos, celdas_por_camara=celdas_por_camara, cerradas=cerradas,
            umbral=umbral_derrape, max_dist_ruta=max_dist_ruta)
        partes.append(seccion_2_trayectorias(FIGURAS["2"], bool(logs)))
    else:
        partes.append("<section><h2>1 · Valor de derrape en cada punto del "
                      "recorrido</h2><p>Sin posiciones dentro de vueltas "
                      "cronometradas en el bag.</p></section>")
        partes.append("<section><h2>2 · Trayectorias de la etiqueta delantera "
                      "y trasera</h2><p>Sin posiciones dentro de vueltas "
                      "cronometradas en el bag.</p></section>")

    if logs:
        # Los frames del log (la serie de distancia perpendicular) y las
        # celdas de la cadena, por cámara, para descargar como CSV
        for cam, d in logs.items():
            TABLAS[f"frames:{cam}"] = d["c"].frames
            TABLAS[f"celdas:{cam}"] = d["c"].celdas
            TABLAS[f"derrapes:{cam}"] = d["c"].derrapes
            TABLAS[f"resumen_vueltas:{cam}"] = d["tabla"]

    # --- 3b: la distancia de derrape que quedó grabada en el bag -----------
    # Va aquí (y no dentro del `if logs`) porque no necesita el log: sale de
    # la telemetría del propio bag
    if len(df_tel):
        TABLAS["derrape_bag"] = df_tel
        tabla_3c = tabla_derrape_por_vuelta(df_tel, umbral_derrape)
        TABLAS["derrape_por_vuelta"] = tabla_3c
        FIGURAS["3b"] = grafica_3b_derrape_bag(df_tel, umbral_derrape)
        FIGURAS["3c"] = grafica_3c_resumen_derrape(tabla_3c, umbral_derrape)
        partes.append(seccion_3b_derrape_bag(FIGURAS["3b"], FIGURAS["3c"], primera))
        primera = False
    else:
        partes.append(
            "<section><h2>3b · Distancia de derrape del bag, vuelta a "
            "vuelta</h2><p>El bag no trae telemetría del controlador "
            "(<code>/telemetria/&lt;coche&gt;/car_control</code>) dentro de "
            "vueltas cronometradas, así que no hay <code>dist_derrape</code> "
            "que dibujar. Es lo que pasa con las grabaciones hechas sin el "
            "algoritmo (PWM manual): solo llevan posiciones, órdenes de PWM y "
            "tiempos de vuelta.</p></section>")

    if logs:
        # df_pos aporta el primer punto de cada vuelta para marcar el inicio
        # de meta sobre el circuito (None si el bag no traía posiciones)
        FIGURAS["4"] = grafica_4_circuito(
            logs, df_pos if hay_vueltas_pos else None)
        partes.append(seccion(
            "4 · El circuito con el PWM de cada celda",
            "Todas las cámaras en el mismo plano (desplazadas según "
            "OFFSETS_CAMARAS en figuras.py, a ajustar por montaje): la "
            "trayectoria en gris tenue y cada celda pintada del color de su "
            "valor de PWM, con la leyenda diciendo qué valor es cada color. "
            "Como todas las celdas de una zona comparten PWM, <strong>un "
            "bloque de un solo color es una zona</strong> y un cambio de color "
            "es una frontera: moviendo el slider se ve cómo nacen, crecen y "
            "desaparecen las zonas vuelta a vuelta. La línea ámbar discontinua "
            "es un hueco tapado (celda gigante); la estrella ámbar es el "
            "inicio de meta y la flecha de su lado, el sentido de la marcha.",
            FIGURAS["4"], primera=primera, claves=["4"],
            csv=f"celdas:{next(iter(logs))}" if logs else None,
        ))
        primera = False

    partes.append(seccion_5_tiempos(vueltas, df_tel, umbral_derrape))

    if logs:
        figs6b, claves6b = [], []
        for cam, d in logs.items():
            clave = f"6b:{cam}" if varias else "6b"
            FIGURAS[clave] = etiquetar_camara(
                grafica_6b_heatmap(d["c"], d["vueltas"], d["perfil_vuelta"]),
                cam, varias)
            figs6b.append(FIGURAS[clave])
            claves6b.append(clave)
        partes.append(seccion(
            "6b · Perfil de PWM vuelta a vuelta",
            "Cada fila es el perfil con el que el coche corrió esa vuelta, "
            "expandiendo cada zona a sus celdas (claro = lento, oscuro = "
            "rápido). Las aspas rojas marcan celdas castigadas durante la "
            "vuelta y los círculos grises las celdas protegidas al empezarla "
            "(no subieron aunque la vuelta anterior fuese limpia).",
            figs6b, claves=claves6b,
        ))

        # 6c va con botones extra: además del panel completo, cada subplot
        # por separado (en la memoria interesa poder incluirlos sueltos)
        figs6c, claves6c = [], []
        botones_subplots = []
        for cam, d in logs.items():
            sufijo = f":{cam}" if varias else ""
            clave = f"6c{sufijo}"
            FIGURAS[clave] = etiquetar_camara(grafica_6c_resumen(d["tabla"]),
                                              cam, varias)
            figs6c.append(FIGURAS[clave])
            claves6c.append(clave)
            enlaces = []
            for panel in PANELES_6C:
                clave_panel = f"6c-{panel}{sufijo}"
                FIGURAS[clave_panel] = etiquetar_camara(
                    grafica_6c_subplot(d["tabla"], panel), cam, varias)
                enlaces.append(boton_pdf(clave_panel, panel))
            etiqueta = f" ({cam})" if varias else ""
            botones_subplots.append(
                f'<div class="descargas"><span style="font-size:12px">'
                f"PDF de cada panel por separado{html.escape(etiqueta)}: </span>"
                + "".join(enlaces) + "</div>")
        partes.append(seccion(
            "6c · Resumen por vuelta",
            "Los derrapes, el tiempo de vuelta (verde = al cerrarla el perfil "
            "subió), el PWM medio (perfil completo frente al aplicado donde "
            "pasó el coche) y la velocidad media medida sobre la trayectoria. "
            "Se puede descargar el panel completo o cada subplot suelto.",
            figs6c, claves=claves6c,
            csv=f"resumen_vueltas:{next(iter(logs))}",
        ) + "".join(botones_subplots))

    partes.append(seccion_7_visor(IMAGENES, t0_bag))

    # --- Sección 8: todos los CSV ------------------------------------------
    enlaces_csv = "".join(
        f"<li>{boton_csv(clave, clave)}</li>" for clave in sorted(TABLAS))
    partes.append(
        "<section><h2>8 · Descarga de datos (CSV)</h2>"
        "<p>Todas las tablas que alimentan las gráficas, por si hace falta "
        "analizarlas de otra forma (hoja de cálculo, otro script...). Las "
        "claves con dos puntos son por cámara.</p>"
        f'<ul class="lista-csv">{enlaces_csv}</ul></section>')

    # Ata la vuelta de la gráfica 2 con la de la 3b. Va al final del documento
    # a propósito: necesita que los dos <div> ya existan (ver la función)
    partes.append(script_sincronizar_vueltas())

    # --- Escribir y servir ---------------------------------------------------
    pagina = ensamblar_html(bag_dir.name, resumen_html, partes)
    salida = bag_dir.parent / f"{bag_dir.name}_analisis.html"
    salida.write_text(pagina, encoding="utf-8")
    print(f"  -> {salida}  ({salida.stat().st_size / 1e6:.1f} MB)")

    if not args.sin_servidor:
        servir(pagina.encode("utf-8"))


if __name__ == "__main__":
    main()
