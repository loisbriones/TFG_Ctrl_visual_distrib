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

Secciones del dashboard:
  0  Resumen de la carrera + anomalías detectadas
  1  Derrape 3D sobre la trayectoria (slider ACUMULATIVO por vueltas)
  2  Trayectorias de las dos pegatinas (slider INSTANTÁNEO: una vuelta)
  3  Distancia perpendicular de la trasera vs celda (INSTANTÁNEO)
  4  El circuito con el PWM de cada celda (todas las cámaras en un plano)
  5  Tabla de tiempos por vuelta + tendencia
  6  Análisis del algoritmo: zonas (6a), heatmap de PWM (6b), resumen (6c)
  7  Visor de imágenes de las cámaras + vídeo mosaico descargable
  8  Descarga de todas las tablas en CSV

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
    grafica_3_dist, grafica_4_circuito, grafica_6a_zonas, grafica_6b_heatmap,
    grafica_6c_resumen, grafica_6c_subplot,
)
from lectura_bag import (
    emparejar_telemetria, encontrar_bag, leer_bag, repartir_por_vueltas,
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

# fps del vídeo mosaico de la sección 7 (--vídeo, no --datos--: es solo para
# revisar las cámaras, no hace falta que sea exacto). Las cámaras no están
# sincronizadas por timestamp en el mosaico: se combinan por ÍNDICE de
# frame (i-ésimo frame de cada cámara), que es una aproximación razonable
# porque todas capturan a un ritmo similar (~30 Hz); si hace falta más
# precisión, cambiar aquí a un montaje por timestamp más cercano.
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
# Sección 5: tabla de tiempos por vuelta + mini-gráfica de tendencia
# ---------------------------------------------------------------------------
def seccion_5_tiempos(vueltas):
    """Tabla HTML con todas las vueltas y su tiempo (mejor vuelta resaltada,
    anómalas marcadas) + pie con mejor/media/peor + mini-gráfica de línea al
    lado para ver la tendencia. Devuelve el HTML de la sección completa."""
    if not vueltas:
        return ("<section><h2>5 · Tiempos por vuelta</h2>"
                "<p>El bag no contiene mensajes de time_per_lap.</p></section>")

    tiempos = [v["tiempo"] for v in vueltas]
    mejor = min(tiempos)
    media = sum(tiempos) / len(tiempos)
    peor = max(tiempos)

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
        f"<tr><td>peor</td><td>{peor:.3f}</td></tr></tfoot></table>"
    )

    fig = go.Figure(
        go.Scatter(
            x=[v["numero"] for v in vueltas], y=tiempos, mode="lines+markers",
            line=dict(color=COL_SERIE_1, width=2), marker=dict(size=6),
            hovertemplate="v%{x}: %{y:.3f} s<extra></extra>",
        )
    )
    fig.add_hline(y=media, line=dict(color=COL_GRID, width=1, dash="dash"),
                  annotation_text=f"media {media:.2f} s",
                  annotation_font_color=COL_TINTA_2)
    fig.update_xaxes(title_text="vuelta", dtick=5)
    fig.update_yaxes(title_text="tiempo (s)", rangemode="tozero")
    _layout_base(fig, "Tendencia del tiempo por vuelta", 420)
    fig.update_layout(margin=dict(l=60, r=20, t=60, b=50))
    mini = fig.to_html(full_html=False, include_plotlyjs=False,
                       config={"displaylogo": False, "responsive": True})

    # La mini-gráfica también se puede sacar a PDF y los tiempos a CSV
    FIGURAS["5"] = fig
    TABLAS["tiempos"] = pd.DataFrame(vueltas)

    return (
        "<section><h2>5 · Tiempos por vuelta</h2>"
        "<p>Tiempos medidos por el controlador al cruzar la línea de meta "
        "(topic time_per_lap del bag). La mejor vuelta va resaltada; una "
        f"vuelta que supere {FACTOR_VUELTA_ANOMALA:.0f}× la media se marca "
        "como anómala (parada, salida de pista, relocalización...).</p>"
        f'<div class="fila-tiempos"><div>{tabla_html}</div>'
        f'<div class="mini-grafica">{mini}</div></div>'
        f'<div class="descargas">{boton_pdf("5", "Descargar gráfica (PDF)")}'
        f'{boton_csv("tiempos", "Descargar tiempos (CSV)")}</div></section>'
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
({', '.join(f'{html.escape(c)}: {n_frames[c]}' for c in camaras)} frames).
Las imágenes se piden al servidor según se mueve el slider (no van dentro
del HTML: pesarían cientos de MB). Las cámaras se combinan por índice de
frame, no por timestamp exacto.</p>
<div class="visor">
  <div class="visor-controles">
    <button id="visor-ant">◀ anterior</button>
    <input type="range" id="visor-slider" min="0" max="{n_max - 1}" value="0">
    <button id="visor-sig">siguiente ▶</button>
    <span id="visor-pos">frame 0 / {n_max - 1}</span>
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
    pos.textContent = "frame " + i + " / " + slider.max;
    for (const cam of camaras) {{
      // Cada cámara tiene su propio número de frames: si esta se queda
      // corta se muestra su último frame disponible
      const ult = tiempos[cam].length - 1;
      const j = Math.min(i, ult);
      document.getElementById("img-" + cam).src =
        "/frames/" + encodeURIComponent(cam) + "/" + j + ".jpg";
      document.getElementById("cap-" + cam).textContent =
        "frame " + j + " · t = " + tiempos[cam][j] + " s";
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
        print("AVISO: sin logs del algoritmo; las secciones 3, 4 y 6 no saldrán")
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

    # --- Resumen + anomalías -----------------------------------------------
    avisos = []
    for camara, d in logs.items():
        avisos += [f"[{camara}] {a}" if varias else a
                   for a in detectar_anomalias(d["c"], d["zonas_fin_vuelta"],
                                               d["vueltas"])]
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
            "trasera en cada punto. El slider ACUMULA: el paso v superpone "
            "las vueltas 1..v, para ver cómo crecen las zonas de derrape con "
            "las vueltas. La vista se rota arrastrando con el ratón. Para "
            "sacar una vuelta concreta a PDF: mover el slider y añadir "
            "<code>&amp;vuelta=N</code> al enlace, o descargar y repetir.",
            FIGURAS["1"], primera=primera, claves=["1"], csv="posiciones",
        ))
        primera = False
        FIGURAS["2"] = grafica_2_trayectorias(df_pos)
        partes.append(seccion(
            "2 · Trayectorias de la etiqueta delantera y trasera",
            "Las dos pegatinas en el plano global, una vuelta cada vez: al "
            "mover el slider se ve cómo cambia la trayectoria de una vuelta "
            "a la siguiente (dónde se dispersa la trasera y dónde va pegada "
            "a la delantera).",
            FIGURAS["2"], claves=["2"], csv="posiciones",
        ))
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

        figs3, claves3 = [], []
        for cam, d in logs.items():
            clave = f"3:{cam}" if varias else "3"
            FIGURAS[clave] = etiquetar_camara(
                grafica_3_dist(d["c"], d["vueltas"]), cam, varias)
            figs3.append(FIGURAS[clave])
            claves3.append(clave)
        partes.append(seccion(
            "3 · Detección de derrape sobre la trayectoria",
            "La distancia perpendicular de la pegatina trasera en función de "
            "su celda, la serie que dispara la máquina de derrapes. En gris "
            "todas las vueltas (contexto), en azul la vuelta del slider y en "
            "rojo sus frames con derrape abierto. La línea negra es el umbral "
            "leído del log.",
            figs3, primera=primera, claves=claves3,
            csv=f"frames:{next(iter(logs))}" if logs else None,
        ))
        primera = False

        FIGURAS["4"] = grafica_4_circuito(logs)
        partes.append(seccion(
            "4 · El circuito con el PWM de cada celda",
            "Todas las cámaras en el mismo plano (desplazadas según "
            "OFFSETS_CAMARAS en figuras.py, a ajustar por montaje): la "
            "trayectoria en gris tenue y cada celda con su PWM escrito como "
            "número. El anillo rojo marca las celdas dentro de una zona de "
            "derrape al empezar la vuelta del slider; la línea ámbar "
            "discontinua es un hueco tapado (celda gigante).",
            FIGURAS["4"], primera=primera, claves=["4"],
            csv=f"celdas:{next(iter(logs))}" if logs else None,
        ))
        primera = False

    partes.append(seccion_5_tiempos(vueltas))

    if logs:
        figs6a, claves6a = [], []
        for cam, d in logs.items():
            clave = f"6a:{cam}" if varias else "6a"
            FIGURAS[clave] = etiquetar_camara(
                grafica_6a_zonas(d["c"], d["vueltas"], d["zonas_fin_vuelta"]),
                cam, varias)
            figs6a.append(FIGURAS[clave])
            claves6a.append(clave)
        partes.append(seccion(
            "6a · Evolución de las zonas de derrape",
            "Cada fila es una vuelta (la primera arriba, se lee hacia abajo). "
            "Las barras naranjas son las zonas de derrape <em>tal como quedan "
            "al terminar esa vuelta</em>, con su PWM en el hover. Debajo de "
            "cada fila, en rojo, los derrapes individuales de esa vuelta "
            "(rombo = 1 sola celda) y las estrellas marcan fusiones. Los "
            "círculos grises son aperturas descartadas por la zona muerta; "
            "los triángulos ámbar, derrames de reducción entre cámaras; la "
            "banda ámbar vertical, una celda gigante (hueco tapado).",
            figs6a, primera=primera, claves=claves6a,
            csv=f"derrapes:{next(iter(logs))}",
        ))
        primera = False

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

    # --- Escribir y servir ---------------------------------------------------
    pagina = ensamblar_html(bag_dir.name, resumen_html, partes)
    salida = bag_dir.parent / f"{bag_dir.name}_analisis.html"
    salida.write_text(pagina, encoding="utf-8")
    print(f"  -> {salida}  ({salida.stat().st_size / 1e6:.1f} MB)")

    if not args.sin_servidor:
        servir(pagina.encode("utf-8"))


if __name__ == "__main__":
    main()
