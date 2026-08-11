#!/usr/bin/env python3
"""
El servidor del dashboard y la exportación a PDF.

Sirve dos cosas en el puerto 8988:
  /              la página
  /pdf?fig=<clave>[&vuelta=N]   una figura en PDF vectorial

El PDF existe solo aquí y no en el HTML suelto porque lo genera kaleido, que
está instalado dentro del contenedor (ver Dockerfile). De ahí que con
--sin-servidor la página salga directamente sin botones.
"""

import urllib.parse
import warnings
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import plotly.graph_objects as go

# Puerto dentro del contenedor; run.sh lo publica tal cual en el host
PUERTO = 8988

# Ancho en puntos del PDF. Las figuras solo fijan el alto (_layout_base), y sin
# esto kaleido usaría su ancho por defecto de 700 px, demasiado estrecho.
ANCHO_PDF = 1100


def figura_a_pdf(fig, vuelta=None):
    """PDF vectorial de una figura, listo para \\includegraphics.

    `vuelta` solo se escribe en el título: sin el slider, el papel no diría de
    qué vuelta es la figura."""
    copia = go.Figure(fig)
    if vuelta is not None and copia.layout.title.text:
        copia.layout.title.text += f" · vuelta {vuelta}"
    # Fuera el slider y los menús: kaleido dibuja el estado ACTIVO de la figura
    # y esos controles solo estorbarían en el papel. Se asignan directamente
    # sobre layout porque update_layout(sliders=[]) no vacía la tupla existente.
    copia.layout.sliders = ()
    copia.layout.updatemenus = ()
    if copia.layout.width is None:
        copia.layout.width = ANCHO_PDF
    with warnings.catch_warnings():
        # kaleido está fijado a la 0.2.1 a propósito (la 1.x exige un Chrome
        # instalado) y plotly avisa de que es antigua en CADA exportación
        warnings.simplefilter("ignore", DeprecationWarning)
        return copia.to_image(format="pdf")


def activar_vuelta(fig, vuelta):
    """Deja una copia de la figura en el estado de la vuelta pedida.

    El estado del slider vive en el navegador; aquí se reproduce aplicando el
    array de visibilidad que ya lleva el paso correspondiente."""
    copia = go.Figure(fig)
    if not copia.layout.sliders:
        return copia
    pasos = copia.layout.sliders[0].steps
    if not any(p.label == str(vuelta) for p in pasos):
        print(f"AVISO: la figura no tiene la vuelta {vuelta} "
              f"(tiene: {', '.join(p.label for p in pasos)}); "
              "el PDF sale con la vuelta que estuviera activa")
    for paso in pasos:
        if paso.label != str(vuelta):
            continue
        args = paso.args or []
        if args and isinstance(args[0], dict) and "visible" in args[0]:
            for traza, visible in zip(copia.data, args[0]["visible"]):
                traza.visible = visible
        break
    return copia


def servir(pagina_bytes, figuras_registradas):
    """Levanta el servidor hasta Ctrl+C. `figuras_registradas` es
    {clave: go.Figure}: las mismas claves que llevan los botones de la página."""

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
            ruta = urllib.parse.unquote(partes.path)
            try:
                if ruta in ("/", "/index.html"):
                    self._enviar(pagina_bytes, "text/html; charset=utf-8")
                elif ruta == "/pdf":
                    self._pdf(urllib.parse.parse_qs(partes.query))
                else:
                    self.send_error(404, f"ruta desconocida: {ruta}")
            except BrokenPipeError:
                pass   # el navegador canceló la descarga: no es un error
            except Exception as e:  # noqa: BLE001 - el servidor no debe caerse
                self.send_error(500, f"{type(e).__name__}: {e}")

        def _pdf(self, consulta):
            clave = consulta.get("fig", [""])[0]
            if clave not in figuras_registradas:
                self.send_error(
                    404, f"figura desconocida: {clave} "
                         f"(hay: {', '.join(sorted(figuras_registradas))})")
                return
            fig = figuras_registradas[clave]
            nombre = clave.replace(":", "_")
            vuelta = consulta.get("vuelta", [None])[0]
            if vuelta:
                fig = activar_vuelta(fig, vuelta)
                nombre += f"_v{vuelta}"
            self._enviar(figura_a_pdf(fig, vuelta), "application/pdf",
                         f"{nombre}.pdf")

        def log_message(self, formato, *args):
            pass   # sin ruido de peticiones por consola

    servidor = ThreadingHTTPServer(("0.0.0.0", PUERTO), Manejador)
    print(f"\nDashboard servido en http://localhost:{PUERTO}  "
          "(Ctrl+C para parar)")
    try:
        servidor.serve_forever()
    except KeyboardInterrupt:
        print("\nServidor parado.")
