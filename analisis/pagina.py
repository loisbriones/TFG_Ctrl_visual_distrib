#!/usr/bin/env python3
"""
Apartir de las figuras ya construidas por figuras.py (objetos
go.Figure), las convierte en HTML y las coloca en el esqueleto de
web/plantilla.html junto con el titulo de cada seccion, su explicacion y su
boton de PDF
"""

import html
import urllib.parse
from pathlib import Path

from figuras import COL_GRID, COL_SUPERFICIE, COL_TINTA, COL_TINTA_2, FUENTE
from textos import TEXTOS, TEXTOS_VACIOS, TITULOS

WEB = Path(__file__).resolve().parent / "web"

# Ids de los <div> que web/panel.js busca por nombre
# Cambiar uno aqui obliga a cambiarlo tambien alli
ID_GRAFICA_2 = "g-trayectorias"
ID_GRAFICA_3 = "g-derrape-bag"

# Los botones de PDF llevan a un endpoint del servidor, asi que con --sin-servidor no llevarian a ningun sitio
CON_SERVIDOR = True

# Config de plotly comun a todas las figuras incrustadas
CONFIG_PLOTLY = {"displaylogo": False, "responsive": True}



def id_div_figura(clave):
    """id del <div> de una figura. Hace falta uno estable (plotly pone un
    UUID distinto en cada generacion) para que el boton de PDF la encuentre"""
    return "g-fig-" + clave.replace(":", "-")

def boton_pdf(clave, etiqueta="Descargar PDF", div_id=None):
    """Enlace al endpoint /pdf. Con div_id, panel.js le añade al vuelo la
    vuelta que marque el slider justo antes de pinchar"""
    if not CON_SERVIDOR:
        return ""
    dato = f' data-grafica="{div_id}"' if div_id else ""
    return (f'<a class="boton" href="/pdf?fig={urllib.parse.quote(clave)}"'
            f"{dato} download>{html.escape(etiqueta)}</a>")

def botones_paneles(claves_y_etiquetas, nombre_camara=""):
    """La fila de botones para bajar cada subplot de la seccion 6 por separado"""
    if not CON_SERVIDOR:
        return ""
    enlaces = "".join(boton_pdf(clave, etiqueta)
                      for clave, etiqueta in claves_y_etiquetas)
    sufijo = f" ({nombre_camara})" if nombre_camara else ""
    return controles(f'<span style="font-size:12px">PDF de cada panel por '
                     f"separado{html.escape(sufijo)}: </span>" + enlaces)

def controles(*piezas):
    """La fila que va debajo de una grafica (botones, checkboxes)"""
    contenido = "".join(p for p in piezas if p)
    return f'<div class="controles">{contenido}</div>' if contenido else ""

def incrustar(fig, clave=None, con_plotlyjs=False):
    """Una figura como HTML. Solo la primera figura del documento embebe
    plotly.js 
    Las demas reutilizan esa copia"""
    return fig.to_html(full_html=False, include_plotlyjs=con_plotlyjs,
                       div_id=id_div_figura(clave) if clave else None,
                       config=CONFIG_PLOTLY)

def envolver(clave, cuerpo):
    """Una <section> con su titulo y su explicacion delante del cuerpo"""
    return (f"<section><h2>{html.escape(TITULOS[clave])}</h2>"
            f"<p>{TEXTOS[clave]}</p>{cuerpo}</section>")



def seccion_vacia(clave):
    """La seccion cuando el bag o los logs no traen lo que necesita"""
    return (f"<section><h2>{html.escape(TITULOS[clave])}</h2>"
            f"<p>{TEXTOS_VACIOS[clave]}</p></section>")

def seccion(clave, figs, claves, primera=False, extra=""):
    """Seccion estandar: titulo, explicacion y una figura por camara, cada una
    con su boton de PDF"""
    cuerpo = []
    for i, (fig, clave_fig) in enumerate(zip(figs, claves)):
        cuerpo.append(incrustar(fig, clave_fig, con_plotlyjs=primera and i == 0))
        cuerpo.append(controles(boton_pdf(clave_fig,
                                          div_id=id_div_figura(clave_fig))))
    return envolver(clave, "".join(cuerpo) + extra)

def seccion_trayectorias(fig, hay_base, primera):
    """Seccion 2. Lleva checkboxes para quitar y poner cada trayectoria"""
    def check(serie, etiqueta):
        return (f'<label><input type="checkbox" data-serie="{serie}" checked> '
                f"{etiqueta}</label>")

    # La delantera y la trasera siempre estan; la base y sus perpendiculares
    # solo si hay logs, que es de donde salen las celdas
    checks = [check("delantera", "Etiq. delantera"),
              check("trasera", "Etiq. trasera")]
    if hay_base:
        checks.insert(0, check("base", "Trayectoria base"))
        checks.append(check("perp", "Dist. derrape"))

    cuerpo = (
        f'<div class="controles"><div class="checks-tray" id="checks-tray-2">'
        f'{"".join(checks)}</div></div>'
        + fig.to_html(full_html=False, include_plotlyjs=primera,
                      div_id=ID_GRAFICA_2, config=CONFIG_PLOTLY)
        + controles(boton_pdf("2", "Descargar gráfica (PDF)", ID_GRAFICA_2))
    )
    texto = TEXTOS["2"] if hay_base else TEXTOS["2-sin-base"]
    return (f"<section><h2>{html.escape(TITULOS['2'])}</h2>"
            f"<p>{texto}</p>{cuerpo}</section>")

def seccion_derrape_bag(fig_detalle, fig_resumen, primera):
    """Seccion 3. Lleva muestra a muestra y debajo resumen por vuelta"""
    cuerpo = (
        fig_detalle.to_html(full_html=False, include_plotlyjs=primera,
                            div_id=ID_GRAFICA_3, config=CONFIG_PLOTLY)
        + controles(boton_pdf("3", "Descargar gráfica (PDF)", ID_GRAFICA_3))
        + f'<p>{TEXTOS["3-resumen"]}</p>'
        + incrustar(fig_resumen, "3-resumen")
        + controles(boton_pdf("3-resumen", "Descargar resumen (PDF)"))
    )
    return envolver("3", cuerpo)

def seccion_tiempos(vueltas, stats, fig_tendencia, fig_cajas, primera=False):
    """Seccion 5: la tabla de tiempos y al lado la tendencia y las cajas"""
    filas = []
    for v in vueltas:
        clases, nota = [], ""
        if v["tiempo"] == stats["mejor"]:
            clases.append("mejor")
            nota = " ★ mejor"
        if v["tiempo"] > stats["factor_anomala"] * stats["media"]:
            clases.append("anomala")
            nota = f" ⚠ &gt;{stats['factor_anomala']:.0f}× media"
        filas.append(f'<tr class="{" ".join(clases)}"><td>{v["numero"]}</td>'
                     f'<td>{v["tiempo"]:.3f}{nota}</td></tr>')
    pie = "".join(f"<tr><td>{nombre}</td><td>{stats[nombre]:.3f}</td></tr>"
                  for nombre in ("mejor", "media", "mediana", "peor"))
    tabla = ('<table class="tiempos"><thead><tr><th>vuelta</th>'
             "<th>tiempo (s)</th></tr></thead><tbody>"
             + "".join(filas) + f"</tbody><tfoot>{pie}</tfoot></table>")

    cajas = ""
    if fig_cajas is not None:
        cajas = (f'<p>{TEXTOS["5-cajas"]}</p>' + incrustar(fig_cajas, "5-cajas")
                 + controles(boton_pdf("5-cajas")))

    cuerpo = (f'<div class="fila-tiempos"><div>{tabla}</div>'
              f'<div class="mini-grafica">'
              f'{incrustar(fig_tendencia, "5", con_plotlyjs=primera)}'
              f'{controles(boton_pdf("5", "Descargar gráfica (PDF)"))}'
              f"{cajas}</div></div>")
    return envolver("5", cuerpo)

def seccion_resumen(nombre_bag, n_posiciones, n_validas, n_vueltas,
                    n_telemetria, logs):
    """La cabecera: de donde salen los datos y que trae cada log"""
    lineas = "".join(
        f"<li><code>{html.escape(d['ruta'].name)}</code> ({camara}): "
        f"{len(d['vueltas'])} vueltas, cadena de {d['c'].n_celdas} celdas "
        f"({'cerrada' if d['c'].cerrada else 'abierta'}), "
        f"{len(d['c'].derrapes)} derrapes</li>"
        for camara, d in logs.items()
    ) or "<li>sin logs del algoritmo</li>"
    return (
        "<section><h2>Resumen de la carrera</h2>"
        f"<p>Bag: <code>{html.escape(nombre_bag)}</code> · {n_posiciones} "
        f"posiciones ({n_validas} válidas) · {n_vueltas} vueltas cronometradas "
        f"· {n_telemetria} telemetrías.</p>"
        f"<h3>Logs del algoritmo</h3><ul>{lineas}</ul></section>"
    )



def _estilos():
    """El CSS, con los colores de figuras.py puestos en sus variables"""
    paleta = {
        "--fuente": FUENTE,
        "--col-superficie": COL_SUPERFICIE,
        "--col-tinta": COL_TINTA,
        "--col-tinta-2": COL_TINTA_2,
        "--col-grid": COL_GRID,
    }
    declaraciones = "\n".join(f"  {k}: {v};" for k, v in paleta.items())
    css = (WEB / "estilos.css").read_text(encoding="utf-8")
    return css.replace(":root { color-scheme: light; }",
                       ":root {\n  color-scheme: light;\n" + declaraciones + "\n}")

def ensamblar(nombre, secciones):
    """Devuelve web/plantilla.html con sus marcadores {{...}} sustituidos"""

    pagina = (WEB / "plantilla.html").read_text(encoding="utf-8")
    for marcador, valor in (
        ("{{titulo}}", html.escape(nombre)),
        ("{{estilos}}", _estilos()),
        ("{{secciones}}", "".join(secciones)),
        ("{{script}}", (WEB / "panel.js").read_text(encoding="utf-8")),
    ):
        pagina = pagina.replace(marcador, valor)

    return pagina

def etiquetar_camara(fig, camara, varias):
    if varias:
        fig.update_layout(title_text=fig.layout.title.text + f" — {camara}")
    return fig
