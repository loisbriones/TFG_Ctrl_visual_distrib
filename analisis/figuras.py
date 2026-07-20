#!/usr/bin/env python3
"""
Todas las figuras plotly del dashboard unificado (analisis.py).

Las figuras 6a/6b/3/6c (zonas, heatmap de PWM, distancia perpendicular y
resumen por vuelta) están movidas TAL CUAL de analizar_log_algoritmo.py;
solo cambió el número de sección en el título. Las nuevas (circuito con PWM,
derrape 3D, trayectorias) siguen el mismo estilo: paleta validada del skill
dataviz, chrome común en _layout_base y sliders por vuelta.

Convención de sliders del dashboard:
  - INSTANTÁNEO: cada paso muestra SOLO su vuelta (figuras 2, 3 y 4).
  - ACUMULATIVO: el paso v muestra las vueltas 1..v superpuestas (figura 1).
Ambos se implementan con arrays de visibilidad: las trazas "estáticas"
(fondos, trayectorias) siempre visibles y un bloque de trazas por vuelta.
"""

import numpy as np
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots

from parseo_log import Carrera

# ---------------------------------------------------------------------------
# Colores (paleta validada del skill dataviz, modo claro). Los roles de estado
# se reservan para "cosas malas" (derrapes/zonas) y los secuenciales para
# magnitud (PWM); el texto siempre va en tintas, nunca en color de serie.
# ---------------------------------------------------------------------------
COL_SUPERFICIE = "#fcfcfb"   # fondo de las gráficas y de la página
COL_TINTA = "#0b0b0b"        # texto principal
COL_TINTA_2 = "#52514e"      # texto secundario (explicaciones)
COL_MUTED = "#898781"        # ejes, etiquetas apagadas, marcadores neutros
COL_GRID = "#e1e0d9"         # rejilla fina
COL_SERIE_1 = "#2a78d6"      # azul, serie categórica 1 (dist, tiempos...)
COL_SERIE_2 = "#1baf7a"      # aqua, serie categórica 2 (segunda serie PWM)
COL_CRITICO = "#d03b3b"      # rojo estado "critical": eventos de derrape
COL_SERIO = "#ec835a"        # naranja estado "serious": zonas de derrape
COL_AVISO = "#b58324"        # ámbar: celdas gigantes y derrames entre cámaras
COL_BUENO = "#0ca30c"        # verde estado "good": vuelta con subida de perfil
COL_CONTEXTO = "#c3c2b7"     # gris de las series de fondo/contexto

# Rampa secuencial azul claro->oscuro para el PWM: claro = lento (v_min),
# oscuro = rápido (v_max), en estructura colorscale de plotly.
_RAMPA_AZUL = [
    "#cde2fb", "#b7d3f6", "#9ec5f4", "#86b6ef", "#6da7ec", "#5598e7",
    "#3987e5", "#2a78d6", "#256abf", "#1c5cab", "#184f95", "#104281", "#0d366b",
]
ESCALA_PWM = [[i / (len(_RAMPA_AZUL) - 1), c] for i, c in enumerate(_RAMPA_AZUL)]

# Tipografía de todo el HTML y las figuras (sans del sistema, sin serifas)
FUENTE = 'system-ui, -apple-system, "Segoe UI", sans-serif'


# ---------------------------------------------------------------------------
# PLANO GLOBAL multicámara: cada cámara tiene su propio sistema de píxeles
# (su imagen), así que para dibujar TODO el circuito en una sola figura cada
# cámara entra con una traslación (dx, dy). Los valores se ajustan A MANO
# aquí hasta que el circuito quede continuo (no hay calibración automática:
# depende de cómo estén colocadas las cámaras en cada montaje). Una cámara
# que no aparezca en el dict se coloca automáticamente a la derecha de la
# anterior (una "plaza" de ANCHO_PLAZA_CAMARA px por cámara).
# ---------------------------------------------------------------------------
OFFSETS_CAMARAS = {
    "camara_01": (0.0, 0.0),
    "camara_02": (660.0, 0.0),
}
# Ancho de la plaza por defecto: 640 px de imagen + 20 de margen visual
ANCHO_PLAZA_CAMARA = 660.0


def offset_camara(camara, indice):
    """(dx, dy) de una cámara en el plano global; si no está en el dict se
    coloca en la plaza `indice` (orden alfabético de cámaras)."""
    if camara in OFFSETS_CAMARAS:
        return OFFSETS_CAMARAS[camara]
    return (indice * ANCHO_PLAZA_CAMARA, 0.0)
# ---------------------------------------------------------------------------
# Aspecto común de las figuras
# ---------------------------------------------------------------------------
def _layout_base(fig, titulo, alto):
    """Chrome común: superficie clara, tinta oscura, rejilla fina, sin logo."""
    fig.update_layout(
        title=dict(text=titulo, font=dict(size=15, color=COL_TINTA)),
        height=alto,
        paper_bgcolor=COL_SUPERFICIE,
        plot_bgcolor=COL_SUPERFICIE,
        font=dict(family=FUENTE, size=12, color=COL_TINTA),
        margin=dict(l=70, r=30, t=95, b=60),
        legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1),
        hoverlabel=dict(font=dict(family=FUENTE, size=12)),
    )
    fig.update_xaxes(gridcolor=COL_GRID, zeroline=False, linecolor=COL_CONTEXTO)
    fig.update_yaxes(gridcolor=COL_GRID, zeroline=False, linecolor=COL_CONTEXTO)
    return fig


def _celda_fisica(c: Carrera, celda):
    """Posición (x, y) de una celda para el hover (None si es gigante)."""
    if c.celdas is None or c.celdas.empty:
        return None
    sel = c.celdas[c.celdas["celda"] == celda]
    if sel.empty or bool(sel.iloc[0]["gigante"]):
        return None
    return sel.iloc[0]


# ===========================================================================
# FIGURA 6a: evolución de las zonas (la central del análisis)
# ===========================================================================
# Eje X = celda de la cadena, eje Y = vuelta (la primera arriba: se lee hacia
# abajo como el propio log). En cada fila (vuelta) se dibuja:
#   - barra naranja gruesa  = cada zona de DERRAPE tal como queda al terminar
#     esa vuelta, con su PWM en el hover
#   - banda ámbar           = las celdas gigantes (fijas todas las vueltas)
#   - segmento/rombo rojo   = cada derrape individual de esa vuelta
#   - estrella roja         = una fusión (el hover lista las zonas absorbidas)
#   - círculo gris          = apertura ignorada por la zona muerta
#   - triángulo ámbar       = derrame de reducción hacia la cámara precedente
# ===========================================================================
def grafica_6a_zonas(c: Carrera, vueltas, zonas_fin_vuelta):
    fig = go.Figure()

    # Bandas verticales fijas en las celdas gigantes
    gigantes = c.celdas[c.celdas["gigante"]] if c.celdas is not None else []
    if len(gigantes):
        for _, g in gigantes.iterrows():
            fig.add_vrect(
                x0=g["celda"] - 0.5, x1=g["celda"] + 0.5,
                fillcolor=COL_AVISO, opacity=0.15, line_width=0,
            )

    # Zonas de derrape al cierre de cada vuelta: una única traza con
    # segmentos separados por None (ligero); el hover lleva pwm e historial
    xs, ys, textos = [], [], []
    for v in vueltas:
        for z in zonas_fin_vuelta[v]:
            if z["tipo"] != "derrape":
                continue
            # El historial de vueltas solo viene en los volcados [ZONA] (los
            # [PERFIL] no lo llevan): puede no estar disponible
            hist = z.get("vueltas")
            texto = (
                f"zona [{z['ini']}, {z['fin']}] ({z['fin'] - z['ini'] + 1} celdas)"
                f"<br>pwm={z['pwm']}"
                + (f"<br>derrapes en vueltas: {hist}" if hist is not None else "")
            )
            xs += [z["ini"], z["fin"], None]
            ys += [v, v, None]
            textos += [texto, texto, ""]
    fig.add_trace(
        go.Scatter(
            # lines+markers: los marcadores hacen visibles las zonas de 1 celda
            x=xs, y=ys, mode="lines+markers", name="zona de derrape",
            line=dict(color=COL_SERIO, width=7),
            marker=dict(size=5, color=COL_SERIO),
            text=textos, hovertemplate="%{text}<extra></extra>",
        )
    )

    # Derrapes individuales de cada vuelta (eventos CERRADO), medio carril
    # por debajo de la fila para no pisar la barra de la zona
    xs, ys, textos = [], [], []
    px_uno, py_uno, t_uno = [], [], []
    for _, d in c.derrapes.iterrows():
        texto = (
            f"derrape v{int(d['vuelta'])} frame {int(d['frame'])}"
            f"<br>celdas [{int(d['ini'])}, {int(d['fin'])}] "
            f"({int(d['n_celdas'])} celdas)"
        )
        if d["n_celdas"] <= 1:
            px_uno.append(d["ini"])
            py_uno.append(d["vuelta"] - 0.30)
            t_uno.append(texto + "<br>(1 sola celda)")
        else:
            xs += [d["ini"], d["fin"], None]
            ys += [d["vuelta"] - 0.30, d["vuelta"] - 0.30, None]
            textos += [texto, texto, ""]
    fig.add_trace(
        go.Scatter(
            x=xs, y=ys, mode="lines", name="derrape (evento)",
            line=dict(color=COL_CRITICO, width=3),
            text=textos, hovertemplate="%{text}<extra></extra>",
        )
    )
    fig.add_trace(
        go.Scatter(
            x=px_uno, y=py_uno, mode="markers", name="derrape de 1 celda",
            marker=dict(symbol="diamond", size=7, color=COL_CRITICO),
            text=t_uno, hovertemplate="%{text}<extra></extra>",
        )
    )

    # Fusiones: estrella en el centro del carveo que absorbió zonas
    fus = [k for k in c.carveos if k["tipo"] == "fusion"]
    fig.add_trace(
        go.Scatter(
            x=[(k["ini"] + k["fin"]) / 2 for k in fus],
            y=[k["vuelta"] - 0.30 for k in fus],
            mode="markers", name="fusión de zonas",
            marker=dict(symbol="star", size=13, color=COL_CRITICO,
                        line=dict(color=COL_SUPERFICIE, width=1)),
            text=[
                f"FUSIÓN en v{k['vuelta']}: la zona pasa a "
                f"[{k['ini']}, {k['fin']}] pwm={k['pwm']}<br>absorbe: "
                + ", ".join(f"[{a}-{b}]" for a, b in k["absorbidas"])
                for k in fus
            ],
            hovertemplate="%{text}<extra></extra>",
        )
    )

    # Aperturas ignoradas por la zona muerta (contexto en gris)
    fig.add_trace(
        go.Scatter(
            x=[e["celda"] for e in c.zona_muerta],
            y=[e["vuelta"] - 0.30 for e in c.zona_muerta],
            mode="markers", name="ignorado (zona muerta)",
            marker=dict(symbol="circle-open", size=6, color=COL_MUTED),
            text=[
                f"v{e['vuelta']} frame {e['frame']}: dist={e['dist']:.1f} > "
                f"umbral pero la celda {e['celda']} está en zona muerta"
                for e in c.zona_muerta
            ],
            hovertemplate="%{text}<extra></extra>",
        )
    )

    # Derrames hacia la cámara precedente (salen por la celda 0) y
    # reducciones externas recibidas (entran por la última celda)
    fig.add_trace(
        go.Scatter(
            x=[0] * len(c.derrames),
            y=[d["vuelta"] - 0.30 for d in c.derrames],
            mode="markers", name="derrame a la precedente",
            marker=dict(symbol="triangle-left", size=11, color=COL_AVISO),
            text=[f"v{d['vuelta']}: se derraman {d['px']:.0f} px hacia la "
                  f"cámara precedente" for d in c.derrames],
            hovertemplate="%{text}<extra></extra>",
        )
    )
    fig.add_trace(
        go.Scatter(
            x=[max(c.n_celdas - 1, 0)] * len(c.externas),
            y=[d["vuelta"] - 0.30 for d in c.externas],
            mode="markers", name="reducción externa recibida",
            marker=dict(symbol="triangle-right", size=11, color=COL_AVISO),
            text=[f"v{d['vuelta']}: la cámara siguiente pide reducir "
                  f"{d['px']:.0f} px al final" for d in c.externas],
            hovertemplate="%{text}<extra></extra>",
        )
    )

    fig.update_xaxes(
        title_text="celda de la cadena",
        range=[-1.5, c.n_celdas + 0.5],
    )
    fig.update_yaxes(
        title_text="vuelta", range=[vueltas[-1] + 0.8, vueltas[0] - 0.8],
        dtick=1,
    )
    alto = max(500, 130 + 22 * len(vueltas))
    return _layout_base(
        fig,
        "6a · Evolución de las zonas de derrape "
        "(estado al cerrar cada vuelta + eventos de esa vuelta; "
        "banda ámbar = celda gigante)",
        alto,
    )


# ===========================================================================
# FIGURA 6b: heatmap del perfil de PWM (vuelta x celda)
# ===========================================================================
# Cada fila es el perfil CON EL QUE EL COCHE CORRIÓ esa vuelta (el último
# volcado [PERFIL] anterior a su primer frame), expandiendo las zonas a
# celdas. Claro = lento (v_min), oscuro = rápido (v_max). Encima:
#   - aspa roja     = celdas castigadas DURANTE esa vuelta (carveos y
#                     castigos de gigantes; efecto en la fila siguiente)
#   - círculo gris  = celdas protegidas al EMPEZAR esa vuelta (no subieron
#                     aunque la vuelta anterior fuera limpia)
# ===========================================================================
def grafica_6b_heatmap(c: Carrera, vueltas, perfil_vuelta):
    v_min = c.params.get("v_min", np.nan)
    v_max = c.params.get("v_max", np.nan)

    z = np.full((len(vueltas), c.n_celdas), np.nan)
    for i, v in enumerate(vueltas):
        z[i, :] = perfil_vuelta[v]
    fig = go.Figure(
        go.Heatmap(
            z=z, x=list(range(c.n_celdas)), y=vueltas,
            colorscale=ESCALA_PWM, zmin=v_min, zmax=v_max,
            xgap=1, ygap=1,
            colorbar=dict(title="PWM"),
            hovertemplate=("vuelta %{y} · celda %{x}"
                           "<br>PWM del perfil: %{z}<extra></extra>"),
        )
    )
    # Castigos aplicados durante cada vuelta: los carveos (todas sus celdas)
    # y los castigos de celdas gigantes
    cast_x, cast_y, cast_t = [], [], []
    for k in c.carveos:
        for i in range(k["ini"], k["fin"] + 1):
            cast_x.append(i)
            cast_y.append(k["vuelta"])
            cast_t.append(f"castigo en v{k['vuelta']}: celda {i} -> pwm {k['pwm']}")
    for g in c.gigante_castigos:
        cast_x.append(g["celda"])
        cast_y.append(g["vuelta"])
        cast_t.append(
            f"gigante castigada en v{g['vuelta']}: "
            f"{g['antes']}→{g['despues']}"
        )
    fig.add_trace(
        go.Scatter(
            x=cast_x, y=cast_y, mode="markers", name="castigo en esa vuelta",
            marker=dict(symbol="x-thin", size=6,
                        line=dict(color=COL_CRITICO, width=1.5)),
            text=cast_t, hovertemplate="%{text}<extra></extra>",
        )
    )
    # Zonas protegidas al empezar cada vuelta (las líneas PROTEGIDA del
    # bloque [VUELTA v], que es el que ABRE la vuelta v)
    prot_x, prot_y, prot_t = [], [], []
    for p in c.protecciones:
        for i in range(p["ini"], p["fin"] + 1):
            prot_x.append(i)
            prot_y.append(p["vuelta"])
            prot_t.append(
                f"v{p['vuelta']}: celda {i} protegida "
                f"(zona [{p['ini']}-{p['fin']}], último derrape en "
                f"v{p['ultimo_derrape']})"
            )
    fig.add_trace(
        go.Scatter(
            x=prot_x, y=prot_y, mode="markers", name="celda protegida (no sube)",
            marker=dict(symbol="circle-open", size=5, color=COL_TINTA_2),
            text=prot_t, hovertemplate="%{text}<extra></extra>",
        )
    )
    fig.update_xaxes(title_text="celda de la cadena")
    fig.update_yaxes(title_text="vuelta",
                     range=[vueltas[-1] + 0.5, vueltas[0] - 0.5], dtick=2)
    alto = max(500, 150 + 20 * len(vueltas))
    return _layout_base(
        fig,
        f"6b · Perfil de PWM con el que se corrió cada vuelta "
        f"(claro={v_min:.0f}, oscuro={v_max:.0f})",
        alto,
    )
# ===========================================================================
# FIGURA 3: distancia perpendicular de la trasera frente a su celda
# ===========================================================================
# La serie con la que trabaja la máquina de derrapes: d(celda) de la pegatina
# trasera. Slider para aislar cada vuelta; de fondo (gris) todas las vueltas.
# Los puntos con derrape abierto van en rojo. La línea negra es el umbral.
# ===========================================================================
def grafica_3_dist(c: Carrera, vueltas):
    umbral = c.params.get("umbral_derrape", 8.0)
    fig = go.Figure()
    fig.add_trace(
        go.Scatter(
            x=c.frames["c_b"], y=c.frames["d_b"], mode="markers",
            name="todas las vueltas",
            marker=dict(size=3, color=COL_CONTEXTO), opacity=0.5,
            hoverinfo="skip",
        )
    )
    for v in vueltas:
        fr = c.frames[c.frames["vuelta"] == v].sort_values("frame")
        # None donde la trasera salta mucho: que la línea no cruce la figura
        xs, ys, textos = [], [], []
        c_prev = None
        for _, r in fr.iterrows():
            if r["c_b"] < 0 or pd.isna(r["d_b"]):
                continue
            if c_prev is not None and abs(r["c_b"] - c_prev) > 5:
                xs.append(None); ys.append(None); textos.append("")
            xs.append(r["c_b"]); ys.append(r["d_b"])
            textos.append(f"frame {int(r['frame'])} · celda {int(r['c_b'])} · "
                          f"d={r['d_b']:.1f} px · pwm={int(r['pwm'])}")
            c_prev = r["c_b"]
        derr = fr[fr["derrapando"]]
        visible = v == vueltas[0]
        fig.add_trace(
            go.Scatter(
                x=xs, y=ys, mode="lines+markers", name=f"vuelta {v}",
                line=dict(color=COL_SERIE_1, width=1.5), marker=dict(size=4),
                text=textos, hovertemplate="%{text}<extra></extra>",
                visible=visible,
            )
        )
        fig.add_trace(
            go.Scatter(
                x=derr["c_b"], y=derr["d_b"], mode="markers",
                name="derrape abierto",
                marker=dict(symbol="x", size=7, color=COL_CRITICO),
                text=[f"frame {int(f)}" for f in derr["frame"]],
                hovertemplate="derrapando · %{text}<br>celda=%{x} "
                              "d=%{y:.1f}<extra></extra>",
                visible=visible,
            )
        )
    fig.add_hline(y=umbral, line=dict(color=COL_TINTA, width=1),
                  annotation_text=f"umbral_derrape = {umbral:.0f} px",
                  annotation_font_color=COL_TINTA_2)

    # Slider: la traza 0 (fondo gris) siempre visible; por cada vuelta se
    # activan sus dos trazas (línea azul + aspas rojas)
    pasos = []
    for i, v in enumerate(vueltas):
        visibles = [True] + [False] * (2 * len(vueltas))
        visibles[1 + 2 * i] = visibles[2 + 2 * i] = True
        pasos.append(dict(label=str(v), method="update",
                          args=[{"visible": visibles}]))
    fig.update_layout(
        sliders=[dict(active=0, currentvalue=dict(prefix="Vuelta "),
                      pad=dict(t=30), steps=pasos)],
    )
    fig.update_xaxes(title_text="celda de la pegatina trasera")
    fig.update_yaxes(title_text="distancia perpendicular (px)",
                     rangemode="tozero")
    return _layout_base(
        fig,
        "3 · Distancia perpendicular de la pegatina trasera a la trayectoria, "
        "vuelta a vuelta",
        520,
    )


# ===========================================================================
# FIGURA 6c: resumen por vuelta (4 paneles)
# ===========================================================================
#   1. nº de derrapes por vuelta (barras rojas: son el evento "malo")
#   2. tiempo por vuelta (marcador verde = al cerrarla el perfil subió,
#      gris = no subió; la última vuelta no tiene cierre y no aparece)
#   3. PWM medio: del perfil (toda la cadena) y el realmente aplicado
#      en los frames de la vuelta (donde pasó el coche)
#   4. velocidad media medida (px/s, filtrando saltos y huecos)
# ===========================================================================
def grafica_6c_resumen(tabla: pd.DataFrame):
    fig = make_subplots(
        rows=2, cols=2, vertical_spacing=0.16, horizontal_spacing=0.10,
        subplot_titles=[
            "derrapes por vuelta", "tiempo por vuelta (s)",
            "PWM medio", "velocidad media (px/s)",
        ],
    )
    fig.add_trace(
        # Los paneles con una sola serie no necesitan entrada de leyenda (el
        # título del panel ya la nombra): solo se listan las dos series de PWM
        go.Bar(x=tabla["vuelta"], y=tabla["n_derrapes"], name="derrapes",
               marker_color=COL_CRITICO, showlegend=False,
               hovertemplate="v%{x}: %{y} derrapes<extra></extra>"),
        row=1, col=1,
    )
    colores = [COL_BUENO if s else COL_MUTED for s in tabla["sube"].fillna(False)]
    fig.add_trace(
        go.Scatter(
            x=tabla["vuelta"], y=tabla["duracion"], mode="lines+markers",
            name="tiempo de vuelta", showlegend=False,
            line=dict(color=COL_SERIE_1, width=2),
            marker=dict(size=8, color=colores),
            text=["perfil subió al cerrarla" if s else "el perfil no subió"
                  for s in tabla["sube"].fillna(False)],
            hovertemplate="v%{x}: %{y:.2f} s<br>%{text}<extra></extra>",
        ),
        row=1, col=2,
    )
    fig.add_trace(
        go.Scatter(x=tabla["vuelta"], y=tabla["pwm_perfil_medio"],
                   mode="lines+markers", name="perfil (media de celdas)",
                   line=dict(color=COL_SERIE_2, width=2), marker=dict(size=6),
                   hovertemplate="v%{x}: %{y:.1f}<extra>perfil</extra>"),
        row=2, col=1,
    )
    fig.add_trace(
        go.Scatter(x=tabla["vuelta"], y=tabla["pwm_aplicado_medio"],
                   mode="lines+markers", name="aplicado en carrera",
                   line=dict(color=COL_SERIE_1, width=2), marker=dict(size=6),
                   hovertemplate="v%{x}: %{y:.1f}<extra>aplicado</extra>"),
        row=2, col=1,
    )
    fig.add_trace(
        go.Scatter(x=tabla["vuelta"], y=tabla["vel_media"],
                   mode="lines+markers", name="velocidad media",
                   showlegend=False,
                   line=dict(color=COL_SERIE_1, width=2), marker=dict(size=6),
                   hovertemplate="v%{x}: %{y:.0f} px/s<extra></extra>"),
        row=2, col=2,
    )
    for fila, col in [(1, 1), (1, 2), (2, 1), (2, 2)]:
        fig.update_xaxes(title_text="vuelta", dtick=5, row=fila, col=col)
    fig.update_yaxes(rangemode="tozero", row=1, col=1)
    fig = _layout_base(fig, "6c · Resumen por vuelta", 640)
    # La leyenda (solo las dos series de PWM) centrada entre los títulos de
    # los dos paneles de arriba, que están a x~0.22 y x~0.78
    fig.update_layout(legend=dict(x=0.5, xanchor="center"))
    return fig


# Un panel de 6c aislado, para poder exportarlo a PDF por separado (pedido:
# en la memoria interesa poder incluir cada subplot suelto). Mismas trazas
# que su panel en grafica_6c_resumen, pero como figura independiente.
PANELES_6C = ("derrapes", "tiempo", "pwm", "velocidad")


def grafica_6c_subplot(tabla: pd.DataFrame, panel: str):
    if panel not in PANELES_6C:
        raise ValueError(f"panel debe ser uno de {PANELES_6C}, no {panel!r}")
    fig = go.Figure()
    if panel == "derrapes":
        fig.add_trace(go.Bar(
            x=tabla["vuelta"], y=tabla["n_derrapes"], marker_color=COL_CRITICO,
            hovertemplate="v%{x}: %{y} derrapes<extra></extra>",
        ))
        fig.update_yaxes(rangemode="tozero")
        titulo = "derrapes por vuelta"
    elif panel == "tiempo":
        colores = [COL_BUENO if s else COL_MUTED for s in tabla["sube"].fillna(False)]
        fig.add_trace(go.Scatter(
            x=tabla["vuelta"], y=tabla["duracion"], mode="lines+markers",
            line=dict(color=COL_SERIE_1, width=2), marker=dict(size=8, color=colores),
            text=["perfil subió al cerrarla" if s else "el perfil no subió"
                  for s in tabla["sube"].fillna(False)],
            hovertemplate="v%{x}: %{y:.2f} s<br>%{text}<extra></extra>",
        ))
        titulo = "tiempo por vuelta (s)"
    elif panel == "pwm":
        fig.add_trace(go.Scatter(x=tabla["vuelta"], y=tabla["pwm_perfil_medio"],
                                 mode="lines+markers", name="perfil (media de celdas)",
                                 line=dict(color=COL_SERIE_2, width=2), marker=dict(size=6)))
        fig.add_trace(go.Scatter(x=tabla["vuelta"], y=tabla["pwm_aplicado_medio"],
                                 mode="lines+markers", name="aplicado en carrera",
                                 line=dict(color=COL_SERIE_1, width=2), marker=dict(size=6)))
        titulo = "PWM medio"
    else:  # velocidad
        fig.add_trace(go.Scatter(
            x=tabla["vuelta"], y=tabla["vel_media"], mode="lines+markers",
            line=dict(color=COL_SERIE_1, width=2), marker=dict(size=6),
            hovertemplate="v%{x}: %{y:.0f} px/s<extra></extra>",
        ))
        titulo = "velocidad media (px/s)"
    fig.update_xaxes(title_text="vuelta", dtick=5)
    return _layout_base(fig, f"6c · {titulo}", 420)


# ===========================================================================
# FIGURA 4 (la más importante): el circuito con el PWM de cada celda
# ===========================================================================
# Todas las cámaras en el MISMO plano (plano global, ver OFFSETS_CAMARAS):
# la trayectoria de fondo en gris tenue y encima cada celda como un punto
# con su PWM escrito DENTRO como número (pedido por Lois: los cambios de
# valor se ven mucho mejor en el número que en un color). El anillo rojo
# marca las celdas dentro de una zona de derrape al EMPEZAR la vuelta del
# slider. Slider INSTANTÁNEO: cada paso enseña el estado de esa vuelta.
#
# datos_camaras: {camara: {"c": Carrera, "vueltas": [v...],
#                          "perfil_vuelta": {v: array PWM por celda},
#                          "zonas_ini_vuelta": {v: [zona]}}}
# ===========================================================================
def grafica_4_circuito(datos_camaras):
    camaras = sorted(datos_camaras)
    # Eje de vueltas del slider: la unión de las vueltas de todas las
    # cámaras (con varias cámaras alguna puede perderse una vuelta)
    vueltas_global = sorted({v for d in datos_camaras.values() for v in d["vueltas"]})

    fig = go.Figure()
    n_estaticas = 0  # trazas siempre visibles (fondos), van primero

    # --- Fondos por cámara: trayectoria gris + huecos tapados (gigantes) ---
    for i, cam in enumerate(camaras):
        dx, dy = offset_camara(cam, i)
        celdas = datos_camaras[cam]["c"].celdas.sort_values("celda").reset_index(drop=True)
        tx, ty = [], []
        gx, gy, gt = [], [], []
        for j, fila in celdas.iterrows():
            if fila["gigante"]:
                # La trayectoria se corta en la gigante y el hueco se dibuja
                # aparte como línea ámbar discontinua entre sus vecinas
                tx.append(None)
                ty.append(None)
                ant = celdas.iloc[j - 1] if j > 0 else None
                sig = celdas.iloc[j + 1] if j + 1 < len(celdas) else None
                if ant is not None and sig is not None and not ant["gigante"] \
                        and not sig["gigante"]:
                    gx += [ant["x"] + dx, sig["x"] + dx, None]
                    gy += [ant["y"] + dy, sig["y"] + dy, None]
                    texto = (f"{cam} · celda gigante {int(fila['celda'])}: "
                             f"hueco tapado de {fila['long_px']:.0f} px")
                    gt += [texto, texto, ""]
            else:
                tx.append(fila["x"] + dx)
                ty.append(fila["y"] + dy)
        # En cadena cerrada se une la última celda con la primera
        normales = celdas[~celdas["gigante"]]
        if datos_camaras[cam]["c"].cerrada and len(normales) > 1:
            tx.append(normales.iloc[0]["x"] + dx)
            ty.append(normales.iloc[0]["y"] + dy)
        fig.add_trace(go.Scatter(
            x=tx, y=ty, mode="lines", name=f"trayectoria {cam}",
            line=dict(color=COL_GRID, width=2), hoverinfo="skip",
            showlegend=(i == 0), legendgroup="tray",
        ))
        fig.add_trace(go.Scatter(
            x=gx, y=gy, mode="lines", name="hueco tapado (gigante)",
            line=dict(color=COL_AVISO, width=2, dash="dash"),
            text=gt, hovertemplate="%{text}<extra></extra>",
            showlegend=(i == 0 and len(gx) > 0), legendgroup="gigante",
        ))
        n_estaticas += 2

    # --- Un bloque de trazas por vuelta (una traza por cámara) ---
    for v in vueltas_global:
        for i, cam in enumerate(camaras):
            d = datos_camaras[cam]
            dx, dy = offset_camara(cam, i)
            normales = d["c"].celdas[~d["c"].celdas["gigante"]].sort_values("celda")
            # Si esta cámara no tiene datos de la vuelta v se usa la última
            # vuelta anterior que sí tenga (el perfil sigue vigente)
            v_datos = max((u for u in d["vueltas"] if u <= v), default=None)
            xs, ys, textos, hovers, anchos, colores_borde = [], [], [], [], [], []
            if v_datos is not None:
                perfil = d["perfil_vuelta"][v_datos]
                zonas = d["zonas_ini_vuelta"].get(v_datos, [])
                for _, cd in normales.iterrows():
                    idx = int(cd["celda"])
                    pwm = perfil[idx] if idx < len(perfil) and not np.isnan(perfil[idx]) else None
                    en_zona = any(z["tipo"] == "derrape" and z["ini"] <= idx <= z["fin"]
                                  for z in zonas)
                    xs.append(cd["x"] + dx)
                    ys.append(cd["y"] + dy)
                    textos.append("" if pwm is None else f"{pwm:.0f}")
                    anchos.append(2.5 if en_zona else 1)
                    colores_borde.append(COL_CRITICO if en_zona else COL_CONTEXTO)
                    hovers.append(
                        f"{cam} · celda {idx}<br>PWM {pwm:.0f}" if pwm is not None
                        else f"{cam} · celda {idx}<br>PWM ?"
                    )
                    if en_zona:
                        hovers[-1] += "<br><b>dentro de zona de derrape</b>"
            fig.add_trace(go.Scatter(
                x=xs, y=ys, mode="markers+text", name=f"celdas {cam}",
                # El número va DENTRO del punto: el marcador es un disco
                # claro con borde (rojo si la celda está en zona de derrape)
                marker=dict(size=17, color=COL_SUPERFICIE,
                            line=dict(color=colores_borde, width=anchos)),
                text=textos, textposition="middle center",
                textfont=dict(size=8, color=COL_TINTA),
                customdata=hovers, hovertemplate="%{customdata}<extra></extra>",
                visible=(v == vueltas_global[0]), showlegend=False,
            ))

    # Slider instantáneo: fondos siempre visibles + el bloque de su vuelta
    pasos = []
    n_cam = len(camaras)
    for k, v in enumerate(vueltas_global):
        visibles = [True] * n_estaticas + [False] * (len(vueltas_global) * n_cam)
        for i in range(n_cam):
            visibles[n_estaticas + k * n_cam + i] = True
        pasos.append(dict(label=str(v), method="update",
                          args=[{"visible": visibles}]))
    fig.update_layout(
        sliders=[dict(active=0, currentvalue=dict(prefix="Vuelta "),
                      pad=dict(t=30), steps=pasos)],
    )
    # Coordenadas de imagen: origen arriba a la izquierda (Y invertida) y
    # misma escala en ambos ejes para que el circuito no salga deformado
    fig.update_yaxes(autorange="reversed", scaleanchor="x", scaleratio=1,
                     title_text="y (px del plano global)")
    fig.update_xaxes(title_text="x (px del plano global)")
    return _layout_base(
        fig,
        "4 · El circuito con el PWM de cada celda "
        "(número = PWM del perfil; anillo rojo = celda dentro de una zona "
        "de derrape)",
        720,
    )


# ---------------------------------------------------------------------------
# Colores de las trayectorias de las pegatinas (los de las figuras gnuplot
# de la memoria de Mario, para que las de esta memoria se parezcan)
# ---------------------------------------------------------------------------
COL_ETIQ_DELANTERA = "#009E73"  # verde etiqueta delantera (fig. 5.17)
COL_ETIQ_TRASERA = "#CC00CC"    # magenta etiqueta trasera (fig. 5.17)
COL_TRAY_BASE_3D = "#9400D3"    # violeta de la línea "Posicion" (fig. 5.12)

# Salto en píxeles entre dos posiciones consecutivas a partir del cual la
# línea de trayectoria se corta (None) en vez de unirlas: o es un cambio de
# cámara, o una detección falsa, o el coche reapareció por otro lado
SALTO_CORTE_TRAZO = 100.0

# Altura de la caja 3D de la figura 1 respecto al lado mayor del circuito.
# El derrape mide decenas de píxeles y el circuito más de mil: a escala real
# el relieve sería invisible, así que el eje z se exagera hasta esta
# fracción (0.35 = un tercio del ancho). Subirlo hace los picos más
# aparatosos; bajarlo, la vista más plana.
ALTURA_RELATIVA_Z = 0.35


def _con_cortes(df, col_x, col_y, camaras):
    """Listas x, y para una traza de línea con None donde no hay que unir:
    cambio de cámara o salto > SALTO_CORTE_TRAZO px. Aplica el offset del
    plano global de cada cámara. df debe venir ordenado por tiempo."""
    xs, ys = [], []
    cam_prev = x_prev = y_prev = None
    indice = {c: i for i, c in enumerate(camaras)}
    for fila in df.itertuples():
        cam = fila.camara
        dx, dy = offset_camara(cam, indice.get(cam, 0))
        x, y = getattr(fila, col_x) + dx, getattr(fila, col_y) + dy
        if cam_prev is not None and (
                cam != cam_prev
                or abs(x - x_prev) + abs(y - y_prev) > SALTO_CORTE_TRAZO):
            xs.append(None)
            ys.append(None)
        xs.append(x)
        ys.append(y)
        cam_prev, x_prev, y_prev = cam, x, y
    return xs, ys


# ===========================================================================
# FIGURA 1: evolución 3D del derrape sobre la trayectoria (fig. 5.12 de
# la memoria de Mario)
# ===========================================================================
# Plano XY = el plano global en píxeles, con la trayectoria de la pegatina
# DELANTERA como línea base en z=0 ("Posicion" en la figura de Mario).
# Eje Z y color = distancia de derrape de la pegatina TRASERA en cada punto
# (el dist_derrape de la telemetría emparejado con su posición).
#
# Slider ACUMULATIVO: el paso "inicio" enseña solo la línea base; el paso v
# superpone los puntos de las vueltas 1..v sin borrar los anteriores, para
# ver cómo van creciendo las zonas de derrape con las vueltas. La vista se
# puede rotar/orbitar con el ratón (scatter3d).
#
# df: DataFrame ordenado por t con columnas
#     [t, vuelta, camara, fx, fy, bx, by, dist]  (dist NaN si la telemetría
#     no se emparejó con esa posición; esos puntos no se dibujan en 3D)
# ===========================================================================
def grafica_1_derrape3d(df):
    camaras = sorted(df["camara"].unique())
    vueltas = sorted(df.loc[df["vuelta"].notna(), "vuelta"].unique())
    indice = {c: i for i, c in enumerate(camaras)}

    fig = go.Figure()

    # Línea base: la trayectoria de la delantera durante UNA vuelta completa
    # (la primera con datos), que dibuja el circuito limpio en z=0. Si no
    # hay vueltas cronometradas se usa todo el recorrido.
    base = df[df["vuelta"] == vueltas[0]] if vueltas else df
    bx, by = _con_cortes(base, "fx", "fy", camaras)
    fig.add_trace(go.Scatter3d(
        x=bx, y=by, z=[0] * len(bx), mode="lines", name="Posición",
        line=dict(color=COL_TRAY_BASE_3D, width=3), hoverinfo="skip",
    ))

    # Un bloque de puntos por vuelta (pegatina trasera, z = dist derrape)
    con_dist = df[df["dist"].notna() & df["vuelta"].notna()]
    z_max = max(float(con_dist["dist"].max()) if len(con_dist) else 1.0, 1.0)
    for v in vueltas:
        fr = con_dist[con_dist["vuelta"] == v]
        xs = [f.bx + offset_camara(f.camara, indice[f.camara])[0]
              for f in fr.itertuples()]
        ys = [f.by + offset_camara(f.camara, indice[f.camara])[1]
              for f in fr.itertuples()]
        fig.add_trace(go.Scatter3d(
            x=xs, y=ys, z=fr["dist"], mode="markers",
            name=f"vuelta {int(v)}", showlegend=False,
            marker=dict(
                size=2.5, color=fr["dist"],
                # Rampa perceptual negro->morado->naranja->amarillo, como la
                # paleta de la figura gnuplot de Mario (Dist_derrape 0..14)
                colorscale="Inferno", cmin=0, cmax=z_max,
                colorbar=dict(title="Dist<br>derrape"),
            ),
            text=[f"v{int(v)} · {f.camara}<br>dist={f.dist:.1f} px"
                  for f in fr.itertuples()],
            hovertemplate="%{text}<extra></extra>",
            visible=False,
        ))

    # Slider acumulativo: paso 0 = solo la base; el paso k enseña las
    # vueltas 1..k (la traza 0, la base, siempre visible)
    pasos = [dict(label="inicio", method="update",
                  args=[{"visible": [True] + [False] * len(vueltas)}])]
    for k, v in enumerate(vueltas):
        visibles = [True] + [i <= k for i in range(len(vueltas))]
        pasos.append(dict(label=str(int(v)), method="update",
                          args=[{"visible": visibles}]))
    # Proporción de la caja 3D: x e y guardan la proporción real del plano
    # (el circuito no sale deformado) y z se lleva ALTURA_RELATIVA_Z del lado
    # mayor. Con aspectmode="data" el eje z (0..~20 px de derrape) quedaría
    # aplastado contra un plano de más de mil píxeles de ancho y no se vería
    # el relieve, que es justo lo que cuenta esta figura.
    rango_x = float(df["fx"].max() - df["fx"].min()) or 1.0
    rango_y = float(df["fy"].max() - df["fy"].min()) or 1.0
    # A los rangos hay que sumarles la separación entre cámaras en el plano
    # global (cada una entra desplazada por su offset)
    desplazamientos = [offset_camara(c, i) for i, c in enumerate(camaras)]
    rango_x += max(d[0] for d in desplazamientos) - min(d[0] for d in desplazamientos)
    rango_y += max(d[1] for d in desplazamientos) - min(d[1] for d in desplazamientos)
    mayor = max(rango_x, rango_y)

    fig.update_layout(
        sliders=[dict(active=0, currentvalue=dict(prefix="Hasta la vuelta "),
                      pad=dict(t=10), steps=pasos)],
        scene=dict(
            xaxis_title="x (px del plano global)",
            yaxis_title="y (px del plano global)",
            zaxis_title="dist. derrape (px)",
            # Y invertida para que el circuito se vea como en la imagen de
            # la cámara. nticks bajo: el eje y es el lado corto de la caja y
            # sus marcas se amontonarían unas sobre otras
            yaxis=dict(autorange="reversed", nticks=5),
            xaxis=dict(nticks=8),
            aspectmode="manual",
            aspectratio=dict(x=rango_x / mayor, y=rango_y / mayor,
                             z=ALTURA_RELATIVA_Z),
            # Punto de vista inicial parecido al de la figura 5.12 de Mario:
            # en diagonal y algo elevado, para ver a la vez el trazado y el
            # relieve del derrape. Cuanto más pequeño el vector eye, más
            # cerca queda la cámara (una caja alargada como esta se ve
            # diminuta con el valor por defecto, 1.25 en los tres ejes). En
            # pantalla se puede rotar a gusto: esto es solo el encuadre de
            # partida, que además es el que sale en el PDF.
            camera=dict(eye=dict(x=1.0, y=1.0, z=0.6)),
        ),
    )
    fig = _layout_base(
        fig,
        "1 · Valor de derrape en cada punto del recorrido "
        "(acumulado hasta la vuelta del slider; arrastrar para rotar)",
        720,
    )
    # Los márgenes de _layout_base son para figuras 2D (dejan sitio al
    # título del eje Y); en 3D la escena los desaprovecha y sale pequeña
    fig.update_layout(margin=dict(l=10, r=10, t=70, b=10))
    return fig


# ===========================================================================
# FIGURA 2: trayectorias de la etiqueta delantera y trasera (fig. 5.17 de
# la memoria de Mario)
# ===========================================================================
# Vista 2D del plano global con las dos trayectorias superpuestas, cada una
# con su color y su leyenda. Slider INSTANTÁNEO (decidido con Lois): cada
# paso enseña SOLO la vuelta seleccionada, así se ve cómo cambia la
# trayectoria al pasar de una vuelta a la siguiente (dónde se dispersa y
# dónde se estabiliza).
#
# df: el mismo DataFrame de grafica_1_derrape3d.
# ===========================================================================
def grafica_2_trayectorias(df):
    camaras = sorted(df["camara"].unique())
    vueltas = sorted(df.loc[df["vuelta"].notna(), "vuelta"].unique())

    fig = go.Figure()
    # Dos trazas "fantasma" siempre visibles que sostienen la leyenda: si la
    # llevaran las trazas de una vuelta, la leyenda desaparecería al mover
    # el slider a otra vuelta
    series = [
        ("bx", "by", "Etiq. trasera", COL_ETIQ_TRASERA),
        ("fx", "fy", "Etiq. delantera", COL_ETIQ_DELANTERA),
    ]
    for _, _, nombre, color in series:
        fig.add_trace(go.Scatter(
            x=[None], y=[None], mode="lines", name=nombre,
            line=dict(color=color, width=1.5), legendgroup=nombre,
            showlegend=True, hoverinfo="skip",
        ))
    for v in vueltas:
        fr = df[df["vuelta"] == v]
        for col_x, col_y, nombre, color in series:
            xs, ys = _con_cortes(fr, col_x, col_y, camaras)
            fig.add_trace(go.Scatter(
                x=xs, y=ys, mode="lines", name=nombre,
                line=dict(color=color, width=1.5),
                legendgroup=nombre, showlegend=False,
                hovertemplate=f"{nombre} · v{int(v)}<br>" +
                              "(%{x:.0f}, %{y:.0f})<extra></extra>",
                visible=(v == vueltas[0]),
            ))

    # Slider instantáneo: dos trazas (trasera + delantera) por vuelta, tras
    # las dos fantasma de la leyenda (siempre visibles)
    pasos = []
    for k, v in enumerate(vueltas):
        visibles = [True, True] + [False] * (2 * len(vueltas))
        visibles[2 + 2 * k] = visibles[3 + 2 * k] = True
        pasos.append(dict(label=str(int(v)), method="update",
                          args=[{"visible": visibles}]))
    fig.update_layout(
        sliders=[dict(active=0, currentvalue=dict(prefix="Vuelta "),
                      pad=dict(t=30), steps=pasos)],
    )
    # Coordenadas de imagen: Y invertida y escala 1:1
    fig.update_yaxes(autorange="reversed", scaleanchor="x", scaleratio=1,
                     title_text="y (px del plano global)")
    fig.update_xaxes(title_text="x (px del plano global)")
    return _layout_base(
        fig,
        "2 · Trayectorias de la etiqueta delantera y trasera "
        "(una vuelta cada vez)",
        680,
    )
