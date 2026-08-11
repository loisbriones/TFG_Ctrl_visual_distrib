#!/usr/bin/env python3
"""
Genera el HTML de analisis de TODAS las ejecuciones del capitulo de pruebas y
los deja ordenados por seccion en RESULTADOS_PRUEBAS/, listos para revisar o
para subir al repositorio junto a la memoria.

Es el hermano de videos/generar_videos.py: mismas ejecuciones, mismo orden y
mismos nombres de fichero (el .html de una prueba se llama igual que su .mp4),
solo que en vez de renderizar el video llama al dashboard de analisis. De
hecho le IMPORTA el catalogo: la correspondencia bag -> fila de la tabla de la
memoria se escribe una sola vez y vive alli.

    analisis/env/bin/python analisis/generar_htmls.py
    analisis/env/bin/python analisis/generar_htmls.py --seccion 7.5
    analisis/env/bin/python analisis/generar_htmls.py --rehacer

El trabajo de verdad lo hace analisis/run.sh, que levanta el contenedor y deja
el HTML autocontenido junto al bag (<bag>_analisis.html); aqui solo se decide
que logs se le pasan, se copia el resultado a su carpeta y se escribe el
indice. Tiene que ir por el contenedor porque lectura_bag.py importa
rosbag2_py, que no esta en el entorno local.

Se lanza con --sin-servidor, asi que los HTML salen sin botones de PDF: esos
solo funcionan con el servidor levantado y aqui lo que se quiere es el archivo.
Para ver lo que vieron las camaras estan los mp4 de videos/salida/.
"""

import argparse
import html
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

RAIZ = Path(__file__).resolve().parent.parent
BAGS = RAIZ / "GRABACIONES" / "bags"
SALIDA = RAIZ / "RESULTADOS_PRUEBAS"
RUN_SH = RAIZ / "analisis" / "run.sh"

sys.path.insert(0, str(RAIZ / "videos"))
from generar_videos import CATALOGO  # noqa: E402
# Se importa con otro nombre porque "carpeta_bag" es ya el parametro de las dos
# funciones de abajo y taparia al import dentro de ellas.
from generar_videos import carpeta_bag as ruta_del_bag  # noqa: E402
from generar_videos import mp4_de  # noqa: E402

# Nombre de la carpeta de cada seccion. El numero delante para que el listado
# salga en el orden del capitulo y no en orden alfabetico.
CARPETAS = {
    "7.1": "7.1_comparativa_entre_coches",
    "7.2": "7.2_pwm_incremental",
    "7.3": "7.3_ajuste_de_parametros",
    "7.4": "7.4_algoritmo_vs_pilotos",
    "7.5": "7.5_politica_optima",
    "7.6": "7.6_red",
    "7.7": "7.7_circuitos_grandes",
    "7.8": "7.8_multiples_coches",
    "7.9": "7.9_peraltes",
}


def _subcarpeta(seccion, bag):
    """Las dos secciones que tienen bastantes ejecuciones como para que la
    carpeta plana estorbe (19 y 18). El corte es el mismo que hace la memoria:
    la 7.4 separa los dos circuitos y la 7.6 separa los escenarios medidos tal
    cual (A) del barrido de retardo inyectado con tc netem (B)."""
    if seccion == "7.4":
        # Los bags del circuito en ocho lo llevan en el nombre, o como "OCHO"
        # (los del algoritmo) o como "_8_" (los manuales de aquel dia).
        return "circuito_ocho" if "OCHO" in bag or "_8_" in bag else "circuito_ovalo"
    if seccion == "7.6":
        return "B_retardo_inyectado" if bag.startswith("B_") else "A_escenarios"
    return ""


def logs_del_coche(carpeta_bag: Path, coche: str):
    """Los derrapesLog_*.txt que hay que pasarle al dashboard para ese coche.

    cargar_logs() indexa los logs POR CAMARA, asi que en un bag de dos coches
    (cuatro logs: dos coches x dos camaras) pasarle la carpeta entera haria que
    el log de un coche pisara el del otro sin avisar. Cuando los nombres llevan
    el coche (derrapesLog_CarController_car1_camara_01.txt) se filtra por el;
    cuando no lo llevan (derrapesLog_CarControllerNode_camara_01.txt, grabado
    cuando aun no habia mas que un coche) se pasan todos, que son suyos.
    """
    todos = sorted(carpeta_bag.glob("derrapesLog_*.txt"))
    con_coche = [p for p in todos if re.search(r"_car\d+_", p.name)]
    if not con_coche:
        return todos
    return [p for p in con_coche if f"_{coche}_" in p.name]


def destino_de(seccion, bag, coche, varios_coches):
    """Donde va el HTML. Mismo nombre que el mp4 de videos/salida/ para poder
    emparejarlos de un vistazo; solo se le pega el coche cuando en el mismo bag
    hay mas de uno y por tanto mas de un HTML."""
    carpeta = SALIDA / CARPETAS[seccion] / _subcarpeta(seccion, bag)
    nombre = f"{bag}_{coche}.html" if varios_coches else f"{bag}.html"
    return carpeta / nombre


def generar(carpeta_bag: Path, coche: str, destino: Path):
    """Llama a run.sh y deja el HTML en su sitio. Devuelve los logs usados.

    El HTML lo escribe analisis.py junto al bag y aqui se COPIA (no se mueve):
    comparativa.py busca los *_analisis.html dentro de la carpeta de los bags,
    y moverlos le quitaria de en medio los que ya estaban."""
    logs = logs_del_coche(carpeta_bag, coche)
    orden = [str(RUN_SH), str(carpeta_bag), *[str(p) for p in logs],
             "--sin-servidor", "--coche", coche]
    inicio = time.time()
    r = subprocess.run(orden, capture_output=True, text=True)
    if r.returncode != 0:
        # La salida del contenedor es lo unico que dice que paso; se sube tal
        # cual al mensaje de fallo para no tener que relanzarlo a mano.
        cola = (r.stdout + r.stderr).strip().splitlines()[-6:]
        raise RuntimeError("run.sh salio con " + str(r.returncode) + ": "
                           + " | ".join(cola))

    escrito = carpeta_bag.parent / f"{carpeta_bag.name}_analisis.html"
    # Que exista no basta: hay 27 bags con un HTML de otro dia al lado. Si
    # run.sh se fue por otro camino y no lo reescribio, se veria aqui.
    if not escrito.is_file() or escrito.stat().st_mtime < inicio:
        raise RuntimeError(f"run.sh no reescribio {escrito.name}")
    destino.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(escrito, destino)
    return logs


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("--seccion", help="Solo una seccion (p. ej. 7.5)")
    # Para rehacer UNA ejecucion sin arrastrar a las demas de su seccion. Hace
    # falta porque los HTML ya publicados no se tocan salvo que se quiera: con
    # --seccion 7.7 --rehacer se rehacen las ocho, y normalmente lo que se
    # quiere es solo la que salio mal.
    parser.add_argument("--bag", help="Solo un bag, por su nombre en el catalogo")
    parser.add_argument("--rehacer", action="store_true",
                        help="Rehace tambien los HTML que ya existan")
    args = parser.parse_args()

    trabajos = [t for t in CATALOGO
                if (not args.seccion or t[0] == args.seccion)
                and (not args.bag or t[1] == args.bag)]
    if not trabajos:
        sys.exit(f"nada que hacer con seccion={args.seccion} bag={args.bag} "
                 f"(secciones: {', '.join(sorted(CARPETAS))})")

    SALIDA.mkdir(parents=True, exist_ok=True)
    fallos = []
    # Un HTML por coche: el dashboard analiza un coche cada vez, asi que los
    # dos bags de la 7.8 dan dos HTML cada uno (el video si los enseña juntos).
    pendientes = [(s, b, e, c, len(coches) > 1)
                  for s, b, e, coches in trabajos for c in coches]
    for n, (seccion, bag, etiqueta, coche, varios) in enumerate(pendientes, 1):
        destino = destino_de(seccion, bag, coche, varios)
        print(f"\n[{n}/{len(pendientes)}] {seccion}  {bag}  {coche}")
        if destino.exists() and not args.rehacer:
            print("  ya existe, se salta (--rehacer para volver a generarlo)")
            continue
        inicio = time.time()
        try:
            logs = generar(ruta_del_bag(bag), coche, destino)
        except Exception as e:  # noqa: BLE001 - un bag roto no puede parar el lote
            print(f"  FALLO: {type(e).__name__}: {e}")
            fallos.append((bag, coche, f"{type(e).__name__}: {e}"))
            continue
        print(f"  -> {destino.relative_to(SALIDA)}  "
              f"{destino.stat().st_size / 1e6:.0f} MB, "
              f"{len(logs)} log(s), en {time.time() - inicio:.0f} s")

    hechos = _escribir_indices()
    print(f"\n{hechos} HTML de los {len(list(_filas()))} del catalogo, en {SALIDA}")
    if fallos:
        print(f"{len(fallos)} fallaron:")
        for bag, coche, motivo in fallos:
            print(f"  - {bag} ({coche}): {motivo}")


def _filas():
    """Una fila por HTML que deberia existir, con lo que hace falta para
    entender el fichero sin abrirlo. Recorre el CATALOGO entero y no solo lo
    que se acaba de generar: con --seccion, una pasada parcial dejaria fuera
    todo lo demas del indice."""
    for seccion, bag, etiqueta, coches in CATALOGO:
        for coche in coches:
            destino = destino_de(seccion, bag, coche, len(coches) > 1)
            logs = (logs_del_coche(ruta_del_bag(bag), coche)
                    if ruta_del_bag(bag).is_dir() else [])
            yield {
                "seccion": seccion,
                "etiqueta": etiqueta,
                "bag": bag,
                "coche": coche,
                "logs": logs,
                "destino": destino,
                "existe": destino.is_file(),
                "vueltas": _vueltas_del_html(destino),
                # mp4_de() y no VIDEOS/<bag>.mp4: los videos ya renderizados se
                # han ido repartiendo a mano en subcarpetas de salida/, asi que
                # mirando solo la raiz la columna "Video" saldria casi vacia
                "mp4": mp4_de(bag),
            }


def _vueltas_del_html(fichero: Path):
    """Cuantas vueltas cronometradas trae ese analisis, o None si no se sabe.

    Se lee del pie de la seccion 0, que el propio dashboard escribe ("... · N
    vueltas cronometradas · ..."). Va en la cabecera del fichero, asi que se
    leen solo los primeros 100 kB y da igual que el HTML pese 35 MB. Es el dato
    que separa un analisis vacio de uno completo: si el bag no trae
    time_per_lap, TODAS las figuras van por vuelta y salen en blanco."""
    if not fichero.is_file():
        return None
    with open(fichero, encoding="utf-8") as f:
        m = re.search(r"(\d+) vueltas cronometradas", f.read(100_000))
    return int(m.group(1)) if m else None


# --------------------------------------------------------------------------
# Los textos del indice. Van aqui, juntos, porque los dos indices (md y html)
# escriben exactamente lo mismo y no tiene sentido mantenerlo por duplicado.
# --------------------------------------------------------------------------
INTRO = ("Cada fichero de la tabla es el analisis completo de UNA ejecucion: se "
         "abre con doble clic, no necesita nada instalado y funciona sin "
         "conexion. Todas las graficas son interactivas: se puede ampliar una "
         "zona arrastrando, volver con doble clic y ver el valor de cada punto "
         "pasando el raton por encima.")

# Que ensena cada seccion del HTML. Quien llega desde la memoria no conoce la
# herramienta, asi que el indice tiene que decirselo antes de abrir el primero.
SECCIONES_HTML = [
    ("1", "El derrape en 3D sobre el trazado: la altura y el color son cuanto "
          "se aparta la cola del coche de su trayectoria.", "una vuelta cada vez"),
    ("2", "El recorrido de las dos pegatinas sobre la trayectoria aprendida en "
          "la calibracion, con la distancia que mide el algoritmo.",
     "una vuelta cada vez, y checkboxes para quitar capas"),
    ("3", "La distancia de derrape a lo largo de la vuelta, con la tension "
          "(PWM) que se estaba aplicando en cada instante.",
     "una vuelta cada vez, tambien con las flechas del teclado"),
    ("4", "El circuito con las zonas de tension del algoritmo: se ve como "
          "nacen, crecen, se fusionan y desaparecen vuelta a vuelta.",
     "una vuelta cada vez"),
    ("5", "Los tiempos de las vueltas, su tendencia y el derrape frente al "
          "tiempo.", None),
    ("6", "El resumen de la carrera: derrapes, tiempo, tension media y "
          "velocidad media, vuelta a vuelta.", None),
]

# Sin el log del algoritmo el dashboard no puede dibujar las zonas de PWM ni la
# trayectoria base, y quien abra uno de esos HTML tiene que saber que no esta
# roto.
SIN_LOG = ("Sin log del algoritmo el dashboard no puede reconstruir las zonas "
           "de PWM: a esos analisis les faltan las secciones 4 y 6 y la "
           "trayectoria base de la 2. El resto (posiciones, derrape y tiempos "
           "por vuelta) sale del bag y esta completo.")
SIN_VIDEO = ("Para ver lo que vieron las camaras estan los videos, uno por "
             "analisis y con el mismo nombre: el dashboard ya no los lleva.")

# La tabla la construye el CATALOGO. La unica seccion del capitulo que no
# aparece es la 7.8, y conviene decir por que antes de que alguien la busque.
FUERA_DEL_INDICE = (
    "La seccion 7.8 (nodo de visualizacion y multiples coches) no tiene "
    "analisis publicado: en las dos grabaciones con dos coches a la vez la "
    "deteccion fallo (las camaras dieron por coche cosas que no lo eran) y las "
    "cifras que saldrian no son las de lo que paso en la pista. Los bags se "
    "conservan para contrastar, no como resultado.")


def _escribir_indices():
    filas = [f for f in _filas() if f["existe"]]
    _indice_md(filas)
    _indice_html(filas)
    return len(filas)


def _indice_md(filas):
    lineas = [
        "# Analisis de las pruebas de la memoria",
        "",
        "Un HTML por cada ejecucion del capitulo de pruebas de la memoria. La",
        "columna de seccion es la del capitulo; la de ejecucion, la fila de la",
        "tabla a la que corresponde.",
        "",
        INTRO,
        "",
        "## Que hay dentro de cada analisis",
        "",
    ]
    for num, que, control in SECCIONES_HTML:
        mando = f" *({control}.)*" if control else ""
        lineas.append(f"{num}. {que}{mando}")
    lineas += [
        "",
        SIN_LOG,
        "",
        SIN_VIDEO,
        "",
        FUERA_DEL_INDICE,
        "",
        "## Las ejecuciones",
        "",
        "| Seccion | Ejecucion | Fichero | Vueltas | Log | Tamaño |",
        "|---|---|---|---|---|---|",
    ]
    for f in filas:
        rel = f["destino"].relative_to(SALIDA)
        log = f"{len(f['logs'])} camara(s)" if f["logs"] else "**sin log**"
        vueltas = "?" if f["vueltas"] is None else str(f["vueltas"])
        mb = f["destino"].stat().st_size / 1e6
        lineas.append(f"| {f['seccion']} | {f['etiqueta']} | [`{rel}`]({rel}) | "
                      f"{vueltas} | {log} | {mb:.0f} MB |")
    faltan = [f for f in _filas() if not f["existe"]]
    if faltan:
        lineas += ["", "Sin generar:", ""]
        lineas += [f"- `{f['bag']}` ({f['coche']})" for f in faltan]
    (SALIDA / "INDICE.md").write_text("\n".join(lineas) + "\n", encoding="utf-8")


def _indice_html(filas):
    """El mismo indice con enlaces, que es como se abre esto de verdad: se hace
    doble clic en INDICE.html y desde ahi se navega a cada analisis."""
    partes = [
        "<!DOCTYPE html><html lang='es'><head><meta charset='utf-8'>",
        "<title>Analisis de las pruebas de la memoria</title>",
        "<style>",
        "body{font-family:system-ui,sans-serif;margin:2rem auto;max-width:70rem;",
        "     line-height:1.5;color:#222;padding:0 1rem}",
        "h1{margin-bottom:.2rem} h2{margin-top:2rem;border-bottom:2px solid #ddd}",
        "p.nota{color:#555;font-size:.9rem}",
        "table{border-collapse:collapse;width:100%} td,th{padding:.35rem .5rem;",
        "     border-bottom:1px solid #eee;text-align:left;vertical-align:top}",
        "th{font-size:.8rem;text-transform:uppercase;color:#666}",
        ".aviso{color:#a00;font-weight:600} a{color:#06c}",
        "ol.guia{color:#555;font-size:.9rem} ol.guia li{margin-bottom:.35rem}",
        "ol.guia em{color:#777}",
        "</style></head><body>",
        "<h1>Analisis de las pruebas</h1>",
        f"<p class='nota'>{html.escape(INTRO)}</p>",
        "<h2>Que hay dentro de cada analisis</h2>",
        "<ol class='guia'>",
        *[f"<li>{html.escape(que)}"
          + (f" <em>({html.escape(control)}.)</em>" if control else "")
          + "</li>"
          for _, que, control in SECCIONES_HTML],
        "</ol>",
        f"<p class='nota'>{html.escape(SIN_LOG)}</p>",
        f"<p class='nota'>{html.escape(SIN_VIDEO)}</p>",
        f"<p class='nota'>{html.escape(FUERA_DEL_INDICE)}</p>",
    ]
    for seccion in sorted(CARPETAS):
        de_la_seccion = [f for f in filas if f["seccion"] == seccion]
        if not de_la_seccion:
            continue
        titulo = CARPETAS[seccion].split("_", 1)[1].replace("_", " ")
        partes.append(f"<h2>{seccion} · {html.escape(titulo)}</h2>")
        partes.append("<table><tr><th>Ejecucion</th><th>Analisis</th>"
                      "<th>Video</th><th>Vueltas</th><th>Log</th></tr>")
        for f in de_la_seccion:
            rel = f["destino"].relative_to(SALIDA)
            log = (f"{len(f['logs'])} camara(s)" if f["logs"]
                   else "<span class='aviso'>sin log</span>")
            video = (f"<code>{html.escape(f['mp4'].name)}</code>"
                     if f["mp4"] else "—")
            vueltas = ("?" if f["vueltas"] is None else
                       f"<span class='aviso'>0</span>" if f["vueltas"] == 0
                       else str(f["vueltas"]))
            partes.append(
                f"<tr><td>{html.escape(f['etiqueta'])}</td>"
                f"<td><a href='{rel}'>{html.escape(rel.name)}</a></td>"
                f"<td>{video}</td><td>{vueltas}</td><td>{log}</td></tr>")
        partes.append("</table>")
    partes.append("</body></html>")
    (SALIDA / "INDICE.html").write_text("\n".join(partes) + "\n", encoding="utf-8")


if __name__ == "__main__":
    sys.exit(main())
