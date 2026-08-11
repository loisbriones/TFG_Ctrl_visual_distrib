#!/usr/bin/env python3
"""
Los títulos y las explicaciones que salen en la página, todos juntos para
poder reescribirlos sin tocar el código que las coloca (pagina.py).

Criterio: una frase de manejo si la sección tiene controles (slider, flechas,
checkboxes) y dos o tres de qué se está mirando. El porqué de cada decisión
de diseño va en el README, no en la página.

Las claves son las mismas con las que las figuras se registran en FIGURAS,
que son también las que viajan en la URL de /pdf.
"""

TITULOS = {
    "1": "1 · Valor de derrape en cada punto del recorrido",
    "2": "2 · Trayectorias de la etiqueta delantera y trasera",
    "3": "3 · Distancia de derrape del bag, vuelta a vuelta",
    "4": "4 · El circuito con las zonas de PWM",
    "5": "5 · Tiempos por vuelta",
    "6": "6 · Resumen por vuelta",
}

TEXTOS = {
    "1":
        "Cada punto del recorrido de la pegatina delantera, con la distancia "
        "de derrape de la trasera como altura y como color. El slider enseña "
        "una vuelta cada vez y los ejes están fijos, así que dos vueltas se "
        "comparan tal cual. El rombo ámbar es la meta y la flecha de su lado, "
        "el sentido de la marcha; la vista se rota arrastrando con el ratón.",

    "2":
        "El recorrido de las dos pegatinas, una vuelta cada vez: el slider "
        "cambia de vuelta y los checkboxes quitan o ponen cada capa. La "
        "trayectoria base (azul) es la que aprendió la calibración, y los "
        "segmentos rojos van de la trasera hasta ella: son la distancia que "
        "mide el algoritmo. La estrella ámbar marca el inicio de la vuelta.",

    # Sin logs no hay celdas, así que no hay trayectoria base ni perpendiculares
    "2-sin-base":
        "El recorrido de las dos pegatinas, una vuelta cada vez: el slider "
        "cambia de vuelta y los checkboxes quitan o ponen cada capa. La "
        "estrella ámbar marca el inicio de la vuelta. Sin logs del algoritmo "
        "no hay trayectoria base que dibujar, así que tampoco salen las "
        "perpendiculares de derrape.",

    "3":
        "La distancia de derrape que publicó el controlador, muestra a "
        "muestra dentro de una vuelta, con el PWM que se estaba aplicando en "
        "el eje derecho. Con el ratón encima, las flechas ← y → cambian de "
        "vuelta. Cada punto lleva el color de la cámara que lo vio: un pico "
        "estrecho que arranca justo en un cambio de cámara (línea de puntos "
        "vertical) es falso, no un derrape.",

    "3-resumen":
        "La misma serie resumida a un valor por vuelta: el máximo, el "
        "percentil 95 y la mediana del derrape.",

    "4":
        "Todas las cámaras en el mismo plano, con cada celda del color y la "
        "forma de su zona. El slider pasa las vueltas y se ve cómo nacen, "
        "crecen, se fusionan y desaparecen; la leyenda da el tramo de celdas "
        "de cada zona, su PWM y si se movió en esa vuelta. Una zona partida "
        "entre dos cámaras sale con el mismo color en las dos y la leyenda la "
        "marca con ↔; la línea ámbar discontinua es un hueco tapado.",

    "5":
        "Los tiempos que midió el controlador al cruzar la línea de meta. La "
        "mejor vuelta va resaltada y las que superan el doble de la media se "
        "marcan como anómalas (una parada, una salida de pista). La "
        "referencia de la gráfica es la mediana y no la media, porque una "
        "sola vuelta anómala desplaza la media.",

    "5-cajas":
        "Derrape frente a tiempo: cada caja son los cuartiles del derrape de "
        "esa vuelta y la línea naranja, su tiempo. Al subir el PWM el coche se "
        "aparta más de la trayectoria y el tiempo baja, hasta que el derrape "
        "se dispara y el tiempo vuelve a empeorar.",

    "6":
        "Cuatro paneles con la vuelta en el eje horizontal: los derrapes, el "
        "tiempo (en verde las vueltas tras las que el perfil subió), el PWM "
        "medio y la velocidad media medida sobre la trayectoria. Cada panel "
        "se puede bajar suelto en PDF.",
}

# Lo que sale en lugar de la figura cuando el bag o los logs no traen lo que
# esa sección necesita. No es un error: hay grabaciones que no lo llevan.
TEXTOS_VACIOS = {
    "1": "El bag no tiene posiciones dentro de vueltas cronometradas.",
    "2": "El bag no tiene posiciones dentro de vueltas cronometradas.",
    "3":
        "El bag no trae telemetría del controlador "
        "(<code>/telemetria/&lt;coche&gt;/car_control</code>) dentro de "
        "vueltas cronometradas. Es lo que pasa con las grabaciones hechas sin "
        "el algoritmo: solo llevan posiciones, órdenes de PWM y tiempos.",
    "5": "El bag no contiene mensajes de <code>time_per_lap</code>.",
}
