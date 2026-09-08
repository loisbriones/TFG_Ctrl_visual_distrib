"""
El codigo esta basado en las ideas de deteccion de los trabajos de:
Mario: https://github.com/mariolopez15/control_coche_scalextric
Adrian: https://github.com/Rego523/GEI-TFG
"""

import cv2 as cv
import numpy as np

# Rangos HSV (minimo, maximo) de cada color de pegatina
# El rojo necesita dos rangos porque su tono da la vuelta en 0/180
# Los rangos son anchos porque la misma pegatina da tonos distintos en cada camara
COLOR_RANGES = {
    "rojo": [
        (np.array([0, 100, 100]), np.array([10, 255, 255])),
        (np.array([160, 100, 100]), np.array([180, 255, 255])),
    ],
    "azul": [
        (
            np.array([85, 114, 80], dtype=np.uint8),
            np.array([130, 255, 255], dtype=np.uint8),
        )
    ],
    "verde": [
        (
            np.array([42, 60, 130], dtype=np.uint8),
            np.array([70, 255, 255], dtype=np.uint8),
        )
    ],
    "naranja": [
        (
            np.array([10, 130, 130], dtype=np.uint8),
            np.array([20, 255, 255], dtype=np.uint8),
        )
    ],
    "amarillo": [
        (
            np.array([20, 50, 150], dtype=np.uint8),
            np.array([50, 255, 255], dtype=np.uint8),
        )
    ],
    "cian": [
        (
            np.array([78, 65, 120],dtype=np.uint8), 
            np.array([120, 255, 255],dtype=np.uint8)
        )
    ],
    "violeta": [
        (
            np.array([120, 80, 100],dtype=np.uint8), 
            np.array([175, 255, 255],dtype=np.uint8)
        )
    ],
}


class ColorDetector:
    """Busca dentro de un frame, usando rangos de color, las dos pegatinas de un
    coche y la linea de meta
    Lo usa CameraNode, que crea un objeto por cada coche
    """

    # Los parametros vienen de CameraNode, que los lee de params.yaml
    def __init__(self, stiker_front, stiker_back, kernel_size):

        # Kernel de la limpieza morfologica, comun a las dos pegatinas
        self.kernel = cv.getStructuringElement(
            cv.MORPH_RECT, (kernel_size, kernel_size)
        )

        # El rojo se guarda como dos rangos y los demas colores como uno solo
        if stiker_front == "rojo":
            self.stiker_front_lower_red1, self.stiker_front_upper_red1 = COLOR_RANGES[
                "rojo"
            ][0]
            self.stiker_front_lower_red2, self.stiker_front_upper_red2 = COLOR_RANGES[
                "rojo"
            ][1]
        else:
            self.stiker_front_lower, self.stiker_front_upper = COLOR_RANGES[
                stiker_front
            ][0]

        # Igual para la pegatina trasera
        if stiker_back == "rojo":
            self.stiker_back_lower_red1, self.stiker_back_upper_red1 = COLOR_RANGES[
                "rojo"
            ][0]
            self.stiker_back_lower_red2, self.stiker_back_upper_red2 = COLOR_RANGES[
                "rojo"
            ][1]
        else:
            self.stiker_back_lower, self.stiker_back_upper = COLOR_RANGES[stiker_back][
                0
            ]

    def find_object(self, frame, min_area, stiker_front, stiker_back):
        """Busca las dos pegatinas en el frame y devuelve las coordenadas del
        centro (x, y) como {"front": ..., "back": ...}, con None en la que no
        encuentre
        """
        
        # Se pasa el frame al espacio de color HSV
        frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        # Mascara binaria con los pixeles del color de la pegatina delantera
        if stiker_front == "rojo":
            mask_stiker_front = cv.add(
                cv.inRange(
                    frame_hsv,
                    self.stiker_front_lower_red1,
                    self.stiker_front_upper_red1,
                ),
                cv.inRange(
                    frame_hsv,
                    self.stiker_front_lower_red2,
                    self.stiker_front_upper_red2,
                ),
            )
        else:
            mask_stiker_front = cv.inRange(
                frame_hsv, self.stiker_front_lower, self.stiker_front_upper
            )

        # Y lo mismo para la trasera
        if stiker_back == "rojo":
            mask_stiker_back = cv.add(
                cv.inRange(
                    frame_hsv,
                    self.stiker_back_lower_red1,
                    self.stiker_back_upper_red1,
                ),
                cv.inRange(
                    frame_hsv,
                    self.stiker_back_lower_red2,
                    self.stiker_back_upper_red2,
                ),
            )
        else:
            mask_stiker_back = cv.inRange(
                frame_hsv, self.stiker_back_lower, self.stiker_back_upper
            )

        front_color = self._detectar(mask_stiker_front, stiker_front, min_area)
        back_color = self._detectar(mask_stiker_back, stiker_back, min_area)

        return {"front": front_color, "back": back_color}

    def _detectar(self, mask, color_bgr, min_area):
        """Aplica la mascara sobre el frame para detectar las pegatinas y
        devolver su posicion"""

        # Apertura morfologica para quitar los pixeles sueltos
        mask_limpia = cv.morphologyEx(mask, cv.MORPH_OPEN, self.kernel)
        # Cierre morfologico para tapar los agujeros entre los pixeles detectados
        mask_limpia = cv.morphologyEx(mask_limpia, cv.MORPH_CLOSE, self.kernel)

        # Contornos exteriores de los objetos que quedaron
        contornos, _ = cv.findContours(
            mask_limpia, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE
        )

        if not contornos:
            return None

        # Nos quedamos con la mancha mas grande, que es la pegatina. Si no llega
        # a min_area es ruido y se descarta
        c = max(contornos, key=cv.contourArea)

        if cv.contourArea(c) < min_area:
            return None

        # La posicion que se publica es el centro del rectangulo que la envuelve
        x, y, w, h = cv.boundingRect(c)

        cx = x + (w / 2)
        cy = y + (h / 2)

        return {"color": color_bgr, "cx": cx, "cy": cy, "x": x, "y": y, "w": w, "h": h}

    def find_finish_line(self, frame, sector_color):
        """
        Busca la franja de color de la meta y devuelve el segmento que la cruza
        a lo ancho o None
        """

        # Se pasa el frame al espacio de color HSV
        frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        if sector_color == "rojo":
            lower_red1, upper_red1 = COLOR_RANGES["rojo"][0]
            lower_red2, upper_red2 = COLOR_RANGES["rojo"][1]
            mask_sector_color = cv.add(
                cv.inRange(frame_hsv, lower_red1, upper_red1),
                cv.inRange(frame_hsv, lower_red2, upper_red2),
            )
        else:
            lower, upper = COLOR_RANGES[sector_color][0]
            mask_sector_color = cv.inRange(frame_hsv, lower, upper)

        # Kernel distinto al de las pegatinas porque las ranuras de los carriles
        # rompen la franja de la meta y las partes detectadas tienen que quedar
        # unidas
        kernel = np.ones((40, 40), np.uint8)

        # Se aplica cierre para unir los pixeles detectados
        mask_sector_color_aplicada = cv.morphologyEx(
            mask_sector_color, cv.MORPH_CLOSE, kernel, iterations=1
        )

        contornos, _ = cv.findContours(
            mask_sector_color_aplicada, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE
        )

        if not contornos:
            return None

        c = max(contornos, key=cv.contourArea)

        if cv.contourArea(c) < 40:
            return None

        # La meta puede ir girada, por eso no se usa boundingRect
        rect = cv.minAreaRect(c)
        box = cv.boxPoints(rect).astype(int)

        p0, p1, p2, p3 = box

        # La meta es el segmento que une los centros de los dos lados cortos
        if self._edge_len(p0, p1) < self._edge_len(p1, p2):
            m1 = ((p0[0] + p1[0]) // 2, (p0[1] + p1[1]) // 2)
            m2 = ((p2[0] + p3[0]) // 2, (p2[1] + p3[1]) // 2)
        else:
            m1 = ((p1[0] + p2[0]) // 2, (p1[1] + p2[1]) // 2)
            m2 = ((p3[0] + p0[0]) // 2, (p3[1] + p0[1]) // 2)

        return (m1, m2)

    def _edge_len(self, a, b):
        """Longitud del lado que va del punto a al punto b"""
        return np.hypot(b[0] - a[0], b[1] - a[1])
