import cv2 as cv
import numpy as np

COLOR_RANGES = {
    "rojo": [
        (np.array([0, 100, 100]), np.array([10, 255, 255])),
        (np.array([160, 100, 100]), np.array([180, 255, 255])),
    ],
    "azul": [(np.array([95, 100, 40], dtype=np.uint8), np.array([130, 255, 255], dtype=np.uint8))],
    "verde": [(np.array([40, 70, 70], dtype=np.uint8),np.array([70, 255, 255], dtype=np.uint8))],
    "naranja": [(np.array([11, 100, 100]), np.array([25, 255, 255]))],
    "amarillo":[(np.array([25, 100, 150], dtype=np.uint8),np.array([35, 255, 255], dtype=np.uint8))],
    "cian": [(np.array([78,65,120]), np.array([120, 255, 255]))],
    "violeta": [(np.array([120, 50, 50]), np.array([150, 255, 255]))],
}


class ColorDetector:
    def __init__(self, target_color_1, target_color_2, kernel_size):
        # Configuramos el kernel que vamos a usar para detectar los colores
        self.kernel = cv.getStructuringElement(
            cv.MORPH_RECT, (kernel_size, kernel_size)
        )

        # Establecemos el primer color que queremos buscar
        if target_color_1 == "rojo":
            self.target_color_1_lower_red1, self.target_color_1_upper_red1 = (
                COLOR_RANGES["rojo"][0]
            )
            self.target_color_1_lower_red2, self.target_color_1_upper_red2 = (
                COLOR_RANGES["rojo"][1]
            )
        else:
            self.target_color_1_lower, self.target_color_1_upper = COLOR_RANGES[
                target_color_1
            ][0]

        # Establecemos el segundo color que queremos buscar
        if target_color_2 == "rojo":
            self.target_color_2_lower_red1, self.target_color_2_upper_red1 = (
                COLOR_RANGES["rojo"][0]
            )
            self.target_color_2_lower_red2, self.target_color_2_upper_red2 = (
                COLOR_RANGES["rojo"][1]
            )
        else:
            self.target_color_2_lower, self.target_color_2_upper = COLOR_RANGES[
                target_color_2
            ][0]

    def find_object(self, frame, min_area, target_color_1, target_color_2):
        # Convertimos el frame que tenemos que procesar a HSV
        frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        
        # Generamos una mascara donde solo van los pixeles que tiene target_color_1
        if target_color_1 == "rojo":
            mask_target_color_1 = cv.add(
                cv.inRange(
                    frame_hsv,
                    self.target_color_1_lower_red1,
                    self.target_color_1_upper_red1,
                ),
                cv.inRange(
                    frame_hsv,
                    self.target_color_1_lower_red2,
                    self.target_color_1_upper_red2,
                ),
            )
        else:
            mask_target_color_1 = cv.inRange(
                frame_hsv, self.target_color_1_lower, self.target_color_1_upper
            )

        # Generamos una mascara donde solo van los pixeles que tienen traget_color_2 
        if target_color_2 == "rojo":
            mask_target_color_2 = cv.add(
                cv.inRange(
                    frame_hsv,
                    self.target_color_2_lower_red1,
                    self.target_color_2_upper_red1,
                ),
                cv.inRange(
                    frame_hsv,
                    self.target_color_2_lower_red2,
                    self.target_color_2_upper_red2,
                ),
            )
        else:
            mask_target_color_2 = cv.inRange(
                frame_hsv, self.target_color_2_lower, self.target_color_2_upper
            )


        #Buscamos los objetos dentro de la mascara para target_color_1
        front_color = self._detectar(mask_target_color_1, target_color_1, min_area) 
        #Buscamos los objetos dentro de la mascara para target_color_2
        back_color = self._detectar(mask_target_color_2, target_color_2, min_area) 

        return {"front": front_color, "back": back_color }

    def _detectar(self, mask, color_bgr, min_area):
         
        # Aplicamos un cierre morfologico para eliminar ruido o pequeñas imprecisiones
        mask_limpia = cv.morphologyEx(mask, cv.MORPH_OPEN, self.kernel)
        mask_limpia = cv.morphologyEx(mask_limpia, cv.MORPH_CLOSE, self.kernel)

        # Lista de puntos que representa el contorno exterior del objeto que acabamos de detectar
        contornos, _ = cv.findContours(mask_limpia, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE) 

        if not contornos:
            return None

        c = max(contornos, key=cv.contourArea)

        # Devuelve 4 puntos que representan un rectangulo que envuelve el contorno detectado
        x, y, w, h = cv.boundingRect(c)

        cx = x + (w // 2)
        cy = y + (h // 2)

        return {"color": color_bgr, "cx": cx, "cy": cy}

        

    def find_sector(self, frame, sector_color):
        frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        
        if sector_color == "rojo":
            mask_sector_color = cv.add(
                cv.inRange(
                    frame_hsv,
                    self.target_color_1_lower_red1,
                    self.target_color_1_upper_red1,
                ),
                cv.inRange(
                    frame_hsv,
                    self.target_color_1_lower_red2,
                    self.target_color_1_upper_red2,
                ),
            )
        else:
            mask_sector_color = cv.inRange(
                frame_hsv, self.target_color_1_lower, self.target_color_1_upper
            )

        section_lines: list[tuple[tuple[int, int], tuple[int, int]]] = []

        # kernel ancho para unir trozos de cada sección
        kernel = np.ones((40, 40), np.uint8)
        mask_sector_color_aplicada = cv.morphologyEx(mask_sector_color, cv.MORPH_CLOSE, kernel, iterations=1)
        contours, _ = cv.findContours(mask_sector_color_aplicada, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

        def edge_len(a, b):
            return np.hypot(b[0] - a[0], b[1] - a[1])

        for cnt in contours:
            if cv.contourArea(cnt) < 50:
                continue
            
            # Busca el rectangulo con area minima que encierra los pixeles que se detectan, pudiendo estar rotado.
            rect = cv.minAreaRect(cnt)
            box = cv.boxPoints(rect).astype(int)
            #Obtenemos las coordenadas.
            p0, p1, p2, p3 = box

            #Buscamos que puntos corresponden con los sectores más pequeños del rectangulo para luego unirlos con una linea recta
            if edge_len(p0, p1) < edge_len(p1, p2):
                m1 = ((p0[0] + p1[0]) // 2, (p0[1] + p1[1]) // 2)
                m2 = ((p2[0] + p3[0]) // 2, (p2[1] + p3[1]) // 2)
            else:
                m1 = ((p1[0] + p2[0]) // 2, (p1[1] + p2[1]) // 2)
                m2 = ((p3[0] + p0[0]) // 2, (p3[1] + p0[1]) // 2)

            section_lines.append((m1, m2))
            
        return section_lines


