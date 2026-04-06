import cv2 as cv
import numpy as np


class ColorDetector:
    def __init__(self):
        self.kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))

        # --- CONFIGURACIÓN ROJO ---
        self.lower_red1, self.upper_red1 = (
            np.array([0, 100, 100]),
            np.array([10, 255, 255]),
        )
        self.lower_red2, self.upper_red2 = (
            np.array([170, 100, 100]),
            np.array([180, 255, 255]),
        )

        # --- CONFIGURACIÓN VERDE ---
        self.lower_green, self.upper_green = (
            np.array([35, 100, 100]),
            np.array([85, 255, 255]),
        )

    def find_object(self, frame):
        frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        mask_red = cv.add(
            cv.inRange(frame_hsv, self.lower_red1, self.upper_red1),
            cv.inRange(frame_hsv, self.lower_red2, self.upper_red2),
        )
        mask_green = cv.inRange(frame_hsv, self.lower_green, self.upper_green)

        points_detected = []

        points_detected.extend(self._detectar(mask_red, "ROJO"))
        points_detected.extend(self._detectar(mask_green, "VERDE"))

        return points_detected

    def _detectar(self, mask, color_bgr):
        puntos = []
        contornos, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        for c in contornos:
            if cv.contourArea(c) > 50:
                x, y, w, h = cv.boundingRect(c)

                cx = x + (w // 2)
                cy = y + (h // 2)

                puntos.append({"color": color_bgr, "cx": cx, "cy": cy})

        return puntos
