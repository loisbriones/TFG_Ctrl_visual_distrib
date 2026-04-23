import cv2 as cv
import numpy as np

COLOR_RANGES = {
    "rojo": [
        (np.array([0, 100, 100]), np.array([10, 255, 255])),
        (np.array([160, 100, 100]), np.array([180, 255, 255])),
    ],
    "azul": [(np.array([72, 131, 69], dtype=np.uint8),np.array([112, 251, 189], dtype=np.uint8))],
    "verde": [(np.array([35, 100, 100], dtype=np.uint8),np.array([85, 255, 255], dtype=np.uint8))],
    "naranja": [(np.array([11, 100, 100]), np.array([25, 255, 255]))],
    "amarillo":[(np.array([25, 100, 150], dtype=np.uint8),np.array([35, 255, 255], dtype=np.uint8))],
    "cian": [(np.array([75, 40, 30]), np.array([105, 255, 255]))],
    "violeta": [(np.array([120, 50, 50]), np.array([150, 255, 255]))],
}


class ColorDetector:
    def __init__(self, target_color_1, target_color_2, kernel_size):
        self.kernel = cv.getStructuringElement(
            cv.MORPH_RECT, (kernel_size, kernel_size)
        )

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
        frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

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

        points_detected = []

        points_detected.extend(
            self._detectar(mask_target_color_1, target_color_1, min_area)
        )
        points_detected.extend(
            self._detectar(mask_target_color_2, target_color_2, min_area)
        )

        return points_detected

    def _detectar(self, mask, color_bgr, min_area):
        puntos = []
         
        mask_limpia = cv.morphologyEx(mask, cv.MORPH_CLOSE, self.kernel)

        contornos, _ = cv.findContours(mask_limpia, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        for c in contornos:
            if cv.contourArea(c) > min_area:
                x, y, w, h = cv.boundingRect(c)

                cx = x + (w // 2)
                cy = y + (h // 2)

                puntos.append({"color": color_bgr, "cx": cx, "cy": cy})

        return puntos
