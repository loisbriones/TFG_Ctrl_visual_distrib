import cv2 as cv
from cv2.typing import Point
import numpy as np

camara = cv.VideoCapture(0)

# Ajustar dimensiones
camara.set(cv.CAP_PROP_FRAME_WIDTH, 640)
camara.set(cv.CAP_PROP_FRAME_HEIGHT, 350)


def detectar_y_dibujar(mascara, color_bgr, nombre_texto, imagen):
    contornos, _ = cv.findContours(mascara, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    for c in contornos:
        if cv.contourArea(c) > 500:
            x, y, w, h = cv.boundingRect(c)

            cx = x + (w // 2)
            cy = y + (h // 2)

            cv.circle(imagen, (cx, cy), 5, (255, 255, 255), -1)
            cv.rectangle(imagen, (x, y), (x + w, y + h), color_bgr, 3)
            cv.putText(
                imagen,
                nombre_texto,
                (x, y - 10),
                cv.FONT_HERSHEY_SIMPLEX,
                0.6,
                color_bgr,
                2,
            )


while True:
    # Capturar frame
    ret, frame = camara.read()

    if not ret:
        print("No se pudo obtener el frame")
        break

    # Pasamos la imagen a HSV
    frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    # --- CONFIGURACIÓN ROJO ---
    lower_red1, upper_red1 = np.array([0, 100, 100]), np.array([10, 255, 255])
    lower_red2, upper_red2 = np.array([170, 100, 100]), np.array([180, 255, 255])

    mask_red = cv.add(
        cv.inRange(frame_hsv, lower_red1, upper_red1),
        cv.inRange(frame_hsv, lower_red2, upper_red2),
    )

    # --- CONFIGURACIÓN VERDE ---
    lower_green = np.array([35, 100, 100])
    upper_green = np.array([85, 255, 255])
    mask_green = cv.inRange(frame_hsv, lower_green, upper_green)

    # Limpieza morfologica
    kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
    mask_red = cv.morphologyEx(mask_red, cv.MORPH_OPEN, kernel)
    mask_green = cv.morphologyEx(mask_green, cv.MORPH_OPEN, kernel)

    # Aplicamos la detección para ambos
    detectar_y_dibujar(mask_red, (0, 0, 255), "ROJO", frame)
    detectar_y_dibujar(mask_green, (0, 255, 0), "VERDE", frame)

    cv.imshow("Deteccion en Tiempo Real", frame)

    if cv.waitKey(1) & 0xFF == ord("q"):
        break

camara.release()
cv.destroyAllWindows()
