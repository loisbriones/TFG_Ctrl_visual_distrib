import time
import math
import csv
import numpy as np
import os


class MarioAlgorithm:
    def __init__(self, v_max, v_min, node_name="node", camara_id="cam"):
        self.velocidad = [v_max, v_min]
        self.derrapes = []
        self.derrapeEvitado = []
        self.trayectoriaUsada = None
        self.distanceMin = 280
        self.distanceForDerrape = []
        self.indiceDerrape = None
        self.indiceSegundo = None
        self.enDerrape = False

        self.lastDistance = 0
        self.changedVelocity = False
        self.tamanoDerrape = []
        self.data = []

        self.umbral_derrape = 15.0
        self.estado_derrapando = False
        self.ultimo_derrape = [0, 0, 0, 0]

        directorio_logs = "/ros2_ws/src/image_processor_pkg/logs_carrera"
        os.makedirs(directorio_logs, exist_ok=True)

        self.log_file = f"{directorio_logs}/derrapesLog_{node_name}_{camara_id}.txt"
        self.csv_file = f"{directorio_logs}/datosDerrapes_{node_name}_{camara_id}.csv"

        try:
            with open(self.log_file, "w") as f:
                f.write("=== LOG INICIADO ===\n")
        except IOError as e:
            print(f"Error inicializando log: {e}")

    # --- NUEVA LÓGICA CENTRALIZADA ---
    def actualizar_estado(self, p_front, p_back, frame_count, vuelta):
        # 1. Filtro de ruido y obtención del nodo más cercano (pegatina frontal)
        _, dist_a_ruta = self.obtener_nodo_mas_cercano(p_front)

        # Tolerancia aumentada por si derrapa mucho a gran velocidad
        if dist_a_ruta > 80.0:
            return None

        # 2. Cálculos para detectar el derrape (pegatina trasera)
        idx_trasero, _ = self.obtener_nodo_mas_cercano(p_back)
        num_nodos = len(self.trayectoriaUsada)

        p_centro = self.trayectoriaUsada[idx_trasero]
        p_siguiente = self.trayectoriaUsada[(idx_trasero + 1) % num_nodos]
        p_anterior = self.trayectoriaUsada[(idx_trasero - 1) % num_nodos]

        d1 = self.distancia_punto_segmento(p_back, p_anterior, p_centro)
        d2 = self.distancia_punto_segmento(p_back, p_centro, p_siguiente)
        dist_derrape = min(d1, d2)

        # 3. Máquina de estados del Derrape
        if dist_derrape > self.umbral_derrape:
            if not self.estado_derrapando:
                self.estado_derrapando = True
                self.ultimo_derrape[0] = int(p_back[0])
                self.ultimo_derrape[1] = int(p_back[1])

            self.ultimo_derrape[2] = int(p_back[0])
            self.ultimo_derrape[3] = int(p_back[1])
        else:
            if self.estado_derrapando:
                self.estado_derrapando = False
                self.derrapeDetected(self.ultimo_derrape.copy(), vuelta, frame_count)
                self.ultimo_derrape = [0, 0, 0, 0]

        # 4. Calcular velocidad ideal basada en la pegatina frontal
        _, nueva_vel = self.setVelocidad((p_front[0], p_front[1]), frame_count)

        return nueva_vel

    # --- HELPERS GEOMÉTRICOS ---
    def obtener_nodo_mas_cercano(self, punto):
        distancias_sq = np.sum((self.trayectoriaUsada - punto) ** 2, axis=1)
        mejor_idx = np.argmin(distancias_sq)
        return int(mejor_idx), math.sqrt(distancias_sq[mejor_idx])

    def distancia_punto_segmento(self, P, A, B):
        AB = B - A
        AP = P - A
        l2 = np.sum(AB**2)
        if l2 == 0:
            return np.linalg.norm(AP)
        t = max(0.0, min(1.0, np.dot(AP, AB) / l2))
        proyeccion = A + t * AB
        return np.linalg.norm(P - proyeccion)

    def saveLogFile(self, data):
        try:
            with open(self.log_file, "a") as log_file:
                log_file.write(data + "\n")
        except IOError as e:
            print(f"Error escribiendo log: {e}")

    def saveData(self, vueltas):
        try:
            with open(self.csv_file, "w", newline="") as archivoCSV:
                columnas = ["vueltas"]
                for point in self.derrapes:
                    columnas.append(str(point))

                fichero = csv.writer(archivoCSV)
                fichero.writerow(columnas)
                for i in range(0, len(self.data)):
                    if len(self.data[i]) < vueltas + 1:
                        for j in range(len(self.data[i]), vueltas + 1):
                            self.data[i].append(None)

                for i in range(0, vueltas + 1):
                    fila = [i]
                    for j in range(0, len(self.derrapes)):
                        fila.append(self.data[j][i])
                    fichero.writerow(fila)
        except IOError as e:
            print(f"Error guardando CSV: {e}")

    def derrapeDetected(self, pos, vuelta, frame):
        self.saveLogFile(f"\n\nDerrape detectado en fotograma número: {frame}")

        if not self.derrapes:
            self.indiceDerrape = 0
            self.indiceSegundo = 0
            self.derrapes.append(pos)
            tamano = self.distancia((pos[0], pos[1]), (pos[2], pos[3]))
            self.tamanoDerrape.append(tamano)
            self.derrapeEvitado.append(vuelta)
            self.distanceForDerrape.append(self.distanceMin)
            self.saveLogFile(
                f"Derrape nuevo tamaño: {tamano} - Inicio: ({pos[0]},{pos[1]}) Fin: ({pos[2]},{pos[3]})"
            )
            self.data.append([])
            for i in range(0, vuelta):
                self.data[0].append(None)
            self.data[0].append(vuelta)
        else:
            if not self.checkDerrapeCercano(pos, vuelta):
                if self.indiceDerrape == 0:
                    self.derrapes.append(pos)
                    tamano = self.distancia((pos[0], pos[1]), (pos[2], pos[3]))
                    self.tamanoDerrape.append(tamano)
                    self.derrapeEvitado.append(vuelta)
                    self.distanceForDerrape.append(self.distanceMin)
                    self.data.append([])
                    for i in range(0, vuelta):
                        self.data[-1].append(None)
                    self.data[-1].append(vuelta)
                else:
                    self.derrapes.insert(self.indiceDerrape, pos)
                    tamano = self.distancia((pos[0], pos[1]), (pos[2], pos[3]))
                    self.tamanoDerrape.insert(self.indiceDerrape, tamano)
                    self.derrapeEvitado.insert(self.indiceDerrape, vuelta)
                    self.distanceForDerrape.insert(self.indiceDerrape, self.distanceMin)
                    self.data.insert(self.indiceDerrape, [])
                    for i in range(0, vuelta):
                        self.data[self.indiceDerrape].append(None)
                    self.data[self.indiceDerrape].append(vuelta)
                    self.indiceDerrape += 1
                    self.indiceSegundo += 1

        self.saveLogFile(f"\nLista de derrapes actualizada: {self.derrapes}")
        return self.derrapes

    def checkDerrapeCercano(self, pos, vuelta):
        posterior = None
        anterior = None
        externo = False

        for i in range(0, len(self.derrapes)):
            dist11 = self.distancia(
                (pos[0], pos[1]), (self.derrapes[i][0], self.derrapes[i][1])
            )
            dist12 = self.distancia(
                (pos[0], pos[1]), (self.derrapes[i][2], self.derrapes[i][3])
            )
            dist21 = self.distancia(
                (pos[2], pos[3]), (self.derrapes[i][0], self.derrapes[i][1])
            )
            dist22 = self.distancia(
                (pos[2], pos[3]), (self.derrapes[i][2], self.derrapes[i][3])
            )

            if dist21 < 20 and dist22 >= self.tamanoDerrape[i]:
                anterior = (self.indiceDerrape - 1) % len(self.derrapes)
                externo = True

            if dist12 < 20 and dist11 >= self.tamanoDerrape[i]:
                posterior = (self.indiceDerrape - 1) % len(self.derrapes)
                if posterior == -1:
                    posterior = len(self.derrapes) - 1
                externo = True

            if dist21 <= (self.tamanoDerrape[i] + 8) and dist22 <= (
                self.tamanoDerrape[i] + 5
            ):
                anterior = (self.indiceDerrape - 1) % len(self.derrapes)
                if anterior == -1:
                    anterior = len(self.derrapes) - 1

            if dist12 <= (self.tamanoDerrape[i] + 8) and dist11 <= (
                self.tamanoDerrape[i] + 5
            ):
                posterior = (self.indiceDerrape - 1) % len(self.derrapes)
                if posterior == -1:
                    posterior = len(self.derrapes) - 1

        if posterior is not None and anterior is not None:
            if posterior == anterior:
                if externo:
                    self.derrapes[posterior][0] = pos[0]
                    self.derrapes[posterior][1] = pos[1]
                    self.derrapes[posterior][2] = pos[2]
                    self.derrapes[posterior][3] = pos[3]

                self.derrapeEvitado[posterior] = vuelta
                self.distanceForDerrape[posterior] += 20
                last = self.data[posterior][-1]
                for j in range(last + 1, vuelta):
                    self.data[posterior].append(None)
                self.data[posterior].append(vuelta)
                return True
            else:
                posterior = (posterior - 1) % len(self.derrapes)
                self.derrapes[posterior][2] = self.derrapes[anterior][2]
                self.derrapes[posterior][3] = self.derrapes[anterior][3]
                self.tamanoDerrape[posterior] = self.distancia(
                    (self.derrapes[posterior][0], self.derrapes[posterior][1]),
                    (self.derrapes[posterior][2], self.derrapes[posterior][3]),
                )
                self.derrapes.pop(anterior)
                self.distanceForDerrape.pop(anterior)
                self.tamanoDerrape.pop(anterior)
                self.derrapeEvitado.pop(vuelta)
                self.derrapeEvitado[posterior] = vuelta
                self.distanceForDerrape[posterior] += 20
                last = self.data[posterior][-1]
                for j in range(last + 1, vuelta):
                    self.data[posterior].append(None)
                self.data[posterior].append(vuelta)
            return True
        else:
            if posterior is not None:
                self.derrapes[posterior][2] = pos[2]
                self.derrapes[posterior][3] = pos[3]
                self.tamanoDerrape[posterior] = self.distancia(
                    (self.derrapes[posterior][0], self.derrapes[posterior][1]),
                    (self.derrapes[posterior][2], self.derrapes[posterior][3]),
                )
                self.derrapeEvitado[posterior] = vuelta
                self.distanceForDerrape[posterior] += 20
                last = self.data[posterior][-1]
                for j in range(last + 1, vuelta):
                    self.data[posterior].append(None)
                self.data[posterior].append(vuelta)
                return True
            elif anterior is not None:
                self.derrapes[anterior][0] = pos[0]
                self.derrapes[anterior][1] = pos[1]
                self.tamanoDerrape[anterior] = self.distancia(
                    (self.derrapes[anterior][0], self.derrapes[anterior][1]),
                    (self.derrapes[anterior][2], self.derrapes[anterior][3]),
                )
                self.derrapeEvitado[anterior] = vuelta
                self.distanceForDerrape[anterior] += 20
                last = self.data[anterior][-1]
                for j in range(last + 1, vuelta):
                    self.data[anterior].append(None)
                self.data[anterior].append(vuelta)
                return True
            else:
                return False

    def setTrayectoria(self, trayectoria):
        if self.trayectoriaUsada is None:
            self.trayectoriaUsada = np.array(trayectoria)

    def setVelocidad(self, pos, frame):
        velocidad = None

        if not self.derrapes or pos is None or frame is None:
            velocidad = self.velocidad[0]

        if self.trayectoriaUsada is not None and self.trayectoriaUsada.size != 0:
            if self.derrapes:
                if not self.enDerrape:
                    dist = self.distanceToDerrape(
                        pos,
                        (
                            self.derrapes[self.indiceDerrape][0],
                            self.derrapes[self.indiceDerrape][1],
                        ),
                    )

                    # LÓGICA CORREGIDA: Filtramos el ruido de cámara exigiendo un salto de >15 píxeles
                    # para dar la curva por finalizada.
                    if dist > self.lastDistance + 15.0:
                        self.indiceDerrape += 1
                        self.enDerrape = True
                        self.indiceDerrape = self.indiceDerrape % len(self.derrapes)
                        self.lastDistance = float(
                            "inf"
                        )  # Reset de distancia para la salida de curva
                        if self.changedVelocity:
                            self.changedVelocity = False
                    else:
                        # Solo actualizamos "lastDistance" si la distancia disminuye (nos estamos acercando)
                        if dist < self.lastDistance:
                            self.lastDistance = dist

                        if dist < self.distanceForDerrape[self.indiceDerrape]:
                            if not self.changedVelocity:
                                self.changedVelocity = True
                                velocidad = self.velocidad[1]  # FRENAR
                else:
                    dist = self.distanceToDerrape(
                        pos,
                        (
                            self.derrapes[self.indiceSegundo][2],
                            self.derrapes[self.indiceSegundo][3],
                        ),
                    )

                    if dist > self.lastDistance + 15.0:
                        self.indiceSegundo += 1
                        self.enDerrape = False
                        self.indiceSegundo = self.indiceSegundo % len(self.derrapes)
                        self.lastDistance = float(
                            "inf"
                        )  # Reset de distancia para la próxima curva
                        velocidad = self.velocidad[0]  # ACELERAR
                    else:
                        if dist < self.lastDistance:
                            self.lastDistance = dist

        if self.enDerrape or not self.distanceForDerrape:
            return None, velocidad
        else:
            return self.distanceForDerrape[self.indiceDerrape], velocidad

    def distanceToDerrape(self, pos, pos2):
        indicePos, indiceDerrape = self.closestPoint(pos, pos2)

        if indicePos < indiceDerrape:
            sumaDistancia = 0
            for i in range(indicePos, indiceDerrape):
                sumaDistancia += self.distancia(
                    self.trayectoriaUsada[i], self.trayectoriaUsada[i + 1]
                )
        elif indicePos == indiceDerrape:
            sumaDistancia = 0
        else:
            sumaDistancia = 0
            fin = self.trayectoriaUsada.shape[0]
            for i in range(indicePos, fin - 1):
                sumaDistancia += self.distancia(
                    self.trayectoriaUsada[i], self.trayectoriaUsada[i + 1]
                )
            for i in range(0, indiceDerrape):
                sumaDistancia += self.distancia(
                    self.trayectoriaUsada[i], self.trayectoriaUsada[i + 1]
                )

        return sumaDistancia

    def closestPoint(self, pos1, pos2):
        diff = self.trayectoriaUsada[:, None] - np.array([pos1, pos2])
        distance_squared = np.sum(diff**2, axis=-1)
        closest_indices = np.argmin(distance_squared, axis=0)
        return closest_indices[0], closest_indices[1]

    def distancia(self, p1, p2):
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
