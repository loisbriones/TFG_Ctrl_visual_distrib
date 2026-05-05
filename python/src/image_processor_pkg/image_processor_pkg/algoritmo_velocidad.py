#!/usr/bin/python3

import math
from typing import Optional, Tuple, Dict, List
import numpy as np
from pathlib import Path


class AlgoritmoVelocidad:
    """
    Algoritmo de control de velocidad reactivo por secciones.
    Mantiene un perfil de PWM por sección, ajustándolo en tiempo real ante derrapes
    y mejorándolo al completar vueltas sin derrapes.
    """

    def __init__(self, car_name: str):
        # Carpeta de salida para logs (opcional)
        self.output_dir = Path(__file__).parent.parent / "data"
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = self.output_dir / f"{car_name}_datos_derrapes.csv"
        self.log_path = self.output_dir / f"{car_name}_derrapes_log.txt"
        self.csv_path.open("w").close()
        self.log_path.open("w").close()

        # Rango de PWM (mín, máx)
        self.vel_min = 55
        self.vel_max = 74

        # Trayectoria base: mapa cámara -> lista de posiciones
        self.trayectoria_por_camara: Dict[int, List[Tuple[int, int]]] = {}
        self.trayectoria_usada: Optional[np.ndarray] = None

        # Perfil de PWM por sección y flags de derrape
        self.speed_profile: Dict[int, List[int]] = {}
        self.drift_flag_per_section: Dict[int, bool] = {}

        # Para reducción: historial frame -> (sección, índice en sección)
        self.frame_sections: Dict[int, Tuple[int, int]] = {}
        self.last_drift = {"window": 20, "reduction": 1}

        # Cursores de posición dentro de cada sección
        self.section_cursors: Dict[int, int] = {}

        self.lap_has_drift = False

    def save_log(self, text: str) -> None:
        with open(self.log_path, "a") as f:
            f.write(text + "")

    def set_trayectoria(self, trayectoria: List[Tuple[int, Tuple[int, int]]]) -> None:
        if self.trayectoria_usada is None:
            pts = []
            for cam_id, pos in trayectoria:
                self.trayectoria_por_camara.setdefault(cam_id, []).append(pos)
                pts.append(pos)
            self.trayectoria_usada = np.array(pts, dtype=float)

    def detect_derrape(self, cam_id: int, pos: Tuple[float, float]) -> bool:
        """
        Detecta derrape comparando la distancia mínima de la posición actual
        a la trayectoria base contra un umbral mayor (más reactivo a desviaciones).
        """
        umbral = getattr(self, "drift_threshold", 130.0)
        pts = self.trayectoria_por_camara.get(cam_id, [])
        if not pts:
            return False
        dist_min = min(math.hypot(pos[0] - x, pos[1] - y) for x, y in pts)
        return dist_min > umbral

    def on_frame(
        self,
        frame: int,
        current_section: int,
        cam_id: int,
        pos: Optional[Tuple[float, float]],
    ) -> int:
        if current_section not in self.speed_profile:
            self.speed_profile[current_section] = []
            self.drift_flag_per_section[current_section] = False

        if current_section not in self.section_cursors:
            self.section_cursors[current_section] = 0

        idx = self.section_cursors[current_section]
        self.frame_sections[frame] = (current_section, idx)
        self.section_cursors[current_section] += 1

        if idx >= len(self.speed_profile[current_section]):
            self.speed_profile[current_section].append(self.vel_min)

        pwm = self.speed_profile[current_section][idx]

        if pos is not None and self.detect_derrape(cam_id, pos):
            self._apply_reduction(frame)
            self.drift_flag_per_section[current_section] = True
            self.lap_has_drift = True

        return pwm

    def _apply_reduction(self, frame: int) -> None:
        w = self.last_drift["window"]
        r = self.last_drift["reduction"]
        start = max(0, frame - w)
        for f in range(start, frame):
            sec, idx = self.frame_sections.get(f, (None, None))
            if sec is None:
                continue
            orig = self.speed_profile[sec][idx]
            new_v = max(self.vel_min, orig - r)
            self.speed_profile[sec][idx] = new_v
            self.save_log(
                f"[REDUCCIÓN] frame {f} sección {sec} idx {idx}: {orig}->{new_v}"
            )
        secs = {
            self.frame_sections[f][0]
            for f in range(start, frame)
            if f in self.frame_sections
        }
        if len(secs) > 2:
            self.last_drift = {"window": 20, "reduction": r + 1}
        else:
            self.last_drift["window"] = w + 20

    def on_lap_end(self) -> None:
        if not self.lap_has_drift:
            for sec, drifted in self.drift_flag_per_section.items():
                if not drifted:
                    self.speed_profile[sec] = [
                        min(self.vel_max, v + 1) for v in self.speed_profile[sec]
                    ]

        for sec in list(self.drift_flag_per_section.keys()):
            self.drift_flag_per_section[sec] = False

        self.last_drift = {"window": 20, "reduction": 1}
        self.frame_sections.clear()
        self.section_cursors.clear()
        self.lap_has_drift = False

    def on_section_end(self, section_idx: int) -> None:
        finish_sec = getattr(self, "finish_section_idx", None)
        if section_idx == finish_sec:
            self.drift_flag_per_section[section_idx] = False
            self.section_cursors[section_idx] = 0
            return

        if not self.lap_has_drift and not self.drift_flag_per_section.get(
            section_idx, False
        ):
            self.speed_profile[section_idx] = [
                min(self.vel_max, v + 1) for v in self.speed_profile[section_idx]
            ]

        self.drift_flag_per_section[section_idx] = False
        self.section_cursors[section_idx] = 0
