#!/usr/bin/python3

import serial
import serial.tools.list_ports
import time


class ArduinoController:
    def __init__(
        self, port: str = "/dev/ttyACM0", baudrate: int = 115200, timeout: float = 1.0
    ):
        """
        Inicializa la conexión serie:
          1) Lista puertos disponibles.
          2) Selecciona un puerto válido (por defecto /dev/ttyACM0 o el primero que coincida con ACM/USB).
          3) Abre el puerto, espera 2 s para reset del Arduino y se vacía buffers.
          4) Detiene ambos raíles al arrancar.
        """
        # 1) Obtener lista de dispositivos serie
        ports = [p.device for p in serial.tools.list_ports.comports()]
        print("[ArduinoController] Puertos serie disponibles:", ports)

        # 2) Verificar que el puerto deseado esté en la lista
        if port not in ports:
            # Si no está, buscar uno que empiece por ACM o USB
            cand = [
                p
                for p in ports
                if p.startswith("/dev/ttyACM") or p.startswith("/dev/ttyUSB")
            ]
            if cand:
                print(f"Puerto '{port}' no encontrado, usando '{cand[0]}'")
                port = cand[0]

        # 3) Intentar abrir la conexión serie
        try:
            self.serial = serial.Serial(port, baudrate, timeout=timeout)
            # Arduino se resetea al abrir, dar tiempo para inicializar
            time.sleep(2)
            self.serial.reset_input_buffer()
            print(f"[ArduinoController] Conexión con Arduino establecida en {port}")

            # 4) Asegurarse de que los raíles arrancan detenidos
            self.stop_all_rails()
            print("[ArduinoController] Rails detenidos tras la conexión inicial")
        except Exception as e:
            print(f"[ArduinoController] Error al conectar con Arduino en {port}: {e}")
            self.serial = None

    def send_command(self, command: str) -> bool:
        """
        Envía un comando al Arduino y espera confirmación:
          - command: cadena del tipo 'r<rail>:<speed>' (por ejemplo 'r1:120').
          - Devuelve True si recibe línea que empieza con 'OK', False por timeout o error.
        """
        if not self.serial:
            print("No hay conexión serial.")
            return False

        # Limpiar buffer de entrada antes de enviar
        self.serial.reset_input_buffer()
        # Enviar comando seguido de nueva línea
        self.serial.write((command + "\n").encode())
        self.serial.flush()
        # Pequeña espera para que Arduino procese
        time.sleep(0.05)

        # Leer hasta 1 s o hasta recibir 'OK'
        deadline = time.time() + 1.0
        while time.time() < deadline:
            line = self.serial.readline().decode(errors="ignore").strip()
            if not line:
                continue
            print("→ recibido del Arduino:", line)  # para depurar
            if line.startswith("OK"):
                return True

        print("Timeout esperando 'OK'")
        return False

    def set_rail_speed(self, rail: int, speed: int):
        """
        Ajusta la velocidad de un raíl específico:
          - rail: 1 o 2.
          - speed: valor entre 0 y 255.
        """
        print(f"[ArduinoController] set_rail_speed({rail}, {speed})")
        # Validar número de raíl
        if rail not in (1, 2):
            print("Carril inválido. Usa 1 o 2.")
            return
        # Limitar speed a rango válido
        speed = max(0, min(255, speed))
        # Enviar comando al Arduino
        self.send_command(f"r{rail}:{speed}")

    def set_both_rails(self, s1: int, s2: int):
        """
        Ajusta la velocidad de ambos raíles secuencialmente.
        """
        self.set_rail_speed(1, s1)
        # Pequeño retardo para no saturar el puerto
        time.sleep(0.02)
        self.set_rail_speed(2, s2)

    def stop_all_rails(self):
        """
        Detiene ambos raíles (velocidad 0).
        """
        self.set_both_rails(0, 0)

    def close(self):
        """
        Cierra la conexión serial si está abierta.
        """
        if self.serial:
            self.serial.close()
            print("Conexión cerrada.")
