#include <AFMotor.h>

// Código que corre en el arduino del nodo de potencia
// Sacado directamente del repositorio de Adrian: https://github.com/Rego523/GEI-TFG/blob/main/arduino/control_scalextric.ino

// Creamos dos objetos para los "rails" 1 y 2
AF_DCMotor rail1(1);  // Motor M1 del shield
AF_DCMotor rail2(2);  // Motor M2 del shield

void setup() {
  Serial.begin(115200);
  // Mensaje de arranque
  Serial.println("Control de Rails listo");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    // Esperamos comandos tipo "r1:0" a "r2:255"
    if (cmd.length() > 0 && cmd.charAt(0) == 'r') {
      int sep = cmd.indexOf(':');
      if (sep > 1) {
        int rail = cmd.substring(1, sep).toInt();
        int speed = cmd.substring(sep + 1).toInt();
        speed = constrain(speed, 0, 255);
        setRailSpeed(rail, speed);
      }
    }
  }
}

// Función auxiliar que aplica velocidad a uno de los dos motores
void setRailSpeed(int rail, int speed) {
  // 1) Elegimos a qué motor apuntar
  AF_DCMotor *motor = nullptr;
  if (rail == 1) {
    motor = &rail1;
  } else if (rail == 2) {
    motor = &rail2;
  } else {
    return;  // carril inválido, nada que hacer
  }

  // 2) Ajustamos freno o velocidad
  if (speed == 0) {
    motor->run(RELEASE);       // freno (0 PWM)
  } else {
    motor->setSpeed(speed);    // valor PWM 1–255
    motor->run(FORWARD);       // dirección “adelante”
  }

  // 3) Confirmación al PC
  Serial.print("OK r");
  Serial.print(rail);
  Serial.print(":");
  Serial.println(speed);
}
