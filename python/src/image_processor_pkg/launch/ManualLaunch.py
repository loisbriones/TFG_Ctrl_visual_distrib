"""
Despliegue del cerebro para coches en CARRERA MANUAL: los conduce una persona
con el mando físico del Scalextric y el sistema solo mira y anota.

De la lista 'coches' del params.yaml, este launch arranca SOLO los que tienen
cars.<coche>.modo: "manual". Los otros tres modos (incremental, automatico y
politica) publican PWM y necesitan el Arduino, así que los arranca BrainLaunch.
Para una carrera mixta (uno manual + uno de los otros) se levantan los dos
docker-compose a la vez: cada uno coge su subconjunto del mismo YAML.

Es BrainLaunch.py con tres diferencias, y las tres importan:

  1. NO se lanza el RaceControllerNode (arduino_bridge). Esta es la garantía de
     verdad de que nada mueve el coche: el Arduino queda fuera del circuito y
     el gatillo de la persona alimenta el carril. El parámetro modo="manual"
     del controlador es la segunda barrera, por si alguien levanta el puente a
     mano en otra terminal.

     Efecto secundario a tener en cuenta: sin el puente, nadie pone los
     carriles a arduino.calibration_speed. La vuelta de calibración hay que
     conducirla a mano, despacio y sin parar, porque de ella sale la
     trayectoria base contra la que se mide todo lo demás.

     (En un mixto donde BrainLaunch corre a la vez, el puente SÍ está y sí pone
     los carriles a calibration_speed, pero solo el suyo: el carril del coche
     manual sigue conduciéndose a mano.)

  2. modo="manual" en cada controlador: no publica órdenes de PWM y su
     terminal pasa a ser el salpicadero del piloto (tiempo de vuelta, si
     mejoró, mejor tiempo). El algoritmo sigue corriendo entero: su log guarda
     lo que HABRÍA hecho, que es justo lo que se quiere comparar después con
     lo que hizo la persona.

  3. El nodo se llama CarControllerManual_<coche>. EstrategiaPerfil nombra sus
     ficheros con el nombre del nodo y los abre en modo "w"
     (AlgoritmoVelocidad.py), así que con un nombre distinto los
     derrapesLog_CarControllerManual_<coche>_camara_XX.txt de una carrera
     manual no pisan los de una autónoma, y el coche va en el nombre para que
     dos coches tampoco se pisen entre sí. El análisis los encuentra igual:
     busca por el patrón derrapesLog_*.txt y saca la cámara de la línea [INIT]
     del propio fichero, no del nombre.

Uso:  docker compose -f docker-compose-manual.yml up
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("image_processor_pkg")
    params_file = os.path.join(pkg_share, "config", "params.yaml")

    # 1. Cargamos la lista de coches y su configuracion por coche desde el YAML
    with open(params_file, "r") as f:
        config = yaml.safe_load(f)
        params = config["/**"]["ros__parameters"]
        coches = params["coches"]
        cars = params["cars"]

    ld = LaunchDescription()

    # 2. Un controlador por coche con cars.<coche>.modo: "manual", igual que en
    #    BrainLaunch pero en manual. Los otros tres modos (incremental,
    #    automatico y politica) publican PWM y necesitan el Arduino, así que los
    #    arranca BrainLaunch. carril_asignado se mantiene aunque en manual no se
    #    use (publicar_velocidad sale antes): así el nodo es el mismo en los
    #    cuatro modos y el mensaje SpeedCarril sigue llevándolo.
    for car_name in coches:
        cfg = cars[car_name]
        if cfg.get("modo", "automatico") != "manual":
            continue

        ld.add_action(
            Node(
                package="image_processor_pkg",
                executable="CarControllerNode.py",
                name=f"CarControllerManual_{car_name}",
                namespace=car_name,
                parameters=[
                    params_file,
                    {
                        "car_name": car_name,
                        "carril_asignado": str(cfg["carril"]),
                        "modo": "manual",
                    },
                ],
                respawn=True,
                respawn_delay=2.0
            )
        )

    # 3. Aquí NO va el arduino_bridge: ver el punto 1 de la cabecera.

    return ld
