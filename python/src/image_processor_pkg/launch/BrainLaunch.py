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

    # 2. Un controlador por cada coche AUTONOMO (modo_manual: false). Los coches
    #    marcados como manuales los arranca ManualLaunch; para una carrera mixta
    #    (uno manual + uno autonomo) se levantan los dos docker-compose a la vez
    #    y cada launch coge su subconjunto del mismo params.yaml.
    hay_autonomo = False
    for car_name in coches:
        cfg = cars[car_name]
        if cfg.get("modo_manual", False):
            continue
        hay_autonomo = True

        ld.add_action(
            Node(
                package="image_processor_pkg",
                executable="CarControllerNode.py",
                # Nombre UNICO por coche: EstrategiaPerfil nombra sus logs con el
                # nombre del nodo (derrapesLog_<nodo>_<camara>.txt); con el mismo
                # nombre dos coches se pisarian los ficheros.
                name=f"CarController_{car_name}",
                namespace=car_name,
                parameters=[
                    params_file,
                    {
                        "car_name": car_name,
                        # Carril fisico del coche (cars.<coche>.carril), no por
                        # orden de la lista: el controlador lo mete en el mensaje
                        # SpeedCarril y el arduino_bridge lo usa para el carril.
                        "carril_asignado": str(cfg["carril"]),
                        "modo_manual": False,
                    },
                ],
                respawn=True,
                respawn_delay=2.0,
            )
        )

    # 3. El arduino_bridge solo se lanza si hay algun coche autonomo que mande
    #    PWM. Si todos los coches son manuales, no aporta nada y no se arranca.
    if hay_autonomo:
        ld.add_action(
            Node(
                package="image_processor_pkg",
                executable="RaceControllerNode.py",
                name="arduino_bridge",
                parameters=[params_file],
            )
        )

    return ld
