"""
Lanza los controladores de los coches en CARRERA MANUAL, los que conduce una
persona con el mando fisico. De la lista 'coches' del params.yaml arranca solo
los que tienen modo: "manual", y a diferencia de ControlLaunch NO lanza el
arduino_bridge, que es la garantia de que nada mueve el coche.

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

    # Hay que leer el YAML para saber cuantos coches hay que controlar
    with open(params_file, "r") as f:
        config = yaml.safe_load(f)
        params = config["/**"]["ros__parameters"]
        coches = params["coches"]
        cars = params["cars"]

    ld = LaunchDescription()

    # Un controlador por coche manual. El nodo se llama CarControllerManual_X
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

    return ld
