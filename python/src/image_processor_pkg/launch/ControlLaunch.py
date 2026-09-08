"""
Lanza un controlador por cada coche que NO sea manual (los modos incremental,
automatico y politica), el puente Arduino y el medidor de red. Lee params.yaml
por su cuenta para saber cuantos nodos tiene que crear.

Uso:  docker compose -f docker-compose-control.yml up
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("image_processor_pkg")
    params_file = os.path.join(pkg_share, "config", "params.yaml")

    # Hay que leer el YAML para saber cuantos controladores hay que levantar
    with open(params_file, "r") as f:
        config = yaml.safe_load(f)
        params = config["/**"]["ros__parameters"]
        coches = params["coches"]
        cars = params["cars"]

    ld = LaunchDescription()

    hay_autonomo = False
    for car_name in coches:

        cfg = cars[car_name]
        modo = cfg.get("modo", "automatico")

        if modo == "manual":
            # Los coches en modo manual usan otro launch
            continue

        hay_autonomo = True

        ld.add_action(
            Node(
                package="image_processor_pkg",
                executable="CarControllerNode.py",
                # Nombre unico por coche y por modo
                name=f"CarController_{modo}_{car_name}",
                namespace=car_name,
                parameters=[
                    params_file,
                    {
                        "car_name": car_name,
                        "carril_asignado": str(cfg["carril"]),
                        "modo": modo,
                    },
                ],
                respawn=True,
                respawn_delay=2.0,
            )
        )

    if hay_autonomo:
        # El puente solo hace falta si hay algun coche al que aplicarle un PWM
        ld.add_action(
            Node(
                package="image_processor_pkg",
                executable="RaceControllerNode.py",
                name="arduino_bridge",
                parameters=[params_file],
            )
        )

    # El medidor de latencia de red solo se levanta si esta activado
    if params.get("net_probe", {}).get("enabled", True):
        ld.add_action(
            Node(
                package="image_processor_pkg",
                executable="NetProbeNode.py",
                name="net_probe",
                parameters=[params_file],
                respawn=True,
                respawn_delay=2.0,
            )
        )

    return ld
