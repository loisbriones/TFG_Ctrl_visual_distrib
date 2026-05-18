from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("image_processor_pkg")
    params_file = os.path.join(pkg_share, "config", "params.yaml")

    # 1. Cargamos la lista de coches desde el YAML
    with open(params_file, "r") as f:
        config = yaml.safe_load(f)
        # CORRECCIÓN: Cambiado 'coches_activos' a 'coches' para que coincida con params.yaml
        coches = config["/**"]["ros__parameters"]["coches"]

    ld = LaunchDescription()

    # 2. Lanzamos un controlador por cada coche
    for indice, car_name in enumerate(coches):
        # Asignamos dinámicamente el carril (car1 -> "r1", car2 -> "r2", etc.)
        carril_id = f"r{indice + 1}"

        ld.add_action(
            Node(
                package="image_processor_pkg",
                executable="controller_node.py",
                name="controller",
                namespace=car_name,
                parameters=[
                    params_file,
                    {
                        "car_name": car_name,
                        "carril_asignado": carril_id,  # Pasamos el carril directamente por parámetro
                    },
                ],
            )
        )

    # 3. Lanzamos el único nodo de Arduino
    # ld.add_action(Node(
    #    package='image_processor_pkg',
    #    executable='arduino_bridge_node.py',
    #    name='arduino_bridge',
    #    parameters=[params_file]
    # ))

    return ld

