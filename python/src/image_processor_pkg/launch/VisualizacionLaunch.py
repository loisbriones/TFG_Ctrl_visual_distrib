from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("image_processor_pkg")
    params_file = os.path.join(pkg_share, "config", "params.yaml")

    # El launch mas simple del paquete: un unico nodo, sin namespace y sin
    # iterar sobre 'coches'. El panel descubre solo que coches y que camaras
    # hay mirando el grafo de ROS (topics /carX/position y /camara_XX/...),
    # asi que vale igual para una carrera de un coche que de dos, autonoma o
    # manual, y para reproducir un bag grabado hace meses.
    #
    # Del params.yaml solo usa el rango de PWM (controller.minimum_speed y
    # maximum_speed) como escala de la barra de velocidad de la tabla.
    return LaunchDescription(
        [
            Node(
                package="image_processor_pkg",
                executable="VisualizacionNode.py",
                name="visualizacion",
                parameters=[params_file],
                # El panel es lo unico que mira la persona durante la carrera:
                # si se cae, que vuelva solo como el resto de nodos
                respawn=True,
                respawn_delay=2.0,
                output="screen",
            )
        ]
    )
