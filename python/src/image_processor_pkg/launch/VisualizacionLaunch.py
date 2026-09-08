"""
Lanza el panel de la carrera en directo, un solo nodo y sin namespace.

Uso:  ./arrancar_visualizacion.sh
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("image_processor_pkg")
    params_file = os.path.join(pkg_share, "config", "params.yaml")

    return LaunchDescription(
        [
            Node(
                package="image_processor_pkg",
                executable="VisualizacionNode.py",
                name="visualizacion",
                parameters=[params_file],
                respawn=True,
                respawn_delay=2.0,
                output="screen",
            )
        ]
    )
