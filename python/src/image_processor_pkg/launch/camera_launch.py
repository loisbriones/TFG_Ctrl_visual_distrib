from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('image_processor_pkg')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    # Obtenemos el ID de la cámara de una variable de entorno (para que cada RPi sea única)
    node_id = os.environ.get('NODE_ID', 'camara_generica')

    return LaunchDescription([
        Node(
            package='image_processor_pkg',
            executable='publisher.py',
            name=f'publisher_{node_id}',
            namespace=node_id,
            parameters=[params_file], # Aquí ya lee la lista 'coches_activos'
            remappings=[('__ns', f'/{node_id}')]
        )
    ])