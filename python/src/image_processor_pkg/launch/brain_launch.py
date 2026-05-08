from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('image_processor_pkg')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    # Cargamos la lista de coches desde el YAML para iterar sobre ella
    with open(params_file, 'r') as f:
        config = yaml.safe_load(f)
        # Accedemos a la lista definida en /**: ros__parameters
        coches = config['/**']['ros__parameters']['coches_activos']

    ld = LaunchDescription()

    # 1. Lanzamos un controlador por cada coche
    for car_name in coches:
        ld.add_action(Node(
            package='image_processor_pkg',
            executable='controller_node.py',
            name='controller',
            namespace=car_name, # Crea /coche_01/..., /coche_02/...
            parameters=[params_file, {'car_name': car_name}]
        ))

    # 2. Lanzamos el único nodo de Arduino
    ld.add_action(Node(
        package='image_processor_pkg',
        executable='arduino_bridge_node.py',
        name='arduino_bridge',
        parameters=[params_file]
    ))

    return ld