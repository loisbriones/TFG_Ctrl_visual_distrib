"""
Lanza el nodo camara

Uso:  NODE_ID=camara_01 CAMERA_DEV=0 docker compose -f docker-compose-camera.yml up
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    
    # Fichero con toda la configuracion del sistema
    params_file = PathJoinSubstitution([
        FindPackageShare('image_processor_pkg'),
        'config',
        'params.yaml'
    ])

    # Identifica a esta camara en toda la red. Es tambien su namespace
    node_id = DeclareLaunchArgument(
        'node_id',
        default_value='camera_00',
        description='Nombre con el que se va a identificar al nodo dentro de la red de ROS2'

    )

    # Que webcam de esta maquina hay que abrir
    camera_name = DeclareLaunchArgument(
        'camera_name',
        default_value='0',
        description='ID numerico que tiene la ruta (0) o ruta entera de la camara (/dev/video0)'
    )

    return LaunchDescription([
        camera_name,
        node_id,
        Node(
            package='image_processor_pkg',
            executable='CameraNode.py', 
            namespace=LaunchConfiguration('node_id'),
            parameters=[
                params_file,
                {'camara_id': LaunchConfiguration('node_id')},
                {'camera.device': LaunchConfiguration('camera_name')}
            ],
            respawn=True,
            respawn_delay=2.0
        )
    ])
