from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    
    # Cargamos el fichero de configuracion: params.yaml 
    params_file = PathJoinSubstitution([
        FindPackageShare('image_processor_pkg'),
        'config',
        'params.yaml'
    ])

    # Obtenemos el nombre del nodo
    node_id = DeclareLaunchArgument(
        'node_id',
        default_value='camera_00',
        description='Nombre con el que se va a identificar al nodo dentro de la red de ROS2'

    )

    # Nombre de la camara que queremos levantar
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
