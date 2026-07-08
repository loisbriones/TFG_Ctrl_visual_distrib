from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, EnvironmentVariable, LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    
    # Cargamos el fichero de configuracion: params.yaml 
    params_file = PathJoinSubstitution([
        FindPackageShare('image_processor_pkg'),
        'config',
        'params.yaml'
    ])

    # Optenemos el nombre del nodo
    node_id = EnvironmentVariable('NODE_ID', default_value='camera_00')
    # Nombre de la camara que queremos levantar
    camera_name = DeclareLaunchArgument(
        'camera_name',
        default_value='0',
        description='ID numerico que tiene la ruta (0) o ruta entera de la camara (/dev/video0)'
    )

    return LaunchDescription([
        camera_name,
        Node(
            package='image_processor_pkg',
            executable='CameraNode.py', 
            namespace=node_id,
            parameters=[
                params_file,
                {'camara_id': node_id},
                {'camera.device': LaunchConfiguration('camera_name')}
            ],
            respawn=True,
            respawn_delay=2.0
        )
    ])
