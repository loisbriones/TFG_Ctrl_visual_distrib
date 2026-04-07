from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, EnvironmentVariable

def generate_launch_description():
    
    # Cargamos el fichero de configuracion: params.yaml 
    params_file = PathJoinSubstitution([
        FindPackageShare('image_processor_pkg'),
        'config',
        'params.yaml'
    ])

    # Optenemos el nombre del nodo
    node_id = EnvironmentVariable('NODE_ID', default_value='camara_generica')

    return LaunchDescription([
        Node(
            package='image_processor_pkg',
            executable='publisher.py', 
            namespace=node_id,
            parameters=[
                params_file,
                {'camara_id': node_id}
            ],
        )
    ])
