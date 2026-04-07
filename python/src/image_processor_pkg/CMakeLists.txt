cmake_minimum_required(VERSION 3.8)
project(image_processor_pkg)

# Paquetes necesarios
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Genearar los mensajes
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObjectLocation.msg"
  DEPENDENCIES builtin_interfaces
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Cargar los scripts de python: publisher y subscriber
install(PROGRAMS 
  image_processor_pkg/publisher.py
  image_processor_pkg/subscriber.py
  image_processor_pkg/procesar_imagen.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
