cmake_minimum_required(VERSION 3.8)
project(image_processor_pkg)

# Paquetes necesarios
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Genearar los mensajes
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObjectLocation.msg"
)

# Cargar los scripts de python: publisher y subscriber
install(PROGRAMS 
  image_processor_pkg/publisher.py
  image_processor_pkg/subscriber.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
