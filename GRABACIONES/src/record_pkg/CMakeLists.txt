cmake_minimum_required(VERSION 3.8)
project(image_processor_pkg)

# Paquetes necesarios
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Generar los mensajes: se compilan automáticamente todos los .msg de msg/
# (para añadir un mensaje nuevo basta con dejar el archivo en esa carpeta)
file(GLOB MSG_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "msg/*.msg")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES builtin_interfaces
)

ament_package()
