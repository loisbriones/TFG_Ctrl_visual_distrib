cmake_minimum_required(VERSION 3.8)
project(image_processor_pkg)

# Paquetes necesarios
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Genearar los mensajes
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CarLocation.msg"
  "msg/LineSegment.msg"
  "msg/FinishLine.msg"
  "msg/Point2D.msg"
  "msg/Stiker.msg"
  "msg/SpeedCarril.msg"
  "msg/TimePerLap.msg"
  "msg/CarControlTelemetry.msg"
  "msg/BoundingRect.msg"
  DEPENDENCIES builtin_interfaces
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Cargar los scripts de python
install(PROGRAMS 
  image_processor_pkg/CameraNode.py
  image_processor_pkg/subscriber.py
  image_processor_pkg/ProcessImage.py 
  image_processor_pkg/RaceControllerNode.py
  image_processor_pkg/ArduinoController.py
  image_processor_pkg/CarControllerNode.py
  image_processor_pkg/AlgoritmoVelocidad.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
