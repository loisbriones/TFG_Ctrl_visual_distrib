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
  # Sondeos de latencia de red (NetProbeNode <-> CameraNode). No son parte del
  # camino de datos: se pueden apagar enteros con net_probe.enabled
  "msg/NetProbe.msg"
  "msg/NetProbeEcho.msg"
  DEPENDENCIES builtin_interfaces
)

# web/ son los tres ficheros de la pagina del panel en directo (index.html,
# estilos.css y panel.js). Van al share, como config y launch, y el nodo los
# encuentra con get_package_share_directory
install(DIRECTORY config launch web
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
  # Panel de la carrera en directo. Su pagina no va aqui: son los tres ficheros
  # de web/, que se instalan arriba en el share
  image_processor_pkg/VisualizacionNode.py
  # Medidor de latencia de red. No forma parte del camino de datos: se puede
  # apagar entero con net_probe.enabled sin desmontar nada
  image_processor_pkg/NetProbeNode.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
