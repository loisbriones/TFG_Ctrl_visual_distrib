cmake_minimum_required(VERSION 3.8)
project(image_processor_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 1. Generar mensajes personalizados 
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObjectLocation.msg"
  DEPENDENCIES builtin_interfaces
)

# 2. Incluir directorios de cabecera
include_directories(include)

# 3. Crear librería de detección de color
add_library(color_detector_lib src/color_detector.cpp)
target_include_directories(color_detector_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(color_detector_lib OpenCV)

# 4. Crear ejecutables y vincular mensajes
# Nodo Emisor (Publisher)
add_executable(publisher_node src/publisher.cpp)
target_link_libraries(publisher_node color_detector_lib)
ament_target_dependencies(publisher_node rclcpp sensor_msgs cv_bridge OpenCV)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(publisher_node "${cpp_typesupport_target}")

# Nodo Receptor (Subscriber)
add_executable(subscriber_node src/subscriber.cpp)
ament_target_dependencies(subscriber_node rclcpp)
target_link_libraries(subscriber_node "${cpp_typesupport_target}")

# 5. Instalación
install(TARGETS color_detector_lib publisher_node subscriber_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config msg
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)

ament_package()
