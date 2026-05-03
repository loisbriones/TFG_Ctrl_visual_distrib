#include "rclcpp/rclcpp.hpp"
#include "image_processor_pkg/msg/object_location.hpp"
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <filesystem> // Para verificar si el archivo existe y su tamaño

class PositionReceiver : public rclcpp::Node {
public:
  PositionReceiver() : Node("position_receiver") {
    // Ruta base para los archivos CSV
    csv_base_path = "/ros2_ws/src/image_processor_pkg/mediciones_cpp_";

    // Nos subscribimos al topic que publica la informacion
    sub = this->create_subscription<image_processor_pkg::msg::ObjectLocation>(
        "/object_position", rclcpp::SensorDataQoS(),
        std::bind(&PositionReceiver::topic_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Nodo PositionReceiver C++ listo.");
  }

  // Destructor para cerrar los archivos correctamente al terminar
  ~PositionReceiver() {
    for (auto const& [id, file_ptr] : recursos) {
      if (file_ptr->is_open()) {
        file_ptr->close();
      }
    }
  }

private:
  // Función para obtener o crear el recurso de escritura
  std::ofstream& obtener_recurso(const std::string& node_id) {

    // Comprobamos si ya esta abierto el fichero. El recurso esta guardado en el diccinario
    if (recursos.find(node_id) == recursos.end()) {
      std::string file_path = csv_base_path + node_id + ".csv";
      
      // Comprobar si necesita cabecera (si no existe o está vacío)
      bool necesita_cabecera = !std::filesystem::exists(file_path) || std::filesystem::file_size(file_path) == 0;
      
      // Creamos el puntero al archivo en modo append
      auto f = std::make_unique<std::ofstream>(file_path, std::ios::app);
      
      if (necesita_cabecera) {
          // Añadimos la cabecera al fichero
          *f << "timestamp_ns,color,pos_x,pos_y,cpu_proc_ms,network_lat_ms,total_lat_ms\n";
          // Forzamos la escritura de la cabecera para evitar errores
          f->flush();
      }
      
      recursos[node_id] = std::move(f);
    }
    
    // Devolvemos el puntero al recuros donde guardar los datos
    return *recursos[node_id];
  }

  void topic_callback(const image_processor_pkg::msg::ObjectLocation::SharedPtr msg) {

    // Tiempo actual
    auto now = this->now();
    // Tiempo de envío desde el mensaje
    auto stamp = rclcpp::Time(msg->stamp);

    // 1. Latencia de red pura
    double latencia_red_ms = (now - stamp).nanoseconds() / 1e6;
    
    // 2. Tiempo de procesamiento (del mensaje)
    double cpu_ms = msg->proc_time;
    
    // 3. Latencia Total
    double total_ms = cpu_ms + latencia_red_ms;

    // --- GUARDAR DATOS ---
    std::ofstream& f = obtener_recurso(msg->node_id);

    f << std::fixed << std::setprecision(4);
    f << now.nanoseconds() << "," 
      << msg->color << "," 
      << msg->x << ","        
      << msg->y << ","        
      << cpu_ms << ","
      << latencia_red_ms << "," 
      << total_ms << "\n";

    // --- LOGS POR CONSOLA ---
    RCLCPP_INFO(this->get_logger(),
                "RECIBIDO -> Color: %s | CPU: %.2fms | RED: %.2fms",
                msg->color.c_str(), cpu_ms, latencia_red_ms);
  }

  std::string csv_base_path;
  // Diccionario (map) para gestionar los escritores de CSV
  std::map<std::string, std::unique_ptr<std::ofstream>> recursos;
  rclcpp::Subscription<image_processor_pkg::msg::ObjectLocation>::SharedPtr sub;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PositionReceiver>());
  rclcpp::shutdown();
  return 0;
}