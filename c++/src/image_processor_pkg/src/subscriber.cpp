
#include "rclcpp/rclcpp.hpp"
#include "image_processor_pkg/msg/object_location.hpp"
#include <fstream>
#include <iostream>

class PositionReceiver : public rclcpp::Node {
public:
  PositionReceiver() : Node("position_receiver") {
    csv_path = "/ros2_ws/src/image_processor_pkg/mediciones_cpp.csv";
    preparar_csv();

    sub = this->create_subscription<image_processor_pkg::msg::ObjectLocation>(
        "object_position", rclcpp::SensorDataQoS(),
        std::bind(&PositionReceiver::topic_callback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Nodo PositionReceiver C++ listo.");
  }

private:
  void preparar_csv() {
    std::ifstream check(csv_path);
    if (!check.is_open() || check.peek() == std::ifstream::traits_type::eof()) {
      std::ofstream f(csv_path);
      f << "timestamp_ns,color,cpu_proc_ms,network_lat_ms,total_lat_ms\n";
    }
  }

  void topic_callback(
      const image_processor_pkg::msg::ObjectLocation::SharedPtr msg) {
    auto now = this->now();
    auto stamp = rclcpp::Time(msg->stamp);

    double latencia_red_ms = (now - stamp).nanoseconds() / 1e6;
    double cpu_ms = msg->proc_time;
    double total_ms = cpu_ms + latencia_red_ms;

    std::ofstream f(csv_path, std::ios::app);
    f << now.nanoseconds() << "," << msg->color << "," << cpu_ms << ","
      << latencia_red_ms << "," << total_ms << "\n";

    RCLCPP_INFO(this->get_logger(),
                "RECIBIDO -> Color: %s | CPU: %.2fms | RED: %.2fms",
                msg->color.c_str(), cpu_ms, latencia_red_ms);
  }

  std::string csv_path;
  rclcpp::Subscription<image_processor_pkg::msg::ObjectLocation>::SharedPtr sub;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PositionReceiver>());
  rclcpp::shutdown();
  return 0;
}
