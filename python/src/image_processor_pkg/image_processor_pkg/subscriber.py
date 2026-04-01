#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from vision_tracker.msg import ObjectLocation

# Perfil para QoS preconfigurado, tiene:
#   History: Keep last,
#   Depth: 5,
#   Reliability: Best effort,
#   Durability: Volatile,
#   Deadline: Default,
#   Lifespan: Default,
#   Liveliness: System default,
#   Liveliness lease duration: default,
#   avoid ros namespace conventions: false
# Informacion sacada de: https://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1SensorDataQoS.html
from rclpy.qos import qos_profile_sensor_data


class PositionReceiver(Node):
    def __init__(self):
        super().__init__("position_receiver")

        self.subscription = self.create_subscription(
            ObjectLocation,
            "object_position",
            self.topic_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info("Node PositionReceiver Ready")

    def topic_callback(self, msg):
        color = msg.color
        cx = msg.x
        cy = msg.y

        self.get_logger().info(
            f"Recibido de la otra RPi -> Color: {color} en [{cx}, {cy}]"
        )


def main(args=None):
    rclpy.init(args=args)

    position_receiver = PositionReceiver()

    try:
        # Mantiene el nodo abierto procesando los mensajes entrantes
        rclpy.spin(position_receiver)
    except KeyboardInterrupt:
        pass
    finally:
        position_receiver.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
