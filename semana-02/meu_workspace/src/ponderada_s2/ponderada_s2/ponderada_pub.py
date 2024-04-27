import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher = self.create_publisher(
            msg_type=Twist,
            topic='/turtle1/cmd_vel',
            qos_profile=10
        )
        self.timer_period = 1.0
        self.side_count = 0
        self.max_sides = 4

    def straight(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        self.get_logger().info(f"Andando com a tartaruga: angular {msg.angular.z} e linear {msg.linear.x}")

    def curve(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = math.pi/3
        self.publisher.publish(msg)
        self.get_logger().info(f"Girando a tartaruga em 60 graus positivos: angular {msg.angular.z} e linear {msg.linear.x}")

    def timer_callback(self):
        if self.side_count < self.max_sides * 3:  # Multiplicando por 2 para garantir que a tartaruga desenhe todos os lados
            if self.side_count % 2 == 0:
                self.straight()
            else:
                self.curve()
            self.side_count += 1
        else:
            self.timer.cancel()  # Cancela o temporizador após desenhar todos os lados do polígono

def main(args=None):
    rclpy.init(args=args)
    tc = TurtleController()
    tc.timer = tc.create_timer(tc.timer_period, tc.timer_callback)
    rclpy.spin(tc)
    tc.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
