import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtle import *

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')  # Inicializa o nó
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)  # Publicador para comandos de movimento
        self.timer_period = 1.0  # Período do temporizador
        self.side_count = 0  # Contador de lados do polígono
        self.max_sides = 4  # Número máximo de lados

    def straight(self):
        msg = Twist()
        msg.linear.x = 2.0  # Movimento linear para frente
        self.publisher.publish(msg)

    def curve(self):
        msg = Twist()
        msg.angular.z = 1.047  # Rotação de 60 graus
        self.publisher.publish(msg)

    def timer_callback(self):
        if self.side_count < self.max_sides * 3:  # Desenha todos os lados do polígono
            if self.side_count % 2 == 0:
                self.straight()  # Movimento linear em lados pares
            else:
                self.curve()  # Curva em lados ímpares
            self.side_count += 1
        else:
            self.timer.cancel()  # Cancela o temporizador

def main(args=None):
    rclpy.init(args=args)
    tc = TurtleController()
    tc.timer = tc.create_timer(tc.timer_period, tc.timer_callback)
    rclpy.spin(tc)
    tc.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
