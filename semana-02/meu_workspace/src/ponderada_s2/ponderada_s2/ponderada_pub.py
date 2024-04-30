import rclpy
import asyncio
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, Kill, SetPen

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller') 
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)  
        self.timer_period = 1.0
        self.side_count = 0 
        self.max_sides = 4  

        # Clientes e serviços
        self.spawn_client = self.create_client(Spawn, '/spawn')
        self.kill_client = self.create_client(Kill, '/kill')
        self.set_pen_client = self.create_client(SetPen, '/turtle1/set_pen')

    def straight(self):
        msg = Twist()
        msg.linear.x = 2.0  # Movimento linear para frente
        self.publisher.publish(msg)

    def curve(self):
        msg = Twist()
        msg.angular.z = 1.047  # Rotação de 60 graus
        self.publisher.publish(msg)

    async def set_pen_color(self, r, g, b):
        node = rclpy.create_node('set_pen_color')
        set_pen_client = node.create_client(SetPen, '/turtle1/set_pen')
        while not set_pen_client.wait_for_service(timeout_sec=1.0):
            print('Service not available, waiting again...')
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = 5  # Mantém a largura da linha inalterada
        request.off = 0    # Mantém a caneta ligada
        future = set_pen_client.call_async(request)
        rclpy.spin_until_future_complete(node, future)

    async def timer_callback(self):
        await self.set_pen_color(255, 0, 0)
        if self.side_count < self.max_sides * 3:  # Desenha todos os lados do polígono
            if self.side_count % 2 == 0:
                self.straight()  # Movimento linear em lados pares
            else:
                self.curve()  # Curva em lados ímpares
            self.side_count += 1
        else:
            node = rclpy.create_node('kill_turtle')
            client = node.create_client(Spawn, '/spawn')

            request = Spawn.Request()
            request.x = 7.0
            request.y = 7.0
            request.theta = 0.0
            request.name = 'turtle2'

            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future)

            if future.result() is not None:
                print('Tartaruga criada')
            else:
                print('Falha ao criar tartaruga')

            time.sleep(2)

            Kill_request = Kill.Request()
            Kill_request.name = 'turtle2'
            kill_client = node.create_client(Kill, '/kill')

            future = kill_client.call_async(Kill_request)
            rclpy.spin_until_future_complete(node, future)
            
            if future.result() is not None:
                print('Tartaruga morta')
            else:
                print('Falha ao matar tartaruga')

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
