# imports para criar uma interface no terminal e controlar o robô
import typer
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
import time
from pynput import keyboard
import sys
import tty
import termios
from yaspin import yaspin
from std_srvs.srv import Empty

# Criação do app e do spinner
app = typer.Typer()
spinner = yaspin(text="Etapa inicializada com sucesso...", color="green")

# Função principal para inicializar o robô e controlá-lo
class TurtleBot(Node):
    def __init__(self):
        print("Inicializando o nó do TurtleBot...")
        time.sleep(1)
        spinner.ok("✔")
        super().__init__('turtlebot')
        self.connected = True
        print("Criando o publicador para o tópico 'cmd_vel'...")
        time.sleep(1)
        spinner.ok("✔")
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.linear_speed = 1.0
        self.angular_speed = 2.0
        self.key_listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.key_listener.start()
        self.current_linear = Vector3(x=0.0, y=0.0, z=0.0)
        self.current_angular = Vector3(x=0.0, y=0.0, z=0.0)
        print(f"TurtleBot inicializado com sucesso, status da conexão: {self.connected}.")
        spinner.ok("✔")
        time.sleep(1)

        self.create_timer(0.1, self.display_status)

    # Função para controlar o robô com as teclas 'W', 'A', 'S', 'D' e 'Q'
    def on_press(self, key):
        try:
            if key.char == 'w':
                self.current_linear.x = self.linear_speed
            elif key.char == 's':
                self.current_linear.x = -self.linear_speed
            elif key.char == 'a':
                self.current_angular.z = self.angular_speed
            elif key.char == 'd':
                self.current_angular.z = -self.angular_speed
            elif key.char == 'q':
                self.emergency_stop()
                self.connected = False
                self.stop_robot_client()

        except AttributeError:
            pass
        self.update_movement()

    # Função para parar o robô quando a tecla é solta
    def on_release(self, key):
        try:
            if key.char in ['w', 's']:
                self.current_linear.x = 0.0
            elif key.char in ['a', 'd']:
                self.current_angular.z = 0.0
        except AttributeError:
            pass
        self.update_movement()
    
    # Função para atualizar o movimento do robô
    def update_movement(self):
        msg = Twist()
        msg.linear = self.current_linear
        msg.angular = self.current_angular
        self.publisher_.publish(msg)

    # Função para parar o robô em caso de emergência
    def emergency_stop(self):
        self.current_linear = Vector3(x=0.0, y=0.0, z=0.0)
        self.current_angular = Vector3(x=0.0, y=0.0, z=0.0)
        self.update_movement()

    # Função para exibir o status do robô em tempo real
    def display_status(self):
        print("\033c", end="")
        print("Controle o robô usando as teclas:")
        print("    W - Frente")
        print("    A - Esquerda")
        print("    S - Trás")
        print("    D - Direita")
        print("Pressione 'Q' para parar o robô e sair.")
        print("\nStatus do Robô:")
        connection_status = "Conectado" if self.connected else "Não Conectado"
        print(f"  Conexão: {connection_status}")
        print(f"  Velocidade Linear: x={self.current_linear.x}, y={self.current_linear.y}, z={self.current_linear.z}")
        print(f"  Velocidade Angular: x={self.current_angular.x}, y={self.current_angular.y}, z={self.current_angular.z}")

    # Função para chamar o serviço de parar o robô
    def stop_robot_client(self):
        print("Chamando o serviço para parar o robô...")
        client = self.create_client(Empty, 'stop_robot')
        if client.wait_for_service(timeout_sec=0.1):
            request = Empty.Request()
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            self.get_logger().info('Serviço chamado com sucesso.')
        else:
            self.get_logger().info('Serviço não disponível.')
        self.destroy_node()
        rclpy.shutdown()
        exit()


# Função principal para inicializar o robô e controlá-lo
def main():
    print("Inicializando rclpy...")
    time.sleep(1)
    spinner.ok("✔")
    rclpy.init(args=None)
    print("Criando a instância do TurtleBot...")
    time.sleep(1)
    spinner.ok("✔")
    robot = TurtleBot()

    print(
"""
Controle o robô usando as teclas:
    W - Frente
    A - Esquerda
    S - Trás
    D - Direita
Pressione 'Q' para parar o robô e sair.
"""
    )

    # para não exibir no terminal as teclas clicadas
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(sys.stdin.fileno())
        rclpy.spin(robot)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

# Função para chamar o app ao rodar o pacote 
if __name__ == "__main__":
    main()
