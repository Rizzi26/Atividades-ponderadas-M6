# Navegação

- [Configuração do ambiente virtual para rodar a atividade](#configuração-do-ambiente-virtual-para-rodar-a-atividade)
- [Configuração do pacote python](#configuração-do-pacote-python)
- [Rodando meu pacote em seu computador](#rodando-meu-pacote-em-seu-computador)
- [Explicação do código e suas principais funções](#explicação-do-código-e-suas-principais-funções)
  - [Imports](#imports)
  - [Class TurtleBot: Interação com o robô](#class-turtlebot-interação-com-o-robô)
  - [Execução do pacote](#execução-do-pacote)
  - [Função inicializadora](#função-inicializadora)
- [Vídeo que comprova plenamente o funcionamento do sistema criado](#vídeo-que-comprova-plenamente-o-funcionamento-do-sistema-criado)
    
# Atividade ponderada semana 05

&emsp;Objetivo: Construir uma CLI (Command line interface) para operar o robô, utilizando os principais serviços do ROS2 para demonstrar as habilidades aprendidas durante as instruções até o momento.

#### Aviso: 

&emsp;Durante a sprint 2 trabalhei em conjunto com os integrantes Gustavo Widman e Rodrigo na construção da CLI e da classe do robô. Portanto, a construção da minha atividade ponderada **pode conter** pedaços de códigos que **reutilizei.**

# Configuração do ambiente virtual para rodar a atividade:

&emsp;Primeiro você deve clonar em sua máquina meu repositório, escolha uma pasta como por exemplo `Documents` e navegue até ela no terminal, em seguida digite `git clone https://github.com/Rizzi26/Atividades-ponderadas-M6`.

&emsp;Agora com o repositório clonado ative a o ambiente virtual do python em sua máquina `venv`, digite o seguinte comando `python3 -m venv venv` e ative o ambiente virtual logo em seguida com o comando `source ./venv/bin/activate`

&emsp;E por fim, instale as dependências necessárias que utilizei nas importações com o comando `python3 -m pip install -r requirements.txt`

# Configuração do pacote python:

&emsp;Em seu termnial navegue até o caminho `.../Atividades-ponderadas-M6/semana-05/meu_workspace/` e digite os seguintes comandos para instalar as dependências necessárias do pacote que fiz `colcon build` e em seguida `source install/local_setup.bash`

&emsp;Agora seu ambiente está configurado para interagir com o robô. Em meu desenvolvimento da atividade utilizo o simulador por questão de praticidade, você pode utilizar támbem caso tenha o we-bots instalado, caso não tenha acesse [esse](https://rmnicola.github.io/m8-ec-encontros/sprint2/encontro4/nav2/#1-setup) tutorial que o professor disponibilizou para instala-lo, siga da seção `5.0 Trocando o Gazebo pelo webots` até o primeiro tópico da seção `5.2 Rodando o nav2 com o webots`.

# Rodando meu pacote em seu computador:

&emsp;Como disse anteriormente, utilizei o simulador we-bots para interagir com o robô por questões de praticidade, portanto vou ensinar a rodar e interagir com meu pacote a partir dele.

&emsp;Após instalar o simulador, abra um terminal em seu computado e digite o seguinte comando `ros2 launch webots_ros2_turtlebot robot_launch.py`, o computador irá executar e lançar o simulador em sua tela exbindo um ambiente 3D no qual é possível simular a teleoperação do nosso robô.

&emsp;Agora com o simulador ativado e com toda a configuração pra rodar meu pacote podemos finalmente rodar o pacote que criamos para interagir com o robô, em um outro terminal acesse meu repositório e navegue até o seguinte caminho `.../Atividades-ponderadas-M6/semana-05/meu_workspace` e rode o comando `ros2 run semana_05 semana_05` e deve aparecer em seu terminal mensagens de incialização e a Command Line Interface que produzi para operar o robô, agora é só se divertir ;)

# Explicação do código e suas principais funções:

### Imports:

&emsp;Aqui podemos observar os imports realizados em meu código para o seu funcionamento:

- `typer` para criar uma interface de linha de comando.
- `rclpy` para trabalhar com **ROS 2** (Robot Operating System 2).
- `Twist` e `Vector3` do **ROS** para enviar comandos de movimento.
- `time` para lidar com temporização.
- `keyboard` para capturar entrada de teclado.
- `sys` para interação com o sistema.
- `tty` e `termios` para configuração do terminal. 
- `yaspin` para exibir um spinner (indicador de progresso) durante a inicialização.

```python 
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
```

&emsp;Inicialização do app typer e o spinner da liby yaspin:

```python 
# Criação do app e do spinner
app = typer.Typer()
spinner = yaspin(text="Etapa inicializada com sucesso...", color="green")
```

### Class TurtleBot: Interação com o robô

&emsp;Esta classe representa o nó do robô. Ele inicializa a comunicação com o ROS, cria um publicador para enviar comandos de movimento para o robô e define métodos para lidar com entrada de teclado, atualizar o movimento do robô, parar o robô em caso de emergência e exibir o status do robô.

```python 
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
                self.destroy_node()
                rclpy.shutdown()
                self.stop_robot_client()
                exit()

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
```

&emsp;Podemos observar importantes funções em nossa classe, tais como as de movimentação do robô e interação com a CLI:

- `def __init__(self):` essa função é responsável por inicializar o TurtleBot, Publisher, Interação com o teclaco, seta as velocidades em que o robô vai se mexer ao receber seus comandos e cria um timer para atualizar o status do robô a cada 0.1 segundos.

- `def on_press(self, key):` essa função é responsável por realizar a leitura das teclas em que estão sendo pressionadas e enviar o "sinal" pressionado para a função de movimentação `def update_moviment(sefl):` ou de parada para a função `def stop_robot_client(self):`.

- `def on_release(self, key)` essa função é responsável por parar o robô ao soltar as teclas de pressão definidas.

- `def update_moviment(self):` essa função é responsável por enviar ao nosso publisher, que em seguida trasnmite ao robô, a velociade linerar e angular em que ele deve se mover de acordo com a tecla pressionada.

- `def emergency_stop(self):` essa função é chamada ao pressionarmos a tecla `Q` que aciona nosso sistema de emergência, ela zera a velociade do robô, tanto angular quanto linear. 

- `def display_status(self):` essa função é chamda a cada 0.1 segundos como definimos no nosso timer dentro da função `def __init__(sefl):`, ela limpa o terminal a cada 0.1 segundos e atualiza o status do roô (velocidade angular, linear e status de conexâo) assim, consegue evitar que os logs atrapalhem a visualização da nossa CLI e consegue transmitir em tempo real o statsu do robô.

- `def stop_robot_client(self):` Este método chama um serviço chamado stop_robot para parar o robô para destruir o nó, ela verifica se há como se conectar ao um novo serviço em 0.1 segundos por chamada, caso não tenha nenhum disponível ele encerra o code.

### Execução do pacote

&emsp;Para que tudo aconteça e rode conforme o programado precisamos configurar nossas funções corretamente para a execução do pacote. A função **main()** é chamada se o script for executado como um programa principal.

```python
# Função para chamar o app ao rodar o pacote 
if __name__ == "__main__":
    main()
```

### Função inicializadora

&emsp;Ao chamar nosso script como um programa principal chamamos a função **main()** que em seguida incializa nossas dependências e a nossa classe `TurtleBot()`. Podemos observar que temos alguns comandos interessantes, como um que evita a exibição das teclas que foram clicadas em nosso terminal, isso ajuda a garantir uma melhor experiência para o usuário em geral:

```python 
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
```

# Vídeo que comprova plenamente o funcionamento do sistema criado:

&emsp;No vídeo eu rodo o simulador webots e depois rodo o meu pacote para interagir com o robô. Faço algumas movimentações, para frente 'W', para a direita 'D', para esquerda 'A' e para trás 'S', podemos ver que o status do robô é atualizado em tempo real ao pressionar as teclas e as velocidades angulares e lineares são alteradas. Ao fim, eu pressiono a tecla 'Q' para ativar o sistma de emergência do robô e fazer com que a aplicação pare, no terminal podemos ver que o status da conexão é perdida e aparece um print no "Chamando o serviço para parar o robô...", "Serviço não disponível." e code para de rodar no terminal.

[![Vídeo que comprova plenamente o funcionamento do sistema criado](https://demura.net/wordpress/wp-content/uploads/2021/04/webot3.png)](https://youtu.be/YAQR5O36W9Y)






