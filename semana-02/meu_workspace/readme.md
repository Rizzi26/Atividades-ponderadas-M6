&emsp;Para conseguir rodar a minha atividade em seu computador é necessário seguir as intruções a seguir:

1 - Clone meu repositório em sua máquina [https:](https://github.com/Rizzi26/Atividades-ponderadas-M6)

&emsp;Abra um terminal e navegue até onde gostaria de clonar o repositório, digite git clone https://github.com/Rizzi26/Atividades-ponderadas-M6

2 - Com o terminal aberto em sua máquina navegue até onde foi clonado o meu repositório e na pasta raiz digite: colcon build

&emsp;Esse comando irá instalar as dependências necessárias para rodar o meu pacote com o ROS (pastas: build, log e install)

3 - Agora com as dependencias instaladas vamos abrir um segundo terminal. Navegue até a pasta raiz do repositório tambem e digite o seguinte comando para iniciar a tela da nossa tartaruga: ros2 run turtlesim turtlesim_node

4 - Agora com a tela da tartaruga iniciada, podemos rodar no primeiro terminal que instanciamos o nosso pacote da ponderada. Digite o seguinte comando na pasta raiz do projeto: ros2 run ponderada_s2 ponderada_pub

O programa deve executar da seguinte forma e a tartaruga deve fazer o seguinte desenho:

![Video ROSS - Turtlesim](https://youtu.be/4sZb-4vxiIw)

&emsp;Configuração dos entry-points:

![Imagem](semana-02/meu_workspace/img/entry-points.png)

&emsp;Código que executa o publisher e envia os comandos para o turtlesim:

1- função que inicializa o nó:

![Imagem](semana-02/meu_workspace/img/init.png)

2- Função que desenha uma reta:

![Imagem](semana-02/meu_workspace/img/straight.png)

3- Função que faz uma curva:

![Imagem](semana-02/meu_workspace/img/curve.png)

4- Função que publica os dados para o turtle:

![Imagem](semana-02/meu_workspace/img/callback.png)




