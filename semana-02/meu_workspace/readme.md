# Atividade Turltle draw ROS:

## Para conseguir rodar a minha atividade em seu computador é necessário seguir as intruções a seguir:

1 - Clone meu repositório em sua máquina, abra um terminal e navegue até onde gostaria de clonar o repositório, digite <b>git clone https://github.com/Rizzi26/Atividades-ponderadas-M6</b>

2 - Com o terminal aberto em sua máquina navegue até onde foi clonado o meu repositório e na pasta raiz digite: <b>colcon build</b>

&emsp;Esse comando irá instalar as dependências necessárias para rodar o meu pacote com o ROS <b>(pastas: build, log e install)</b>

3 - Agora com as dependencias instaladas vamos abrir um segundo terminal. Navegue até a pasta raiz do repositório tambem e digite o seguinte comando para iniciar a tela da nossa tartaruga: <b>ros2 run turtlesim turtlesim_node</b>

4 - Agora com a tela da tartaruga iniciada, podemos rodar no primeiro terminal que instanciamos o nosso pacote da ponderada. Digite o seguinte comando na pasta raiz do projeto: <b>ros2 run ponderada_s2 ponderada_pub</b>

# Assista o vídeo abaixo, o programa deve ser executado da seguinte forma e a tartaruga deve fazer o seguinte desenho:

[![Watch the video](https://davesroboshack.com/wp-content/uploads/2022/11/01_turtlesim-1024x638.png)](https://www.youtube.com/watch?v=4sZb-4vxiIw)

# Configuração dos entry-points:

<div align="center">
  <img src="\img\entry-points.png">
</div>

# Código que executa o publisher e envia os comandos para o turtlesim:

## 1- função que inicializa o nó:

<div align="center">
  <img src="\img\init.png">
</div>

## 2- Função que desenha uma reta:

<div align="center">
  <img src="\img\straight.png">
</div>

## 3- Função que faz uma curva:

<div align="center">
  <img src="\img\curve.png">
</div>

## 4- Função que publica os dados para o turtle:

<div align="center">
  <img src="\img\callback.png">
</div>





