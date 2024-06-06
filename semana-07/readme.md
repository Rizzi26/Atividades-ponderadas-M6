# Atividade ponderada semana 08

## Sumário

- [Configuração do ambiente virtual para rodar a atividade](#configuração-do-ambiente-virtual-para-rodar-a-atividade)
- [Rodando meus scripts MLP em sua máquina](#rodando-meus-scripts-mlp-em-sua-máquina)
  - [Qual a diferença entre os scripts?](#qual-a-diferença-entre-os-scripts)
  - [Script MLP padrão (sem o uso da liby pyTorch)](#script-mlp-padrão-sem-o-uso-da-liby-pytorch)
  - [Script MLP com o uso da liby `pyTorch`](#script-mlp-com-o-uso-da-liby-pytorch)
- [Explicando o código](#explicando-o-código)
  - [MLP padrão](#mlp-padrão)
  - [MLP utilizando pyTorch](#mlp-utilizando-pytorch)
- [Comprovação do funcionamento da atividade](#comprovação-do-funcionamento-da-atividade)

&emsp;Objetivo: Implementar um **MLP** para resolver o problema do XOR. Execute um script implementando o algoritimo de forma padrão sem usar a biblioteca `pyTorch` e outro script utilizando o `pyTorch` para simplificar a implementação do **MLP**.

# Configuração do ambiente virtual para rodar a atividade:

&emsp;Primeiro você deve clonar em sua máquina meu repositório, escolha uma pasta como por exemplo `Documents` e navegue até ela no terminal, em seguida digite `git clone https://github.com/Rizzi26/Atividades-ponderadas-M6`.

&emsp;Agora com o repositório clonado ative a o ambiente virtual do python em sua máquina, `venv`, digite o seguinte comando `python3 -m venv venv` e ative o ambiente virtual logo em seguida com o comando `source ./venv/bin/activate`

&emsp;E por fim, instale as dependências necessárias que utilizei nas importações com o comando `python3 -m pip install -r requirements.txt`

# Rodando meus scripts MLP em sua máquina:

## Qual a diferença entre os scripts?

&emsp;A principal diferença entre os dois algoritmos é a implementação e a biblioteca utilizada. Ambos são modelos de rede neural artificial do tipo Perceptron de Múltiplas Camadas (MLP) usados para resolver o problema da porta lógica XOR, mas um é implementado com NumPy (Python puro) e o outro com PyTorch, uma biblioteca de aprendizado de máquina em Python.

## Script MLP padrão (sem o uso da liby pyTorch):

&emsp;Com o venv ativado, navegue até a seguinte pasta do meu repositório `.../Atividades-ponderadas-M6/semana-08` e digite o seguinte comando para rodar o script do MLP-padrão `python3 mlppadrao.py`. Após executar esse comando aguarde o script ser rodado e verifique o terminal da sua máquina para visualizar o resultado.

## Script MLP com o uso da liby `pyTorch`:

&emsp;Com o venv ativado, navegue até a seguinte pasta do meu repositório `.../Atividades-ponderadas-M6/semana-08` e digite o seguinte comando para rodar o script do MLP-torch `python3 mlptorch.py`. Após executar esse comando aguarde o script ser rodado e verifique o terminal da sua máquina para visualizar o resultado.

# Explicando o código

## MLP padrão:

### Aviso:

&emsp;Para a construção do código abaixo eu li o artigo disponibilizado no barema da atividade, que pode ser encontrado ao clicar [aqui](https://towardsdatascience.com/how-neural-networks-solve-the-xor-problem-59763136bdd7), e me baseei no código que ele aborda para a construção do MLP-Padrão.

```python 
import numpy as np
import math

class MLP:
    def __init__(self, train_data, target, lr=0.1, num_epochs=10000, num_input=2, num_hidden=2, num_output=1):
        self.train_data = train_data
        self.target = target
        self.lr = lr
        self.num_epochs = num_epochs

        self.weights_01 = np.random.uniform(size=(num_input, num_hidden))
        self.weights_12 = np.random.uniform(size=(num_hidden, num_output))
        self.b01 = np.random.uniform(size=(1, num_hidden))
        self.b12 = np.random.uniform(size=(1, num_output))

        self.losses = []

    def _sigmoid(self, x):
        return 1 / (1 + np.exp(-x))

    def _delsigmoid(self, x):
        return x * (1 - x)
```

&emsp;Aqui, a classe MLP que representa a rede neural é definida. Os métodos `__init__`, `_sigmoid`, e `_delsigmoid` são definidos. O método `__init__ ` inicializa os parâmetros da rede, incluindo pesos e vieses aleatórios. Os métodos `_sigmoid` e `_delsigmoid` são funções de ativação e suas derivadas, respectivamente.

```python
    def forward(self, batch):
        self.hidden_ = np.dot(batch, self.weights_01) + self.b01
        self.hidden_out = self._sigmoid(self.hidden_)

        self.output_ = np.dot(self.hidden_out, self.weights_12) + self.b12
        self.output_final = self._sigmoid(self.output_)

        return self.output_final
```

&emsp;O método `forward` é responsável por calcular as saídas da rede neural com base nas entradas fornecidas. Ele realiza a propagação direta (forward propagation) através das camadas oculta e de saída da rede.

```python
    def update_weights(self):
        loss = 0.5 * (self.target - self.output_final) ** 2
        self.losses.append(np.sum(loss))

        error_term = (self.target - self.output_final)
        grad12 = self.hidden_out.T @ (error_term * self._delsigmoid(self.output_final))
        grad01 = self.train_data.T @ (((error_term * self._delsigmoid(self.output_final)) @ self.weights_12.T) * self._delsigmoid(self.hidden_out))

        self.weights_01 += self.lr * grad01
        self.weights_12 += self.lr * grad12

        self.b01 += np.sum(self.lr * (((error_term * self._delsigmoid(self.output_final)) @ self.weights_12.T) * self._delsigmoid(self.hidden_out)), axis=0)
        self.b12 += np.sum(self.lr * error_term * self._delsigmoid(self.output_final), axis=0)
```

&emsp;O método `update_weights` é responsável por atualizar os pesos da rede neural com base no erro calculado durante a propagação direta (forward propagation). Ele usa o algoritmo de retropropagação (backpropagation) para calcular os gradientes e, em seguida, atualiza os pesos e os vieses.

```python 
    def train(self):
        for _ in range(self.num_epochs):
            self.forward(self.train_data)
            self.update_weights()
```

&emsp;O método `train` é responsável por treinar a rede neural. Ele executa um loop sobre o número especificado de épocas e, em cada época, realiza uma passagem para frente (forward pass) e uma atualização dos pesos (backward pass) usando o método `forward` e `update_weights`, respectivamente.

```python
    def predict(self, data):
        return self.forward(data)
```

&emsp;O método `predict` é usado para fazer previsões com base nos dados fornecidos. Ele simplesmente chama o método `forward` para calcular as saídas da rede neural.

```python
# Dados de treinamento para a porta XOR
train_data = np.array([[0, 0],
                       [0, 1],
                       [1, 0],
                       [1, 1]])

# Saídas esperadas para a porta XOR
target = np.array([[0], [1], [1], [0]])

# Criar o MLP
mlp = MLP(train_data, target)

# Treinar o MLP
mlp.train()

# Testar o MLP
print("Resultados após o treinamento:")
for i in range(len(train_data)):
    print(f"Entrada: {train_data[i]}, Saída esperada: {target[i]}, Saída do MLP: {mlp.predict(train_data[i])}")
```

&emsp;Aqui, aqui são definidos dados de treinamento e alvos para a porta lógica XOR. Em seguida, cria uma instância da classe MLP, a treina usando os dados de treinamento e imprime os resultados das previsões após o treinamento.

## MLP utilizando pyTorch:

```python 
import torch
import torch.nn as nn
import torch.optim as optim

# rede MLP
class MLP_torch(nn.Module):
    def __init__(self):
        super(MLP_torch, self).__init__()
        self.hidden = nn.Linear(2, 2)  # camada escondida
        self.output = nn.Linear(2, 1)  # camada de saída
        self.sigmoid = nn.Sigmoid()    # função de ativação

    def forward(self, x):
        x = self.sigmoid(self.hidden(x))
        x = self.sigmoid(self.output(x))
        return x
```

&emsp;Aqui, é definido a rede neural MLP usando PyTorch. A classe `MLP_torch` é uma subclasse de `nn.Module` do PyTorch, que permite a construção de modelos de rede neural. Ela tem duas camadas lineares (`nn.Linear`), uma camada oculta e uma camada de saída, e uma função de ativação sigmoidal.

```python 
# Dados de treinamento para a porta XOR
train_data = torch.tensor([[0, 0],
                           [0, 1],
                           [1, 0],
                           [1, 1]], dtype=torch.float32)

# Saídas esperadas para a porta XOR
target = torch.tensor([[0], [1], [1], [0]], dtype=torch.float32)

# Inicializar o MLP
mlp_torch = MLP_torch()

# Definir a função de perda e o otimizador
criterion = nn.MSELoss()
optimizer = optim.SGD(mlp_torch.parameters(), lr=0.2)
```

&emsp;Aqui, é definido os dados de treinamento e alvos para a porta lógica XOR. A rede neural MLP é inicializada usando a classe MLP_torch que defini anteriormente. Além disso, a função de perda (Mean Squared Error Loss) e um otimizador (Stochastic Gradient Descent) são definidos para treinar a rede.

```python
# Treinar o MLP
num_epochs = 5000
for epoch in range(num_epochs):
    optimizer.zero_grad()      # zerar os gradientes
    outputs = mlp_torch(train_data)  # previsão
    loss = criterion(outputs, target)  # calcular perda
    loss.backward()            # retropropagação
    optimizer.step()           # atualizar pesos

    if epoch % 1000 == 0:
        print(f'Epoch [{epoch}/{num_epochs}], Loss: {loss.item():.4f}')
```

&emsp;Aqui, ocorreo o treinamento da rede neural MLP. Para cada época, é feita uma previsão (`outputs`), calcula a perda (`loss`), realiza a retropropagação (`loss.backward()`), e, finalmente, atualiza os pesos (`optimizer.step()`). Também é impreimido a perda a cada 1000 épocas para acompanhar o progresso do treinamento.

```python 
# Testar o MLP
print("Resultados após o treinamento com PyTorch:")
with torch.no_grad():
    for i in range(len(train_data)):
        output = mlp_torch(train_data[i])
        print(f"Entrada: {train_data[i].numpy()}, Saída esperada: {target[i].numpy()}, Saída do MLP: {output.item():.4f}")
```

&emsp;Aqui, é testado a rede neural MLP após o treinamento. Previsões são feitas para cada exemplo de entrada nos dados de treinamento e imprime as entradas, saídas esperadas e saídas previstas pela MLP.

# Comprovação do funcionamento da atividade:

&emsp;Podemos verificar o que é impresso na saída do terminal ao executar ambos os scripts. Ao observar suas saídas, o algoritmo que apresentou melhor desempenho foi o **MLP-Padrão**. Isso pode ser atribuído às diferenças nos valores definidos para pesos, parâmetros e taxa de aprendizado em cada algoritmo, o que pode influenciar no resultado final. No entanto, ambos os algoritmos demonstram uma boa precisão, indicando uma ótima acurácia de previsão.

&emsp;Imagem: MLP-Padrão

<div align="center">
  <img src="\img\mlppadrao.png">
</div>

&emsp;Imagem: MLP-Torch

<div align="center">
  <img src="\img\mlptorch.png">
</div>
