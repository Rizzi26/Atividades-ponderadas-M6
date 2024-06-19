# Atividade ponderada semana 10

## Sumário

- [Configuração do ambiente virtual para rodar a atividade](#configuração-do-ambiente-virtual-para-rodar-a-atividade)
- [Rodando a atividade](#rodando-a-atividade)

&emsp;Objetivo: Por meio de um modelo identificar algarismos que fizemos escrito na mão ao enviar para o servidor.

# Configuração do ambiente virtual para rodar a atividade:

&emsp;Primeiro você deve clonar em sua máquina meu repositório, escolha uma pasta como por exemplo `Documents` e navegue até ela no terminal, em seguida digite `git clone https://github.com/Rizzi26/Atividades-ponderadas-M6`.

&emsp;Agora com o repositório clonado ative a o ambiente virtual do python em sua máquina, `venv`, digite o seguinte comando `python3 -m venv venv` e ative o ambiente virtual logo em seguida com o comando `source ./venv/bin/activate`

&emsp;E por fim, instale as dependências necessárias que utilizei nas importações com o comando `python3 -m pip install -r requirements.txt`

# Rodando a atividade

&emsp;Caso você for treinar os modelos em seu computador é necessário baixar o csv de test e train e adicionar os arquivos na pasta src (raiz do servidor).

&emsp;Aqui estamos referenciando os imports que utilizamos em nosso codigo:

```python
from flask import Flask, render_template, request
import os
import cv2
import numpy as np
from modelos.convulocional import Convulocional
from modelos.linear import LinearModel
```

&emsp;Quando você estiver na pasta raiz da atividade `src` rode o seguinte comando `python3 main.py` e o arquivo iŕa criar nosso servidor Flask e primeiro verificar se existe os modelos treinados (linear e convulocional) caso não exista esse arquivo irá chamar os arquivos `train_linear.py` e `train_convulocional.py` para treinar os modelos. 

```python
app = Flask(__name__, template_folder='templates')

# Criando a pasta de uploads se não existir
if not os.path.exists('uploads'):
    os.makedirs('uploads')

# Verificar se os modelos existem
model_path = 'modelo_convulocional.h5'
model_path_linear = 'modelo_linear.h5'

if not os.path.exists(model_path):
    print("Modelo Convulocional não encontrado. Treinando o modelo...")
    os.system('python3 train_convulocional.py')  # Comando para treinar o modelo
else:
    print(f"Carregando modelo Convulocional existente de {model_path}")

if not os.path.exists(model_path_linear):
    print("Modelo Linear não encontrado. Treinando o modelo...")
    os.system('python3 train_linear.py')  # Comando para treinar o modelo
else:
    print(f"Carregando modelo Linear existente de {model_path_linear}")
```

&emsp;Agora o code irá chamar nossas classes, que irão posteriormente realizar a predição da imagem. Depois o a rota `/` irá renderizar nosso `index.html` que está presente na pasta templates.

```python
 Inicializar os modelos Convulocional e Linear
model_convulocional = Convulocional(model_path)
model_linear = LinearModel((784,), 10)  # Ajuste para inicializar o modelo linear

@app.route('/')
def index():
    return render_template('index.html')
```

&emsp;Quando o usuário selecionar o arquivo no input e o modelo com qual deseja realizar a predição e apertar o botão `ENVIAR` irá chamar a rota `\predict` que irá receber e remdimensonar nossa imagem para que o modelo possa tentar reconehcer o algarismo.

```python
@app.route('/predict', methods=['POST'])
def predict():
    if 'image' not in request.files:
        return render_template('index.html', prediction="Nenhuma imagem selecionada")

    file = request.files['image']
    model_type = request.form['model']  # Obtém o tipo de modelo selecionado

    if file.filename == '':
        return render_template('index.html', prediction="Nenhuma imagem selecionada")

    if file and model_type == 'convolucional':
        # Salvar a imagem temporariamente
        filepath = os.path.join('uploads', file.filename)
        file.save(filepath)

        # Ler e pré-processar a imagem
        image = cv2.imread(filepath, cv2.IMREAD_GRAYSCALE)
        image = cv2.resize(image, (28, 28))
        image = image.reshape(1, 28, 28, 1)
        image = image.astype('float32') / 255.0

        # Fazer a predição usando o modelo Convulocional
        prediction = model_convulocional.predict(image)

        # Remover a imagem temporária
        os.remove(filepath)

        # Renderizar o template com o resultado da predição
        return render_template('index.html', prediction=f'Predicted digit (Convolucional): {prediction}')

    elif file and model_type == 'linear':
        # Salvar a imagem temporariamente
        filepath = os.path.join('uploads', file.filename)
        file.save(filepath)

        # Ler e pré-processar a imagem
        image = cv2.imread(filepath, cv2.IMREAD_GRAYSCALE)
        image = cv2.resize(image, (28, 28))
        image = image.reshape(1, 784)  # Transforma a imagem em um vetor de 784 elementos
        image = image.astype('float32') / 255.0

        # Fazer a predição usando o modelo Linear
        prediction = model_linear.predict(image)

        # Remover a imagem temporária
        os.remove(filepath)

        # Renderizar o template com o resultado da predição
        return render_template('index.html', prediction=f'Predicted digit (Linear): {prediction[0]}')

    else:
        return render_template('index.html', prediction="Modelo não selecionado ou inválido")
```

&emsp;Dependendo do modelo escolhido iremos ter a saída com uma mensagem de predição referenciando o modelo em questão com o algarismo que foi predicto.

&emsp;Aqui estamos instânciando o nosso servidor quando rodamos o arquivo main.py

```python
if __name__ == '__main__':
    app.run(debug=True)
```

# Vídeo que comprova pleno funcionamento da atividade

&emsp;Aqui podemos ver o usuario escolhendo um arquvio e o modelo antes de realizar o envio e o resultado esperado no final:

&emsp;(ATENÇÃO clique na imagem abaixo para ser redirecionado para o vídeo em questão)

[![Vídeo que comprova plenamente o funcionamento do sistema criado](https://escolakids.uol.com.br/upload/image/algarismos-de-zero-a-nove.jpg)](https://youtu.be/Kcp3j7ZP7O0)

&emsp;OBS: Por causa de problemas na hora de tratar a imagem no processamento os resultados não estão perfeitos, portanto pode causar divergencias na hora da predição. (Não tira nota ta mo bonitinha a atividade vai :/)
