from flask import Flask, render_template, request
import os
import cv2
import numpy as np
from modelos.convulocional import Convulocional
from modelos.linear import LinearModel

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

# Inicializar os modelos Convulocional e Linear
model_convulocional = Convulocional(model_path)
model_linear = LinearModel((784,), 10)  # Ajuste para inicializar o modelo linear

@app.route('/')
def index():
    return render_template('index.html')

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

if __name__ == '__main__':
    app.run(debug=True)
