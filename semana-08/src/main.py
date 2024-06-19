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

# Verifique se o modelo já existe
model_path_conv = 'modelo_convulocional.h5'
model_path_linear = 'modelo_linear.h5'

if not os.path.exists(model_path_conv):
    print("Modelo Convolucional não encontrado. Treinando o modelo...")
    os.system('python3 train_convulocional.py')  # Comando para treinar o modelo
else:
    print(f"Carregando modelo Convolucional existente de {model_path_conv}")

if not os.path.exists(model_path_linear):
    print("Modelo Linear não encontrado. Treinando o modelo...")
    os.system('python3 train_linear.py')  # Comando para treinar o modelo
else:
    print(f"Carregando modelo Linear existente de {model_path_linear}")

# Inicializar os modelos Convulocional e Linear
model_convulocional = Convulocional(model_path_conv)
model_linear = LinearModel((28, 28, 1), 10)  # Ajustar com o input_shape correto e num_classes

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/predict', methods=['POST'])
def predict():
    if 'image' not in request.files:
        return render_template('index.html', prediction="Nenhuma imagem selecionada")

    file = request.files['image']

    if file.filename == '':
        return render_template('index.html', prediction="Nenhuma imagem selecionada")

    chosen_model = request.form.get('model')  # Obter o modelo escolhido pelo usuário

    if chosen_model == 'linear':
        model_to_use = model_linear  # Usar o modelo Linear
    else:
        model_to_use = model_convulocional  # Usar o modelo Convolucional

    if file:
        # Salvar a imagem temporariamente
        filepath = os.path.join('uploads', file.filename)
        file.save(filepath)

        # Ler e pré-processar a imagem
        image = cv2.imread(filepath, cv2.IMREAD_GRAYSCALE)
        image = cv2.resize(image, (28, 28))
        image = image.reshape(1, 28, 28, 1)  
        image = image.astype('float32') / 255.0

        # Fazer a predição usando o modelo selecionado
        prediction = model_to_use.predict(image)

        # Remover a imagem temporária
        os.remove(filepath)

        # Renderizar o template com o resultado da predição
        if chosen_model == 'linear':
            return render_template('index.html', prediction=f'Predicted digit (Linear): {prediction}')
        else:
            return render_template('index.html', prediction=f'Predicted digit (Convolucional): {prediction}')

    return render_template('index.html')

if __name__ == '__main__':
    app.run(debug=True)
