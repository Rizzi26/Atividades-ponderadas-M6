import cv2
import numpy as np
from flask import Flask, render_template
# from keras.datasets import mnist  # Corrigido aqui
# import matplotlib.pyplot as plt
# from keras.utils import to_categorical
# from keras.models import Sequential
# from keras.layers import Dense, Conv2D, MaxPool2D, Flatten, Dropout
# from keras.optimizers import Adam
# from tensorflow.keras.models import load_model

# # Carregar e preparar o conjunto de dados MNIST
# (x_train, y_train), (x_test, y_test) = mnist.load_data()
# x_train = x_train.reshape(x_train.shape[0], 28, 28, 1)
# x_test = x_test.reshape(x_test.shape[0], 28, 28, 1)
# x_train = x_train.astype('float32') / 255
# x_test = x_test.astype('float32') / 255
# y_train = to_categorical(y_train, 10)
# y_test = to_categorical(y_test, 10)

# # Definir o modelo
# model = Sequential([
#     Conv2D(32, kernel_size=(3, 3), activation='relu', input_shape=(28, 28, 1)),
#     MaxPool2D(pool_size=(2, 2)),
#     Conv2D(64, kernel_size=(3, 3), activation='relu'),
#     MaxPool2D(pool_size=(2, 2)),
#     Flatten(),
#     Dense(128, activation='relu'),
#     Dropout(0.5),
#     Dense(10, activation='softmax')
# ])

# # Compilar o modelo
# model.compile(optimizer=Adam(), loss='categorical_crossentropy', metrics=['accuracy'])

# # Treinar o modelo
# model.fit(x_train, y_train, validation_data=(x_test, y_test), epochs=1, batch_size=200)

# # Salvar o modelo
# model.save('mnist_cnn.h5')

app = Flask(__name__, template_folder='templates')

@app.route('/')
def index():
    return render_template('index.html')

if __name__ == '__main__':
    app.run()
