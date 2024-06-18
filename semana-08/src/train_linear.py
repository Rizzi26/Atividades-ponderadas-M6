import numpy as np
import pandas as pd
from sklearn.model_selection import train_test_split
import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Flatten, Dense
from tensorflow.keras.optimizers import Adam

# Carregar os dados do MNIST
df_train = pd.read_csv('mnist_train.csv')
df_test = pd.read_csv('mnist_test.csv')

# Converter para float32 e normalizar os dados
train_data = np.array(df_train, dtype='float32')
test_data = np.array(df_test, dtype='float32')

x_train = train_data[:, 1:] / 255.0
y_train = train_data[:, 0]

x_test = test_data[:, 1:] / 255.0
y_test = test_data[:, 0]

# Dividir em conjunto de treino e validação
x_train, x_validate, y_train, y_validate = train_test_split(x_train, y_train, test_size=0.2, random_state=42)

# Definir o modelo linear
linear_model = Sequential([
    Flatten(input_shape=(784,)),
    Dense(128, activation='relu'),  
    Dense(10, activation='softmax') 
])

# Compilar o modelo
linear_model.compile(
    loss='sparse_categorical_crossentropy', 
    optimizer=Adam(learning_rate=0.001),  
    metrics=['accuracy']
)

# Treinar o modelo
history = linear_model.fit(
    x_train,
    y_train,
    batch_size=128,
    epochs=15,
    verbose=1,
    validation_data=(x_validate, y_validate)
)

# Avaliar o modelo no conjunto de teste
test_loss, test_accuracy = linear_model.evaluate(x_test, y_test, verbose=1)
print(f'Acurácia no conjunto de teste: {test_accuracy}')

# Salvar o modelo treinado
linear_model.save('modelo_linear.h5')
print("Modelo linear treinado e salvo com sucesso!")
