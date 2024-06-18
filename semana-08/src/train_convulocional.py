import tensorflow as tf
import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Conv2D, MaxPooling2D, Dropout, Flatten, Dense
from tensorflow.keras.optimizers import Adam

# Carregar os dados
df_train = pd.read_csv('mnist_train.csv')
df_test = pd.read_csv('mnist_test.csv')

# Converter para float32 e normalizar
train_data = np.array(df_train, dtype='float32')
test_data = np.array(df_test, dtype='float32')

x_train = train_data[:, 1:] / 255.0
y_train = train_data[:, 0]

x_test = test_data[:, 1:] / 255.0
y_test = test_data[:, 0]

# Dividir em conjunto de treino e validação
x_train, x_validate, y_train, y_validate = train_test_split(x_train, y_train, test_size=0.2, random_state=42)

# Reshape para o formato adequado
image_rows = 28
image_cols = 28
image_shape = (image_rows, image_cols, 1)

x_train = x_train.reshape(x_train.shape[0], *image_shape)
x_test = x_test.reshape(x_test.shape[0], *image_shape)
x_validate = x_validate.reshape(x_validate.shape[0], *image_shape)

# Definir o modelo CNN
cnn_model = Sequential([
    Conv2D(filters=32, kernel_size=3, activation='relu', input_shape=image_shape),
    MaxPooling2D(pool_size=2),
    Dropout(0.2),
    Flatten(),
    Dense(32, activation='relu'),
    Dense(10, activation='softmax')
])

# Compilar o modelo
cnn_model.compile(
    loss='sparse_categorical_crossentropy',
    optimizer=Adam(learning_rate=0.03),
    metrics=['accuracy']
)

# Treinar o modelo
history = cnn_model.fit(
    x_train,
    y_train,
    batch_size=4096,
    epochs=35,
    verbose=1,
    validation_data=(x_validate, y_validate)
)

# Salvar o modelo treinado
cnn_model.save('modelo_convulocional.h5')
print("Modelo treinado e salvo em 'modelo_convulocional.h5'")
