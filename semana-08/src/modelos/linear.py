import tensorflow as tf
import numpy as np

class LinearModel:
    def __init__(self, input_shape, num_classes):
        self.model = tf.keras.Sequential([
            tf.keras.layers.Flatten(input_shape=input_shape),
            tf.keras.layers.Dense(num_classes, activation='softmax')
        ])
        self.model.compile(optimizer='adam',
                           loss='sparse_categorical_crossentropy',
                           metrics=['accuracy'])

    def train(self, x_train, y_train, epochs=10, batch_size=32):
        self.model.fit(x_train, y_train, epochs=epochs, batch_size=batch_size)

    def predict(self, x):
        predictions = self.model.predict(x)
        predicted_digit = np.argmax(predictions)

        return predicted_digit
