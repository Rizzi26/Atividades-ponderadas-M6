import tensorflow as tf
import numpy as np

class Convulocional:
    def __init__(self, model_path='modelo_convulocional.h5'):
        self.model = tf.keras.models.load_model(model_path)
        self.model.make_predict_function()  # Corrigido para make_predict_function

    def predict(self, image):
        # Fazer a predição usando o modelo carregado
        predictions = self.model.predict(image)
        predicted_digit = np.argmax(predictions[0])
        print(f"Predição usando o modelo Convulocional: {predicted_digit}")
        return predicted_digit
        # Fazer a predição usando o modelo carregado