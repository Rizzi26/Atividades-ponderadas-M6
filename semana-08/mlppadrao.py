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

    def forward(self, batch):
        self.hidden_ = np.dot(batch, self.weights_01) + self.b01
        self.hidden_out = self._sigmoid(self.hidden_)

        self.output_ = np.dot(self.hidden_out, self.weights_12) + self.b12
        self.output_final = self._sigmoid(self.output_)

        return self.output_final

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

    def train(self):
        for _ in range(self.num_epochs):
            self.forward(self.train_data)
            self.update_weights()

    def predict(self, data):
        return self.forward(data)

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