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

# Dados de treinamento para a porta XOR
train_data = torch.tensor([[0, 0],
                           [0, 1],
                           [1, 0],
                           [1, 1]], dtype=torch.float32)

# Saídas esperadas para a porta XOR
target = torch.tensor([[0], [1], [1], [0]], dtype=torch.float32)

# Inicializar o MLP
mlp_torch = MLP_torch()

# função de perda e o otimizador
criterion = nn.MSELoss()
optimizer = optim.SGD(mlp_torch.parameters(), lr=0.2)

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

# Testar o MLP
print("Resultados após o treinamento com PyTorch:")
with torch.no_grad():
    for i in range(len(train_data)):
        output = mlp_torch(train_data[i])
        print(f"Entrada: {train_data[i].numpy()}, Saída esperada: {target[i].numpy()}, Saída do MLP: {output.item():.4f}")
