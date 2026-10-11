import torch
from torch.utils.data import TensorDataset, DataLoader
import torch.nn as nn

# Part One
# TensorDataset 是一个 PyTorch 中的数据集类，用于将多个张量打包成一个数据集。
# X = torch.randn(200, 5)
# Y = torch.randint(0, 2, (200,))

# dataset = TensorDataset(X, Y)
# loader = DataLoader(dataset, batch_size=10, shuffle=True)

# x, y = next(iter(loader))
# print(x.shape, y.shape)

# Part Two
# Optimizer
# opt_sgd = torch.optim.SGD(
#     model.parameters(),
#     lr=0.01,
#     momentum=0.9,
#     weight_decay=0.0001,
# )

# opt_adam = torch.optim.Adam(
#     model.parameters(),
#     lr=1e-3,
#     weight_decay=1e-4
# )

# 标准训练循环 5 步
# 1. 前向传播
# 2. 计算损失
# 3. 清空梯度
# 4. 反向传播
# 5. 更新参数

model = nn.Linear(5, 2)
criterion = nn.CrossEntropyLoss()
optimizer = torch.optim.Adam(model.parameters(), lr=1e-3, weight_decay=1e-4)

x = torch.randn(16, 5)
y = torch.randint(0, 2, (16,))

pred = model(x)
loss = criterion(pred, y)
optimizer.zero_grad()
loss.backward()
optimizer.step()

print(loss.item())