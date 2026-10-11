import torch 
import numpy as np
from torch import nn
from torch.nn import functional as F

# a = torch.tensor([[1.0, 2.0], [3.0, 4.0]])
# b = torch.zeros(2, 2)
# c = torch.randn(2, 2)

# print(a.shape) # 输出：torch.Size([2, 2])
# print(a.dtype) # 输出：torch.float32
# print(a.device) # 输出：cpu

# print(a + b)
# print(a * 2)
# print(a @ c)

# x = torch.randn(3, 224, 224)
# x = x.unsqueeze(0)
# x = x.squeeze(0)
# y = x.view(3, -1)
# print(x.shape)
# print(y.shape) # 输出：torch.Size([3, 224 * 224])

# Autograd 自动求导
# w 是可训练的参数
# w = torch.tensor(3.0, requires_grad=True)
# x = torch.tensor(2.0)

# # 定义损失函数
# y = (w * x - 5) ** 2

# # 计算损失函数对 w 的梯度
# print(y.backward())
# print(w.grad) # dy/dw = 2 * (w * x - 5) * x = 2 * (3 * 2 - 5) * 2 = 2 * (6 - 5) * 2 = 2 * 1 * 2 = 4

# with torch.no_grad():
#     w -= 0.01 * w.grad
#     w.grad.zero_()
#     print(w)

class SmallNet(nn.Module):
    def __init__(self, in_dim, hidden, out_dim):
        super().__init__()  ## 必须先调用父类
        self.fc1 = nn.Linear(in_dim, hidden)
        self.relu = nn.ReLU()
        self.fc2 = nn.Linear(hidden, out_dim)

    def forward(self, x):
        x = self.fc1(x)
        x = self.relu(x)
        x = self.fc2(x)
        return x


model = SmallNet(in_dim=10, hidden=32, out_dim=1)
print(model)
dummy = torch.randn(4, 10)
out = model(dummy)
print(out.shape)