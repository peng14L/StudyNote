import torch
from torch.utils.data import Dataset, DataLoader

# Part One 
# 创建张量
# a = torch.tensor([1, 2, 3])
# b = torch.tensor([1.0, 2.0, 3.0])
# print(a.dtype) # torch.int64
# print(b.dtype) # torch.float32

# c = torch.tensor([1, 2, 3], dtype=torch.float32) # 指定张量数据类型

# arange = torch.arange(0, 10, 2)
# print(arange)
# linspace = torch.linspace(0, 1, 5)
# print(linspace)
# randn = torch.randn(2, 3) # 标准正态分布随机数
# print(randn)

# # 张量属性
# x = torch.randn(2, 3, 4)
# print(x.shape)
# print(x.dtype)
# print(x.device)
# print(x.requires_grad)
# print(x.ndim)
# print(x.numel())

# device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
# print(device)


# Part Two
# Dataset and DataLoader
# 自定义Dataset只需实现两个方法
# __len__() 和 __getitem__()
class MyDataset(Dataset):
    def __init__(self, n=100, dim=5):
        self.x = torch.randn(n, dim)
        self.y = torch.randint(0, 2, (n,))
    
    def __len__(self):
        return len(self.x)
    
    def __getitem__(self, idx):
        return self.x[idx], self.y[idx]

dataset = MyDataset()
loader = DataLoader(
    dataset,
    batch_size=16,
    shuffle=True,
    num_workers=0,
    drop_last=False,
)

for xb, yb in loader:
    print(xb.shape, yb.shape)
    break

# It's okay, we can study these day by day.

a = torch.randn(7, 2, 3, 5)
print(len(a))