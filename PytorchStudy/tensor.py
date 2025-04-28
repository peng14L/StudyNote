import torch
import numpy as np

x = torch.tensor([[1, 2], [3, 4]])
x_np = np.array(x) # tensor to np.array
x_tensor = torch.from_numpy(x_np) # np.array to tensor