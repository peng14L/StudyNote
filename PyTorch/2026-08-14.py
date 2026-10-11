import torch
import argparse

# 今日不学新内容，只做本周学习内容的总结
# 1. 本周学习pytorch的基础tensor用法
x = torch.randint(0, 2, (4,))
# y = torch.randint(0, 2, 4)
print(x.requires_grad)
parser = argparse.ArgumentParser(description="pytorch train")
parser.add_argument("--lr", type=float, default=0.001, help="learning rate")
parser.add_argument("--batch_size", type=int, default=32, help="batch size")
args = parser.parse_args()

print(f"learning rate is {args.learning_rate}")
