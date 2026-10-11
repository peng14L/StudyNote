import torch

# Part One
# x = torch.tensor([2.0], requires_grad=True)
# y = x ** 2 + 3 * x 
# y.backward()
# print(x.grad)
# print(x.grad_fn)
# print(y.grad_fn)
# print(x.is_leaf)
# print(y.is_leaf)

# Part Two
x = torch.tensor([1.0, 2.0, 3.0], requires_grad=True)
y = x ** 2
y.backward(torch.tensor([1.0, 1.0, 1.0]))
# print(x.grad)

a = torch.tensor(2.0, requires_grad=True)
for _ in range(3):
    b = a ** 2
    b.backward()
    print(a.grad)
    # print(b.grad)

for _ in range(3):
    if a.grad is not None:
        a.grad.zero_()
    b = a ** 2
    b.backward()
    print(a.grad)