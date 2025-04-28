# Study the std::function

## Definition
std::function 是C++11引入的通用函数包装器，属于<functional>头文件，用于存储、复制和调用任何可调用对象(函数、Lambda、仿函数等)。提供了一种类型安全的方式统一处理不同类型的回调。

## 用法
```c++
#include <functional>
std::function<返回值类型(参数类型列表)> 对象名；
```

## 例子
### 1. 普通函数
```c++
#include <functional>

int add(int a, int b) { return a + b; }
std::function<int(int, int)> f = add;
std::cout << f(2, 3);
```
### 2. Lambda表达式
```c++
std::function<void()> greet = []() { std::cout << "Hello!" ; };
greet();
```

### 3. 类成员函数
```c++
class Math {
public:
  int multiply(int x, int y) { return x * y; }
};

Math math;
std::function<int(Math&, int, int)> f = &Math::multiply;
std::cout << f(math, 4, 5);
```

