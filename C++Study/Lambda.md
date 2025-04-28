# 基本语法
[capture](parameters) -> return_type { //函数体 }

1. capture: 捕获列表，定义从外部作用于捕获哪些变量以及如何捕获
2. parameters: 与普通函数的参数列表类似
3. return_type: 使用尾位置返回类型语法
4. 函数体： 包含Lambda表达式的代码

# 捕获列表详解
捕获列表决定了Lambda表达式如何访问外部变量
1. 值捕获
```c++
int a = 1;
auto lambda = [a] { return a + 1; };
```

2. 引用捕获
```c++
int a = 1;
auto lambda = [&a] { a += 1; return a; };
```

3. 隐式捕获
```c++
int a = 1, int b = 2;
auto lambda1 = [=] { return a + b; }; // 值捕获所有外部变量
auto lambda2 = [&] { a++; b++; }; // 引用捕获所有外部变量
```
4. 混合捕获
```c++
int a = 1, b = 2, c = 3;
auto lambda = [=, &b, &c] { 
    // a值捕获，b和c引用捕获
    return a + b + c;
};
```

5. 捕获this指针(类成员函数中使用)
```c++
class MyClass {
    int value = 0;

public:
    void foo() {
        auto lambda = [this] { return value;  };
    }
};

6. mutable 关键字
默认情况下，值捕获的变量在Lambda内是const的，使用mutable可以修改
```c++
int a = 1;
auto lambda = [a]() mutable {
    a += 1;
    return a;
}