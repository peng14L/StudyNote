# C++中的一些概念
## 常量指针和指针常量
1. 常量指针: 指针指向的内容是常量，不能通过该指针修改目标值，但指针本身可以指向其他地址 const int* p，指针可以变，指向的数据不能变
```c++
int a = 10, b = 20;
const int *ptr = &a;  // ptr是常量指针

// *ptr = 30;  // 错误！不能通过ptr修改a的值
a = 30;        // 正确，直接修改变量a是允许的

ptr = &b;      // 正确，指针本身可以指向其他地址
```
2. 指针常量: 指针本身是常量，初始化后不能指向其他地址，但可以通过指针修改目标值。int* const p 指针固定了，但指向的数据可以改
```c++
int a = 10, b = 20;
int* const ptr = &a;

*ptr = 30; // 正确，可以修改a的值
// ptr = &b; 错误。指针不能再指向其他地址
```
3. 常量指针常量: 都不能改，const int* const p, 指针不能变，数据也不能变

## std::move的用法和优势
std::move在头文件<utility>, 是一个类型转换函数，告诉编译器把该对象当成右值处理
std::move本身不会移动数据，只是做了一个类型转换，真正执行“移动”的，是后续调用的移动构造函数或移动赋值运算符
1. 基本写法
```c++
#include <utility>

std::string a = "hello";
std::string b = std::move(a); // 将a的资源 "搬到" b
```

## memcpy的用法介绍
是一个c风格的内存操作函数，用于将一段内存区域的内容复制到另一段内存区域。
1. 函数原型
```c++
void *memcpy(void *dest, const void *src, size_t n);
```
* dest: 目标内存地址
* src: 源内存地址
* n: 要复制的字节数
* 返回值: 返回目标内存地址dest

2. 基本用法
```c++
#include <iostream>
#include <cstring>

int main() {
    const char src[] = "Hello, memcpy";
    char dest[20];

    std::memcpy(dest, src, sizeof(src));

    std::cout << "Copied: " << dest << std::endl;

    int src = 42;
    int dest;
    std::memcpy(&dest, &src, sizeof(int));

    return 0;
}
```

## 现代内存操作方法
在现代C++(C++11/14/17/20及以后)中，内存操作的方式已经发生了显著变化，传统的C风格函数如memcpy虽然仍然可用
但更推荐使用以下更安全、更符合C++特性的方法
1. std::copy/std::copy_n
```c++
#include <algorithm>
#include <vector>

int src[] = {1, 2, 3, 4, 5};
int dest[5];

// 范围复制
std::copy(std::begin(src), std::end(src), std::begin(dest));

// 指定数量复制
std::copy_n(src, 3, dest);
```