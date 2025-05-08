# <thread>的基本用法

## 创建线程
```c++
#include <thread>

std::thread thread_object(callable, args...);
```
* callable: 可调用对象，可以是函数指针、函数对象、Lambda表达式等
* args...: 传递给callable的参数列表

## 使用函数指针
```c++
#include <iostream>
#include <thread>

void printMessage(int count) {
    for (int i = 0; i < count; ++i) {
        std::cout << "Hello form thread!\n";
    }
}

int main() {
    std::thread t1(printMessage, 5);
    t1.join(); // 等待线程完成
    return 0;
}
```
* join() 用于等待线程完成执行
* detach()将线程与主线程分离，线程在后台独立运行，主线程不再等待它

## 线程的传参
```c++
#include <iostream>
#include <thread>

void increment(int& x) {
    ++x;
}

int main() {
    int num = 0;
    std::thread t(increament, std::ref(num));
    t.join();
    std::cout << "Value after increment: " << num << std::endl;
    return 0;
}
```