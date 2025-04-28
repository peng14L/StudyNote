# 简介
sstream 是C++标准库中的一个组件，提供了一种方便的方式处理字符串流
它包含了三个类
1. istringstream 用于从字符串中读取数据
2. ostringstream 用于将数据写入字符串
3. stringstream 是istringstream和ostringstream的组合

## 示例
1. 从字符串中读取数据
```c++
#include <iostream>
#include <sstream>

int main() {
    std::string data = "10 20.5";
    std::istringstream iss(data);
    
    int i;
    double d;

    iss >> i >> d;

    std::cout << "Integer: " << i << std::endl;
    std::cout << "Double: " << std::endl;

    return 0;
}
```

2. 向字符串中写入数据
```c++
#include <iostream>
#include <sstream>

int main() {
    std::ostringstream oss;
    int i = 100;
    double d = 10.5;

    oss << i << " " << d;

    std::string result = oss.str();
    std::cout << "Resulting string: " << result << std::endl;

    return 0;
}

```