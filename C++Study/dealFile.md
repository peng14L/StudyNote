# 关于文件内容的读取和写入
## 标准库 fstream
定义了三个新的数据类型
1. ofstream 用于创建文件并向文件写入信息
2. ifstream 用于从文件读取信息
3. fstream 同时具有ofstream、ifstream两种功能
C++中进行文件处理必须在源码文件中包含头文件<iostream>和<fstream>

## 打开文件
在从文件读取信息或者项文件写入信息之前必须先打开文件。open()是fstream、ifstream、ifstream对象的一个成员
```c++
void open(const char *filename, iso::openmode mode);
```
模式
1. ios::app 追加模式，所有写入都追加到文件末尾
2. ios::ate 文件打开后定位到文件末尾
3. ios::in 打开文件用于读取
4. ios::out 打开文件用于写入
5. ios::trunc 如果该文件已经存在，其内容将在打开文件之前被截断，即把文件长度设为0

### 示例
1. 读取并写入
```c++
#include <fstream>
#include <iostream>

int main() {
    std::fstream file;
    file.open("example.txt", std::ios::out); // 以输出模式打开文件

    if (!file) {
        std::cerr << "Unable to open file!" << std::endl;
        return 1; // 文件打开失败
    }

    file << "Hello, World!" << std::endl; // 写入文本
    file.close(); // 关闭文件

    return 0;
}
```

2. 读取
```c++
#include <fstream>
#include <iostream>
#include <stream>

int main() {
    std::fstream file;
    file.open("example.txt", std::ios::in);

    if (!file) {
        std::cerr << "Unable to open file!" << std::endl;
        return 1;
    }

    std::string line;
    while (getline(file, line)) {
        std::cout << line << std::endl;
    }

    file.close();

    return 0;
}
```

### yaml文件的读取
#### 细节处理
1. 判断字段是否存在
```c++
if (config["some_field"]) {
    std::cout << "field exists" << std::endl;
}
```
2. 判断字段内容是否为空
```c++
#include <yaml-cpp/yaml.h>
#include <string>
#include <iostream>

int main() {
    YAML::Node config = YAML::LoadFile("config.yaml");

    // 标量字段是否存在，字符串、数字、布尔值
    if (config["some_field"] && ！config["some_field"].as<std::string>().empty()) {
        std::cout << "Non-empty string field" << std::endl;
    }
}
```
