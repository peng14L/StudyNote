# map基本用法介绍
<map> 提供了一种关联容器，用于存储键值对，map容器中的元素是按照键的顺序自动排序的， 通常是升序。
每个键在map中只能出现一次
map提供了双向迭代器，可以向前和向后遍历元素

## 基本语法
```c++
#include <map>

std::map<key_type, value_type> myMap;

myMap[key] = value; // 插入元素

value = myMap[key] // 访问元素

// 遍历元素
for (std::map<key_type, value_type>::iterator it = myMap.begin(); it != myMap.end(); ++it) {
    std::cout << it->first << " => " << it->second << std::endl;
}

// 查找元素
iter = myMap.find(key) // find返回迭代器，指向当前查找元素的位置否则返回myMap.end()
```