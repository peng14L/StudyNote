#include <algorithm>
#include <iostream>
#include <vector>

int main() {
    int src[] = {1, 2, 3, 4, 5};
    int dest[5];

    // 范围复制
    // std::copy(std::begin(src), std::end(src), std::begin(dest));
    std::cout << sizeof(dest) << std::endl;
    std::cout << sizeof(dest[0]) << std::endl;
    std::cout << sizeof(dest)/ sizeof(dest[0]) << std::endl;
    // 指定数量复制
    std::copy_n(src, 5, dest);
    for (int i = 0; i < 5; ++i) {
        std::cout << dest[i] << std::endl;
    }

    return 0;
}
