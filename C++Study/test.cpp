#include <algorithm>
#include <iostream>
#include <vector>

int main() {
    // int src[] = {1, 2, 3, 4, 5};
    // int dest[5];

    // // 范围复制
    // // std::copy(std::begin(src), std::end(src), std::begin(dest));
    // std::cout << sizeof(dest) << std::endl;
    // std::cout << sizeof(dest[0]) << std::endl;
    // std::cout << sizeof(dest)/ sizeof(dest[0]) << std::endl;
    // // 指定数量复制
    // std::copy_n(src, 5, dest);
    // for (int i = 0; i < 5; ++i) {
    //     std::cout << dest[i] << std::endl;
    // }

    int a = 0;

    // 指针常量, 指针是常量，即指针不能再指向其他变量, 创建时就要赋值；
    int* const p = &a;
    // *p = a;
    std::cout << "p = " << p << std::endl;
    std::cout << "*p = " << *p << std::endl;

    int b = 1;
    *p = b;
    std::cout << "*p = " << *p << std::endl;


    // 常量指针，指向常量的指针，指针可以再指向其他变量，但是不能通过指针修改变量值
    const int* q;
    int c = 4;
    q = &c;
    std::cout << "q = " << q << std::endl;
    std::cout << "*q = " << *q << std::endl;

    return 0;
}
