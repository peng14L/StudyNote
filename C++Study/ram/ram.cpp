#include <iostream>

// #pragma pack(1)
struct Test {
    int x;
    char y;
} test;
// #pragma pack()

int main() {
    int x;
    char y;

    std::cout << "size of int " << sizeof(x) << std::endl;
    std::cout << "size of char " << sizeof(y) << std::endl;
    std::cout << "size of struct " << sizeof(test) << std::endl;

    return 0;

}
