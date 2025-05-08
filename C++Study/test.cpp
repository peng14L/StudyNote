// #include <algorithm>
#include <iostream>
#include <map>
#include <vector>
#include <Eigen/Dense>
#include <thread>
#include <mutex>

// Eigen::Matrix4d eulerAngleToTransMatrix(const double& x, const double& y,
//     const double& z,
//     const double& roll,
//     const double& pitch,
//     const double& yaw) {
// Eigen::Translation3d translation(x, y, z);
// Eigen::AngleAxisd angleaxis_roll(roll, Eigen::Vector3d::UnitX());
// Eigen::AngleAxisd angleaxis_pitch(pitch, Eigen::Vector3d::UnitY());
// Eigen::AngleAxisd angleaxis_yaw(yaw, Eigen::Vector3d::UnitZ());
// return (translation * angleaxis_yaw * angleaxis_pitch * angleaxis_roll)
// .matrix();
// }
std::mutex mutex;
int c = 0;
void proc1(const int& i) {
    // mutex.lock();
    for (int s = 0; s < i; ++s) {
        std::cout << "This is proc1 " << c++ << std::endl;
    }
    // mutex.unlock();
}

void proc2() {
    // mutex.lock();
    for (int i = 0; i < 5; ++i) {
        c += 2;
        std::cout << "This is proc2 " << c << std::endl;
    }
    // mutex.unlock();
}

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

    // int a = 0;

    // // 指针常量, 指针是常量，即指针不能再指向其他变量, 创建时就要赋值；
    // int* const p = &a;
    // // *p = a;
    // std::cout << "p = " << p << std::endl;
    // std::cout << "*p = " << *p << std::endl;

    // int b = 1;
    // *p = b;
    // std::cout << "*p = " << *p << std::endl;


    // 常量指针，指向常量的指针，指针可以再指向其他变量，但是不能通过指针修改变量值
    // const int* q;
    // int c = 4;
    // q = &c;
    // std::cout << "q = " << q << std::endl;
    // std::cout << "*q = " << *q << std::endl;

    // std::map<double, int> map;
    // std::vector<double> vec{1, 2, 3, 4, 5, 1};

    // for (std::size_t i = 0; i < vec.size(); ++i) {
    //     double x = vec[i];
    //     while (map.find(x) != map.end()) {
    //         x += 0.01;
    //     }

    //     (void)map.emplace(x, i);
    // }

    // for (const auto& pair : map) {
    //     std::cout << pair.first << "-> " << pair.second << std::endl;
    // }
    // double x = 0, y = 0, z = 1;
    // Eigen::AngleAxisd angleaxis_roll(M_PI / 3, Eigen::Vector3d::UnitX());
    // Eigen::Translation3d translation(x, y, z);
    // auto matrix = (angleaxis_roll * translation).matrix();

    // std::cout << "Matrix: " << matrix << std::endl;

    std::thread t1(proc1, 10);
    std::thread t2(proc2);
    t1.join();
    t2.join();

    std::cout << "outcome c = " << c << std::endl;
    return 0;
}
