* Eigen::Translation3d
是Eigen::Translation<double, 3>的别名，用于表示一个三维空间的平移变换

```c++
Eigen::Translation translation{0, 0, 1};
```
translation.vector()返回的是Eigen::Vector3d

* Eigen::AngleAxisd 
用于表示三维空间中绕任意轴旋转的轴角表示的一种类型
其中d代表double类型，也可以用f表示float
```c++
#include <Eigen/Geometry>
#include <iostream>

int main() {
    // 绕 Z 轴旋转 90 度
    double angle = M_PI / 2;
    Eigen::Vector3d axis(0, 0, 1);
    Eigen::AngleAxisd rotation(angle, axis);

    std::cout << "Angle (rad): " << rotation.angle() << std::endl;
    std::cout << "Axis: " << rotation.axis().transpose() << std::endl;

    // 使用 AngleAxis 构建旋转矩阵
    Eigen::Matrix3d rotation_matrix = rotation.toRotationMatrix();
    std::cout << "Rotation matrix:\n" << rotation_matrix << std::endl;

    return 0;
}

```