# 一个简单的例子
```c++
#include <ros/ros.h>
#include <signal.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "my_node");
    signal(SIGINT, sigHandler);
    signal(SIGTERM, sigHandler);

    while (ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}
```