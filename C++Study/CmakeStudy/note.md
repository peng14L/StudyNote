# cmake基本介绍
1. ${CMAKE_CURRENT_SOURCE_DIR}
此变量通常存储CMakeListd.txt所在路径
2. ${CMAKE_CURRENT_BINARY_DIR}
表示当前处理的 CMakeLists.txt 文件对应的二进制目录（构建目录）的完整路径
在单配置生成器（如 Unix Makefiles）中，通常就是 build/ 目录
在多配置生成器（如 Visual Studio）中，可能包含子目录（如 build/Debug/ 或 build/Release/
