#include <yaml-cpp/yaml.h>
#include <string>
#include <iostream>

int main() {
    YAML::Node yaml = YAML::LoadFile("../config/default.yaml");
    std::string name = yaml["name"].as<std::string>();
    std::cout << name << std::endl;
    std::cout << true << std::endl;
    bool flag = false;
    if(!flag) {
      std::cout << "flag is true" << std::endl;
    }

    return 0;
}