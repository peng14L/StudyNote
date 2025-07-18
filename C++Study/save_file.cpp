#include <iostream>
#include <fstream>
#include <string>
#include <cmath>

double calculateSquareSum(const int& m, const int& n) {

    double square_sum = std::pow(m, 2) + std::pow(n, 2);

    return square_sum;
}

int main() {
    double result = calculateSquareSum(3, 4);

    std::ofstream outFile("result.txt", std::ios::app);

    if (!outFile.is_open()) {
        std::cerr << "fail to open file" << std::endl;
        return 1;
    }

    outFile << "The square sum of 3 and 4 is: " << result << std::endl;

    outFile.close();

    return 0;
}