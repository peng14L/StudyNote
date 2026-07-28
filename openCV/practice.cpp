#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // 读取图像
    cv::Mat image = cv::imread("../example.png");

    if (image.empty()) {
        std::cout << "错误：无法加载图像，请检查路径是否正确。" << std::endl;
        return -1;
    }

    // 裁剪图像
    cv::Rect roi(100, 100, 200, 200); // (x, y, width, height)
    cv::Mat croppedImage = image(roi);

    // 缩放图像
    cv::Mat resizedImage;
    resize(image, resizedImage, cv::Size(400, 400)); // 缩放到400x400

    // 显示裁剪和缩放后的图像
    cv::imshow("Cropped Image", croppedImage);
    cv::imshow("Resized Image", resizedImage);
    cv::waitKey(5000);
    cv::destroyAllWindows();

    return 0;
}