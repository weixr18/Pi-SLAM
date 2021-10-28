#include <iostream>
#include <opencv2/opencv.hpp>

int main()
{
    const char* filename = "../orb/left.jpg";
    cv::Mat pic = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
    cv::imshow("pic", pic);
    cv::waitKey();
    return 0;
}
