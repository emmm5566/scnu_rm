#include <opencv2/opencv.hpp>

int main()
{
    cv::VideoCapture cap(0);
    cv::Mat img;

    cv::Mat imgHSV, mask, imgColor;
    int HueMin = 0, SaturationMin = 0, ValueMin = 0;
    int HueMax = 179, SaturationMax = 255, ValueMax = 255;

    cv::namedWindow("Trackbars", (640, 200)); 
    cv::createTrackbar("Hue Min", "Trackbars", &HueMin, 179);
    cv::createTrackbar("Hue Max", "Trackbars", &HueMax, 179);
    cv::createTrackbar("Saturation Min", "Trackbars", &SaturationMin, 255);
    cv::createTrackbar("Saturation Max", "Trackbars", &SaturationMax, 255);
    cv::createTrackbar("Value Min", "Trackbars", &ValueMin, 255);
    cv::createTrackbar("Value Max", "Trackbars", &ValueMax, 255);

    while(true)
    {
        cap.read(img);

        cv::cvtColor(img, imgHSV, cv::COLOR_BGR2HSV);

        cv::Scalar lower(HueMin, SaturationMin, ValueMin); 
        cv::Scalar upper(HueMax, SaturationMax, ValueMax); 
        cv::inRange(imgHSV, lower, upper, mask); 
        cv::bitwise_and(img, img, imgColor, mask = mask);//位与运算函数，根据掩码mask从原图中提取符合条件的颜色区域

        std::cout << HueMin << "," << HueMax
            << "," << SaturationMin << "," << SaturationMax
            << "," << ValueMin << "," << ValueMax << std::endl;
        //HueMin,HueMax,SaturationMin,SaturationMax,ValueMin,ValueMax

        cv::imshow("Image", img);
        cv::imshow("Image HSV", imgHSV);
        cv::imshow("Image Mask", mask);//黑白二值图（白色=符合阈值，黑色=不符合）
        cv::imshow("Image Color", imgColor); //彩色图（保留符合阈值区域的原始颜色，其他区域黑色）


        cv::waitKey(1);
    }

    return 0;
}