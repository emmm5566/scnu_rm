#include <opencv2/opencv.hpp>

int main()
{
    std::string path = "/home/emmm/Desktop/scnu_rm/OpenCV/course/img/person.png";
    cv::Mat img = cv::imread(path);
    resize(img, img, cv::Size(500, 500));

    cv::Mat imgGray, imgBlur, imgCanny, imgDil, imgErode;
    //图像灰度化
    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);
    //高斯模糊
    cv::GaussianBlur(imgGray, imgBlur, cv::Size(7,7), 5, 0); //数值越小模糊度越低
    //边缘检测
    cv::Canny(imgBlur, imgCanny, 50, 150); //阈值越小检测的边缘越多

    //膨胀
    cv::Mat kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(5,5)); //设置核，size越大膨胀越多,只能用奇数
    cv::dilate(imgCanny, imgDil, kernel);
    //腐蚀
    cv::erode(imgDil, imgErode, kernel);

    cv::imshow("Image", img);
    cv::imshow("Image Gray", imgGray);
    cv::imshow("Image Blur", imgBlur);
    cv::imshow("Image Canny", imgCanny);
    cv::imshow("Image Dilation", imgDil);
    cv::imshow("Image Erode", imgErode);

    cv::waitKey(0);

    return 0;
}