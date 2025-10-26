#include <opencv2/opencv.hpp>

int main()
{
    std::string path = "/home/emmm/Desktop/scnu_rm/OpenCV/course/img/person.png";
    cv::Mat img = cv::imread(path);
    
    cv::Mat imgResize, imgCrop;
    //调整大小
    std::cout << img.size() << std::endl; //查看图像大小
    resize(img, imgResize, cv::Size(), 0.5, 0.5);

    //裁减图像
    cv::Rect roi(800, 700, 900, 1000); //相对于(0,0,0,0)移动量
    imgCrop = img(roi);
    
    cv::imshow("Image", img);
    cv::imshow("Image Resize", imgResize);
    cv::imshow("Image Crop", imgCrop);

    cv::waitKey(0);

    return 0;
}