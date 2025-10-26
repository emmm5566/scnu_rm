#include <opencv2/opencv.hpp>

int main()
{
    std::string path = "/home/emmm/Desktop/scnu_rm/OpenCV/course/img/cards.png";
    cv::Mat img = cv::imread(path);
    
    //截取点像素坐标
    float weight = 250, height = 350;
    cv::Point2f src[4] = {{714,726}, {1017,798}, {567,1053}, {891,1140}};
    cv::Point2f dst[4] = {{0.0f,0.0f}, {weight,0.0f}, {0.0f,height}, {weight,height}};

    //透视变换（Perspective Transformation）
    cv::Mat mat, imgWarp;
    mat = cv::getPerspectiveTransform(src, dst); //计算透视变换矩阵：根据源图像和目标图像中的对应点，计算透视变换矩阵mat
    cv::warpPerspective(img, imgWarp, mat, cv::Point(weight, height)); // 应用透视变换：根据透视变换矩阵mat，将原图像img透视变换输出imgWarp
    
    //在四个坐标点上画小圆圈
    for(int i=0; i<4; i++)
    {
        circle(img, src[i], 10, cv::Scalar(0, 0, 255), cv::FILLED);
    }

    cv::imshow("Image", img);
    cv::imshow("Image Warp", imgWarp);

    cv::waitKey(0);

    return 0;
}