#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <iostream>

class Tools 
{
public:
    // 绘制旋转矩形（框选装甲板）
    static void drawRotatedRect(cv::Mat& img, const cv::RotatedRect& rect, 
                               const cv::Scalar& color = cv::Scalar(0, 255, 0), int thickness = 5) 
    {
        cv::Point2f vertices[4];
        rect.points(vertices);
        for (int i = 0; i < 4; ++i) 
        {
            cv::line(img, vertices[i], vertices[(i + 1) % 4], color, thickness);
        }
    }

    // 绘制文本信息（支持指定起始位置和行间距）
    static void drawText(cv::Mat& img, const std::string& text, const cv::Point& pos, 
                        double scale = 0.5, const cv::Scalar& color = cv::Scalar(0, 255, 255), int thickness = 2) 
    {
        cv::putText(img, text, pos, cv::FONT_HERSHEY_SIMPLEX, scale, color, thickness);
    }
};

// 装甲板类，存储特征和位姿信息
class Armor 
{
public:
    cv::RotatedRect rect;            // 装甲板旋转矩形
    std::vector<cv::Point2f> points; // 装甲板四个顶点（图像坐标）
    cv::Mat rvec, tvec;              // 旋转向量、平移向量
    cv::Mat R;                       // 旋转矩阵
    double distance;                 // 到相机的距离（mm）
    cv::Vec3f eulerAngles;           // 欧拉角（滚转、俯仰、偏航，单位：度）

    // PnP解算位姿
    bool solvePnP(const std::vector<cv::Point3f>& objPoints, 
                 const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs) 
    {
        cv::solvePnP(objPoints, points, cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_EPNP);
        cv::Rodrigues(rvec, R);
        distance = cv::norm(tvec);

        // 旋转矩阵转欧拉角（滚转、俯仰、偏航）
        eulerAngles[0] = std::atan2(R.at<double>(2, 1), R.at<double>(2, 2)) * 180 / CV_PI;
        eulerAngles[1] = std::atan2(-R.at<double>(2, 0), std::sqrt(std::pow(R.at<double>(2, 1), 2) + std::pow(R.at<double>(2, 2), 2))) * 180 / CV_PI;
        eulerAngles[2] = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0)) * 180 / CV_PI;
        return true;
    }

    // 绘制位姿信息（固定在窗口左上角，参数：图像、起始坐标、行索引）
    //armor是const对象，只能调用被声明为const的成员函数
    void drawInfo(cv::Mat& img, const cv::Point& startPos, int lineIdx) const 
    {
        int lineSpacing = 25; // 行间距（像素）
        cv::Point pos = startPos + cv::Point(0, lineIdx * lineSpacing); // 计算当前行坐标

        // 绘制装甲板ID（如果有多个装甲板，区分显示）
        std::string idText = "Armor " + std::to_string(lineIdx) + ":";
        Tools::drawText(img, idText, pos);

        // 绘制距离（右移100像素，与ID错开）
        std::string distText = "Dist: " + std::to_string((int)distance) + "mm";
        Tools::drawText(img, distText, pos + cv::Point(100, 0));

        // 绘制欧拉角（再右移120像素）
        std::string anglesText = "R: " + std::to_string((int)eulerAngles[0]) 
                              + " P: " + std::to_string((int)eulerAngles[1])
                              + " Y: " + std::to_string((int)eulerAngles[2]);
        Tools::drawText(img, anglesText, pos + cv::Point(220, 0));
    }
};

// 装甲板检测器
class ArmorDetector 
{
public:
    // 相机内参（替换为实际标定结果）
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) <<
        1200, 0, 640,
        0, 1200, 360,
        0, 0, 1);

    // 畸变系数（替换为实际标定结果）
    cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << 0, 0, 0, 0, 0);

    // 装甲板3D坐标（单位：mm，根据实际尺寸调整）
    std::vector<cv::Point3f> armor3DPoints = {
        cv::Point3f(70, 25, 0),   // 右上角
        cv::Point3f(-70, 25, 0),  // 左上角
        cv::Point3f(-70, -25, 0), // 左下角
        cv::Point3f(70, -25, 0)   // 右下角
    };

    // 检测装甲板
    std::vector<Armor> detect(const cv::Mat& bgrImg) 
    {
        std::vector<Armor> armors;

        // 预处理
        cv::Mat grayImg, binaryImg;
        cv::cvtColor(bgrImg, grayImg, cv::COLOR_BGR2GRAY);
        cv::threshold(grayImg, binaryImg, 200, 255, cv::THRESH_BINARY);

        // 提取轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binaryImg, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 筛选装甲板
        for (const auto& contour : contours) 
        {
            double area = cv::contourArea(contour);
            if (area < 200) continue;

            cv::RotatedRect rect = cv::minAreaRect(contour);
            float ratio = std::max(rect.size.width, rect.size.height) / std::min(rect.size.width, rect.size.height);
            if (ratio < 2 || ratio > 8) continue;

            // 存储装甲板信息
            Armor armor;
            armor.rect = rect;
            cv::Point2f vertices[4];
            rect.points(vertices);
            armor.points = {vertices[0], vertices[1], vertices[2], vertices[3]};

            // 解算位姿
            armor.solvePnP(armor3DPoints, cameraMatrix, distCoeffs);
            armors.push_back(armor);
        }

        return armors;
    }
};

int main() 
{
    cv::VideoCapture cap("/home/emmm/Desktop/scnu_rm/OpenCV/task/img/task2_video.mp4");
    if (!cap.isOpened()) 
    {
        std::cout << "Error opening video stream or file" << std::endl;
        return -1;
    }

    ArmorDetector detector;
    cv::Point infoStartPos(10, 30); // 左上角起始坐标（x=10, y=30）

    while (true) 
    {
        cv::Mat bgrImg;
        cap >> bgrImg;
        if (bgrImg.empty()) break;

        // 检测装甲板
        std::vector<Armor> armors = detector.detect(bgrImg);

        // 绘制结果
        cv::Mat result = bgrImg.clone();
        
        // 1. 绘制所有装甲板框
        for (const auto& armor : armors) {
            Tools::drawRotatedRect(result, armor.rect);
        }

        // 2. 在左上角绘制位姿信息（按装甲板索引分行显示）
        for (size_t i = 0; i < armors.size(); ++i) {
            armors[i].drawInfo(result, infoStartPos, i); // i为行索引（0开始）
        }

        // 显示结果
        cv::imshow("Armor Detection with Pose", result);
        if (cv::waitKey(100) == 27) break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}