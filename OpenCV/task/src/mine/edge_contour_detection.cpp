#include<opencv2/opencv.hpp>
#include<iostream>
using namespace std;
using namespace cv;

class LightDescriptor
{
public:
    LightDescriptor() {};
    LightDescriptor(const cv::RotatedRect& light)
    {
        width = light.size.width;
        length = light.size.height;
        center = light.center;
        angle = light.angle;
        area = light.size.area();
    }
    const LightDescriptor& operator =(const LightDescriptor& ld)
    {
        this->width = ld.width;
        this->length = ld.length;
        this->center = ld.center;
        this->angle = ld.angle;
        this->area = ld.area;
        return *this;
    }
public:
    float width;
    float length;
    cv::Point2f center;
    float angle;
    float area;
};
 
int main(int argc, char** argv)
{
    VideoCapture capture("/home/emmm/Desktop/scnu_rm/OpenCV/task/img/task2_video.mp4"); // 替换为你的视频路径
    Mat frame;
    while (true)
    {
        capture.read(frame);
        if (frame.empty())
        {
            break;
        }
        
        // 1. 转换到HSV颜色空间，精准分割红色灯条
        Mat hsv;
        cvtColor(frame, hsv, COLOR_BGR2HSV);
        Mat redMask;
        // 红色在HSV的两个区间：0-10和160-180
        Scalar lower_red1 = Scalar(0, 120, 120);
        Scalar upper_red1 = Scalar(10, 255, 255);
        Scalar lower_red2 = Scalar(160, 120, 120);
        Scalar upper_red2 = Scalar(180, 255, 255);
        Mat mask1, mask2;
        inRange(hsv, lower_red1, upper_red1, mask1);
        inRange(hsv, lower_red2, upper_red2, mask2);
        redMask = mask1 | mask2;

        // 2. 预处理：高斯模糊 + 形态学操作增强灯条轮廓
        GaussianBlur(redMask, redMask, Size(3, 3), 1.5);
        Mat element = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
        dilate(redMask, redMask, element);
        morphologyEx(redMask, redMask, MORPH_CLOSE, element);

        // 3. Canny边缘检测（可选，若灯条边缘不清晰可启用）
        // Canny(redMask, redMask, 50, 150, 3);

        // 4. 轮廓检测
        vector<vector<Point>>contours;
        vector<Vec4i>hierachy;
        findContours(redMask, contours, hierachy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        vector<LightDescriptor> lightInfos;

        for (int i = 0; i < contours.size(); i++) {
            double area = contourArea(contours[i]);
            if (area < 20 || contours[i].size() <= 5) // 增大面积阈值，过滤更多噪声
                continue;

            // 椭圆拟合 + 旋转矩形
            RotatedRect Light_Rec = fitEllipse(contours[i]);

            // 新增：矩形度筛选（轮廓面积 / 外接矩形面积）
            double rectArea = Light_Rec.size.area();
            double rectangularity = area / rectArea;
            if (rectangularity < 0.6) // 矩形度低于0.6则过滤
                continue;

            // 原长宽比筛选（调整阈值）
            if (Light_Rec.size.width / Light_Rec.size.height > 3 || 
                Light_Rec.size.width / Light_Rec.size.height < 0.3) // 限制更合理的长宽比范围
                continue;

            // 扩大灯条区域（保持原逻辑）
            Light_Rec.size.height *= 1.1;
            Light_Rec.size.width *= 1.1;
            lightInfos.push_back(LightDescriptor(Light_Rec));
        }
       
        for (size_t i = 0; i < lightInfos.size(); i++) {
            for (size_t j = i + 1; j < lightInfos.size(); j++) {
                LightDescriptor& leftLight = lightInfos[i];
                LightDescriptor& rightLight = lightInfos[j];

                // 1. 角度差筛选（放宽到15度，适应实际场景）
                float angleDiff_ = abs(leftLight.angle - rightLight.angle);
                if (angleDiff_ > 15) 
                    continue;

                // 2. 长度差比率筛选（放宽到0.5，适应灯条长度差异）
                float LenDiff_ratio = abs(leftLight.length - rightLight.length) / max(leftLight.length, rightLight.length);
                if (LenDiff_ratio > 0.5) 
                    continue;

                // 3. 中心距离与长度比筛选（优化阈值）
                float dis = sqrt(pow(leftLight.center.x - rightLight.center.x, 2) + 
                                 pow(leftLight.center.y - rightLight.center.y, 2));
                float meanLen = (leftLight.length + rightLight.length) / 2;
                float ratio = dis / meanLen;
                if (ratio > 3 || ratio < 0.8) // 调整距离比范围
                    continue;

                // 4. Y方向差值筛选（减少垂直方向误匹配）
                float yDiff = abs(leftLight.center.y - rightLight.center.y);
                float yDiff_ratio = yDiff / meanLen;
                if (yDiff_ratio > 0.8) 
                    continue;

                // 绘制装甲板
                Point center = Point((leftLight.center.x + rightLight.center.x) / 2, 
                                    (leftLight.center.y + rightLight.center.y) / 2);
                RotatedRect rect = RotatedRect(center, Size(dis, meanLen), 
                                              (leftLight.angle + rightLight.angle) / 2);
                Point2f vertices[4];
                rect.points(vertices);
                for (int k = 0; k < 4; k++) {
                    line(frame, vertices[k], vertices[(k + 1) % 4], Scalar(0, 0, 255), 2);
                }
            }
        }
 
        imshow("装甲板识别（掩码）", redMask);
        imshow("装甲板识别（结果）", frame);
        int c = waitKey(30);
        if (c == 27)
            break;
    }
    waitKey(0);
    return 0;
}