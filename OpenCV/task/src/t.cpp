#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <algorithm>

using namespace cv;
using namespace std;

// 提取灯条（假设灯条为高亮物体，可根据实际情况调整阈值）
Mat extractLightBars(Mat& src) {
    cv::Mat channels[3];
    cv::split(src, channels);
    cv::Mat binary, Gaussian;
    cv::threshold(channels[2], binary, 200, 255, 0);
    cv::GaussianBlur(binary, Gaussian, cv::Size(5,5), 0);

    return Gaussian;
}

// 寻找灯条并返回最小外接矩形
vector<RotatedRect> findLightBarRects(Mat& binary) {
    vector<vector<Point>> contours;
    findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    
    vector<RotatedRect> lightBarRects;
    
    for (auto& contour : contours) {
        // 过滤面积过小的轮廓
        if (contourArea(contour) < 30) continue;
        
        // 计算最小外接矩形
        RotatedRect rect = minAreaRect(contour);
        
        // 过滤不符合灯条形状的矩形（长宽比筛选）
        float ratio = max(rect.size.width, rect.size.height) / 
                      min(rect.size.width, rect.size.height);
        if (ratio > 5 && ratio < 20) {  // 灯条通常是细长的
            lightBarRects.push_back(rect);
        }
    }
    
    // 确保只保留两个灯条（取面积最大的两个）
    if (lightBarRects.size() > 2) {
        sort(lightBarRects.begin(), lightBarRects.end(), 
             [](const RotatedRect& a, const RotatedRect& b) {
                 return a.size.area() > b.size.area();
             });
        lightBarRects.resize(2);
    }
    
    return lightBarRects;
}

// 计算包围两个灯条的大矩形
RotatedRect getEnclosingRect(const vector<RotatedRect>& lightBars) {
    if (lightBars.size() != 2) return RotatedRect();
    
    // 收集所有顶点
    vector<Point2f> allVertices;
    Point2f pts[4];
    lightBars[0].points(pts);
    for (int i = 0; i < 4; i++) allVertices.push_back(pts[i]);
    lightBars[1].points(pts);
    for (int i = 0; i < 4; i++) allVertices.push_back(pts[i]);
    
    float minX = allVertices[0].x, maxX = allVertices[0].x;
    float minY = allVertices[0].y, maxY = allVertices[0].y;
    for(auto & allVertice : allVertices)
    {
        minX = min(minX, allVertice.x);
        maxX = max(maxX, allVertice.x);
        minY = min(minY, allVertice.y);
        maxY = max(maxY, allVertice.y);
    }
    vector<Point2f> rectVertices; 
    rectVertices.push_back(Point2f(minX, minY));
    rectVertices.push_back(Point2f(maxX, minY));
    rectVertices.push_back(Point2f(minX, maxY));
    rectVertices.push_back(Point2f(maxX, maxY));   

    // 计算最小外接矩形
    return minAreaRect(rectVertices);
}

// 旋转向量转欧拉角 (Z-Y-X顺序，单位：度)
vector<double> rvec2euler(Mat& rvec) {
    Mat R;
    Rodrigues(rvec, R);  // 旋转向量转旋转矩阵
    
    double sy = sqrt(R.at<double>(0,0)*R.at<double>(0,0) + R.at<double>(1,0)*R.at<double>(1,0));
    bool singular = sy < 1e-6;
    
    double x, y, z;
    if (!singular) {
        x = atan2(R.at<double>(2,1), R.at<double>(2,2)) * 180 / CV_PI;
        y = atan2(-R.at<double>(2,0), sy) * 180 / CV_PI;
        z = atan2(R.at<double>(1,0), R.at<double>(0,0)) * 180 / CV_PI;
    } else {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1)) * 180 / CV_PI;
        y = atan2(-R.at<double>(2,0), sy) * 180 / CV_PI;
        z = 0;
    }
    
    return {x, y, z};  // 滚转角, 俯仰角, 偏航角
}

int main() {
    // 读取图像（替换为你的图像路径）
    Mat frame = imread("../img/pnp.jpg");
    if (frame.empty()) {
        cerr << "无法读取图像!" << endl;
        return -1;
    }
    
    // 1. 提取灯条
    Mat binary = extractLightBars(frame);
    imshow("binary", binary);
    
    // 2. 寻找灯条最小外接矩形
    vector<RotatedRect> lightBars = findLightBarRects(binary);
    if (lightBars.size() != 2) {
        cerr << "未检测到两个灯条!" << endl;
        return -1;
    }
    
    // 绘制灯条最小外接矩形（绿色）
    for (auto& rect : lightBars) {
        Point2f pts[4];
        rect.points(pts);
        for (int i = 0; i < 4; i++) {
            line(frame, pts[i], pts[(i+1)%4], Scalar(0, 255, 0), 2);
        }
    }
    
    // 3. 计算并绘制包围大矩形（红色）
    RotatedRect enclosingRect = getEnclosingRect(lightBars);
    Point2f enclosingPts[4];
    enclosingRect.points(enclosingPts);
    for (int i = 0; i < 4; i++) {
        line(frame, enclosingPts[i], enclosingPts[(i+1)%4], Scalar(0, 0, 255), 2);
    }
    
    // 4. PnP解算准备
    // 3D世界坐标（假设大矩形在z=0平面，单位：mm，根据实际尺寸调整）
    vector<Point3f> objectPoints;
    float width = enclosingRect.size.width * 0.5f;  // 假设像素与实际尺寸的比例
    float height = enclosingRect.size.height * 0.5f;
    
    // 注意：3D点顺序需与2D点顺序对应
    objectPoints.emplace_back(-width, -height, 0);  // 左上
    objectPoints.emplace_back(width, -height, 0);   // 右上
    objectPoints.emplace_back(width, height, 0);    // 右下
    objectPoints.emplace_back(-width, height, 0);   // 左下
    
    // 图像2D坐标
    vector<Point2f> imagePoints;
    for (int i = 0; i < 4; i++) {
        imagePoints.push_back(enclosingPts[i]);
    }
    
    // 相机内参（需替换为实际标定结果）
    Mat cameraMatrix = (Mat_<double>(3, 3) << 
        800, 0, 320,   // fx, 0, cx
        0, 800, 240,   // 0, fy, cy
        0, 0, 1);      // 0, 0, 1
    
    Mat distCoeffs = Mat::zeros(5, 1, CV_64F);  // 畸变系数，无畸变时为0
    
    // 5. 解算PnP
    Mat rvec, tvec;
    solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
    
    // 6. 计算欧拉角
    vector<double> euler = rvec2euler(rvec);
    
    // 输出结果
    cout << "旋转向量 (rvec): \n" << rvec << endl << endl;
    cout << "平移向量 (tvec): \n" << tvec << endl << endl;
    cout << "欧拉角 (滚转, 俯仰, 偏航) 度: \n";
    cout << "roll: " << euler[0] << ", pitch: " << euler[1] << ", yaw: " << euler[2] << endl;
    
    // 显示结果
    imshow("灯条检测与PnP解算", frame);
    waitKey(0);
    
    return 0;
}