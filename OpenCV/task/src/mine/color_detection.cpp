/*************************颜色分割法**************************** */

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

using namespace cv;
using namespace std;

Mat processFrame(Mat frame) {
    // 1. 转换到HSV颜色空间
    Mat hsv;
    cvtColor(frame, hsv, COLOR_BGR2HSV);

    // 2. 定义红色HSV阈值（两个区间）
    Scalar lower_red1 = Scalar(0, 120, 120);
    Scalar upper_red1 = Scalar(10, 255, 255);
    Scalar lower_red2 = Scalar(160, 120, 120);
    Scalar upper_red2 = Scalar(180, 255, 255);

    Mat mask1, mask2;
    inRange(hsv, lower_red1, upper_red1, mask1);
    inRange(hsv, lower_red2, upper_red2, mask2);
    Mat mask = mask1 | mask2;

    // 3. 形态学操作
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(mask, mask, MORPH_OPEN, kernel);
    morphologyEx(mask, mask, MORPH_CLOSE, kernel);

    // 4. 轮廓检测
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // 5. 筛选灯条轮廓
    vector<RotatedRect> lightBars;
    // 同时保存每个灯条对应的轮廓（用于计算外接圆）
    vector<vector<Point>> validContours;
    
    for (auto &contour : contours) {
        if (contourArea(contour) < 50) continue;

        RotatedRect rect = minAreaRect(contour);
        float aspectRatio = max(rect.size.width, rect.size.height) / 
                            min(rect.size.width, rect.size.height);
        if (aspectRatio > 3) {
            lightBars.push_back(rect);
            validContours.push_back(contour); // 保存有效的轮廓
        }
    }

    // 6. 匹配装甲板的两个灯条
    vector<vector<RotatedRect>> armorPlates;
    for (int i = 0; i < lightBars.size(); i++) {
        for (int j = i + 1; j < lightBars.size(); j++) {
            RotatedRect &r1 = lightBars[i];
            RotatedRect &r2 = lightBars[j];

            // 角度差筛选
            float angleDiff = abs(r1.angle - r2.angle);
            if (angleDiff > 10) continue;

            // 修正：使用轮廓计算最小外接圆（而不是用center点）
            Point2f center1, center2;
            float radius1, radius2;
            // 第一个参数传入轮廓点集（validContours[i]）
            minEnclosingCircle(validContours[i], center1, radius1);
            minEnclosingCircle(validContours[j], center2, radius2);

            // 中心距离计算
            float distance = norm(center1 - center2);
            float length1 = max(r1.size.width, r1.size.height);
            float length2 = max(r2.size.width, r2.size.height);
            float lengthRatio = abs(length1 - length2) / max(length1, length2);
            if (lengthRatio > 0.3) continue;

            float distanceRatio = distance / max(length1, length2);
            if (distanceRatio > 3 || distanceRatio < 0.5) continue;

            armorPlates.push_back({r1, r2});
        }
    }

    // 7. 绘制识别结果
    Mat result = frame.clone();
    for (auto &plate : armorPlates) {
        for (auto &bar : plate) {
            Point2f vertices[4];
            bar.points(vertices);
            for (int k = 0; k < 4; k++) {
                line(result, vertices[k], vertices[(k + 1) % 4], Scalar(0, 255, 0), 2);
            }
        }
    }

    return result;
}

int main() {
    VideoCapture cap("/home/emmm/Desktop/scnu_rm/OpenCV/task/img/task2_video.mp4"); // 替换为你的视频路径
    if (!cap.isOpened()) {
        cout << "无法打开视频！" << endl;
        return -1;
    }

    Mat frame, result;
    while (cap.read(frame)) {
        result = processFrame(frame);
        imshow("装甲板灯条识别", result);
        if (waitKey(30) == 27) break; // ESC键退出
    }

    cap.release();
    destroyAllWindows();
    return 0;
}
