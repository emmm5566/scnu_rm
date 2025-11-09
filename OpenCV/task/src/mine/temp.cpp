#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <climits>  // 用于INT_MAX

using namespace std;
using namespace cv;

// 常量定义：可根据实际场景调整
const int RED_THRESHOLD = 160;        // 红色通道二值化阈值
const int BINARY_MAX_VAL = 255;       // 二值化最大值
const Size GAUSSIAN_KERNEL = Size(5, 5);  // 高斯模糊核大小
const Size MORPH_KERNEL = Size(3, 3);     // 形态学操作核大小
const float ASPECT_RATIO_MIN = 1.5f;  // 灯条最小高宽比
const int MIN_HEIGHT = 30;            // 灯条最小高度
const int MIN_WIDTH = 15;             // 灯条最小宽度
const string VIDEO_PATH = "/home/emmm/Desktop/scnu_rm/OpenCV/task/img/task2_video.mp4";  // 视频路径

// 自定义结构体：表示一对匹配的灯条
struct LightBarPair {
    Rect left;    // 左侧灯条
    Rect right;   // 右侧灯条
    bool isValid; // 标记是否为有效匹配

    // 构造函数
    LightBarPair() : isValid(false) {}
    LightBarPair(const Rect& l, const Rect& r) : left(l), right(r), isValid(true) {}
};

/**
 * @brief 预处理函数：提取红色通道并进行二值化、滤波等操作
 * @param frame 原始视频帧
 * @param output 处理后的二值图像
 */
void preprocessFrame(const Mat& frame, Mat& output) {
    if (frame.empty()) {
        output = Mat();
        return;
    }

    // 1. 分离颜色通道，提取红色通道
    vector<Mat> channels;
    split(frame, channels);
    if (channels.size() < 3) {  // 确保通道分离成功
        output = Mat();
        return;
    }
    Mat redChannel = channels[2];  // BGR格式中索引2为红色通道

    // 2. 二值化：保留高亮度红色区域
    threshold(redChannel, output, RED_THRESHOLD, BINARY_MAX_VAL, THRESH_BINARY);

    // 3. 高斯模糊：去除高频噪声
    GaussianBlur(output, output, GAUSSIAN_KERNEL, 0);

    // 4. 形态学操作：先腐蚀去噪，再膨胀恢复灯条轮廓
    Mat kernel = getStructuringElement(MORPH_RECT, MORPH_KERNEL);
    erode(output, output, kernel);    // 腐蚀去除小噪声
    dilate(output, output, kernel);   // 膨胀恢复灯条形状
}

/**
 * @brief 灯条检测函数：从二值图像中筛选符合条件的灯条
 * @param binary 预处理后的二值图像
 * @return 符合条件的灯条矩形列表
 */
vector<Rect> detectLightBars(const Mat& binary) {
    vector<Rect> lightBars;
    if (binary.empty()) return lightBars;

    // 1. 检测轮廓
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(binary, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // 2. 筛选灯条轮廓
    for (const auto& contour : contours) {
        // 过滤过小轮廓
        double area = contourArea(contour);
        if (area < 50.0) continue;  // 面积过小视为噪声

        // 计算轮廓外接矩形
        Rect boundingRect = cv::boundingRect(contour);
        if (boundingRect.width == 0) continue;  // 避免除零错误

        // 筛选长条状灯条（高宽比、尺寸过滤）
        float aspectRatio = static_cast<float>(boundingRect.height) / boundingRect.width;
        if (aspectRatio >= ASPECT_RATIO_MIN && 
            boundingRect.height > MIN_HEIGHT && 
            boundingRect.width > MIN_WIDTH) {
            lightBars.push_back(boundingRect);
        }
    }

    return lightBars;
}

/**
 * @brief 灯条匹配函数：从灯条列表中找到最相似的一对
 * @param lightBars 灯条矩形列表
 * @return 匹配的灯条对（包含有效性标记）
 */
LightBarPair matchLightBarPair(const vector<Rect>& lightBars) {
    // 至少需要2个灯条才能匹配
    if (lightBars.size() < 2) {
        return LightBarPair();
    }

    // 寻找面积最接近的一对灯条（假设成对灯条尺寸相似）
    int minAreaDiff = INT_MAX;
    size_t bestIdx1 = 0, bestIdx2 = 1;

    for (size_t i = 0; i < lightBars.size(); ++i) {
        for (size_t j = i + 1; j < lightBars.size(); ++j) {
            int areaDiff = abs(lightBars[i].area() - lightBars[j].area());
            if (areaDiff < minAreaDiff) {
                minAreaDiff = areaDiff;
                bestIdx1 = i;
                bestIdx2 = j;
            }
        }
    }

    // 区分左右灯条（按x坐标）
    if (lightBars[bestIdx1].x < lightBars[bestIdx2].x) {
        return LightBarPair(lightBars[bestIdx1], lightBars[bestIdx2]);
    } else {
        return LightBarPair(lightBars[bestIdx2], lightBars[bestIdx1]);
    }
}

/**
 * @brief 绘制函数：在原始帧上绘制灯条和包围框
 * @param frame 原始视频帧
 * @param lightPair 匹配的灯条对
 */
void drawDetectionResult(Mat& frame, const LightBarPair& lightPair) {
    if (frame.empty() || !lightPair.isValid) return;

    // 1. 绘制灯条外接矩形
    rectangle(frame, lightPair.left, Scalar(0, 255, 0), 2);   // 左侧灯条：绿色
    rectangle(frame, lightPair.right, Scalar(0, 255, 0), 2);  // 右侧灯条：绿色

    // 2. 计算四边形顶点（灯条中心线上的上下点）
    Point leftTop(lightPair.left.x + lightPair.left.width/2, lightPair.left.y);
    Point leftBottom(lightPair.left.x + lightPair.left.width/2, lightPair.left.y + lightPair.left.height);
    Point rightBottom(lightPair.right.x + lightPair.right.width/2, lightPair.right.y + lightPair.right.height);
    Point rightTop(lightPair.right.x + lightPair.right.width/2, lightPair.right.y);

    // 3. 绘制包围四边形
    vector<Point> vertices = {leftTop, leftBottom, rightBottom, rightTop};
    for (size_t i = 0; i < vertices.size(); ++i) {
        line(frame, vertices[i], vertices[(i+1)%4], Scalar(0, 0, 255), 2);  // 红色四边形
    }
}

int main() {
    // 1. 打开视频文件
    VideoCapture capture(VIDEO_PATH);
    if (!capture.isOpened()) {
        cerr << "错误：无法打开视频文件！路径：" << VIDEO_PATH << endl;
        return -1;
    }

    // 2. 逐帧处理
    Mat frame, binary;
    while (true) {
        // 读取当前帧
        capture >> frame;
        if (frame.empty()) {
            cout << "视频处理完毕或读取失败" << endl;
            break;
        }

        // 3. 预处理
        preprocessFrame(frame, binary);

        // 4. 检测灯条
        vector<Rect> lightBars = detectLightBars(binary);

        // 5. 匹配灯条对
        LightBarPair matchedPair = matchLightBarPair(lightBars);

        // 6. 绘制结果
        drawDetectionResult(frame, matchedPair);

        // 7. 显示窗口
        imshow("预处理二值图", binary);
        imshow("红色灯条识别结果", frame);

        // 按ESC键退出（ASCII码27）
        char key = waitKey(10);
        if (key == 27) {
            cout << "用户手动退出" << endl;
            break;
        }
    }

    // 8. 释放资源
    capture.release();
    destroyAllWindows();
    return 0;
}
    