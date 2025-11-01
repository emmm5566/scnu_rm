#include "stdio.h"
#include<iostream> 
// 引入OpenCV核心库（数据结构、基础运算）
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
// 引入OpenCV高层GUI库（窗口显示、交互）
#include <opencv2/highgui/highgui.hpp>

using namespace std;  // 启用标准库命名空间
using namespace cv;   // 启用OpenCV命名空间

// 常量定义（针对红色灯条特性调整）
const int kThreashold = 160;         // 二值化阈值：红色灯条通常亮度低于蓝色，降低阈值以保留更多细节
const int kMaxVal = 255;             // 二值化最大值（白色）
const Size kGaussianBlueSize = Size(5, 5);  // 高斯模糊核大小（5x5，平衡降噪与轮廓保留）


int main()
{
    // 1. 初始化视频捕获对象
    VideoCapture video;  // 视频捕获类，用于读取视频文件或摄像头
    // 打开目标视频文件（需替换为实际视频路径）
    video.open("/home/emmm/Desktop/scnu_rm/OpenCV/task/img/task2_video.mp4");
    
    // 检查视频是否成功打开（新增容错处理）
    if (!video.isOpened()) {
        cerr << "错误：无法打开视频文件！请检查路径是否正确。" << endl;
        return -1;  // 打开失败则退出程序
    }

    // 2. 定义图像处理变量
    Mat frame;               // 存储视频每一帧（BGR格式）
    Mat channels[3];         // 存储通道分离结果（BGR→channels[0]=蓝, 1=绿, 2=红）
    Mat binary;              // 红色通道二值化图像（突出红色灯条）
    Mat Gaussian;            // 高斯模糊后图像（降噪）
    Mat dilateImg;           // 形态学处理后图像（增强灯条轮廓）
    vector<vector<Point>> contours;  // 存储检测到的轮廓（每个轮廓由一组点组成）
    vector<Vec4i> hierarchy;         // 存储轮廓层级关系（用于区分父子轮廓）
    Rect boundRect;          // 存储轮廓的外接矩形（轴对齐，非旋转）
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));  // 形态学操作核（用于腐蚀/膨胀）


    // 3. 逐帧处理视频
    for (;;) {  // 无限循环，直到视频结束或手动退出
        Rect point_array[20];  // 存储符合条件的红色灯条矩形（最多20个）
        video >> frame;        // 读取一帧图像到frame

        // 若读取的帧为空（视频结束），退出循环
        if (frame.empty()) {
            break;
        }


        // 4. 颜色通道处理（核心修改：适配红色灯条）
        // 4.1 通道分离：将BGR图像拆为3个单通道
        split(frame, channels);  // channels[2]为红色通道（重点修改点：原代码用channels[0]蓝色通道）
        
        // 4.2 红色通道二值化：提取高亮度红色区域
        // 函数说明：threshold(输入通道, 输出二值图, 阈值, 最大值, 类型)
        // 红色灯条在channels[2]中亮度较高，通过阈值160筛选（低于蓝色阈值220，因红色通常不极端高亮）
        threshold(channels[2], binary, kThreashold, kMaxVal, 0);


        // 5. 预处理优化（针对红色易受干扰的特性）
        // 5.1 高斯模糊：平滑图像，减少高频噪声（如红色背景杂点）
        GaussianBlur(binary, Gaussian, kGaussianBlueSize, 0);

        // 5.2 形态学操作：先腐蚀去噪，再膨胀恢复灯条轮廓
        // 红色灯条易受环境干扰（如红色广告牌、衣物），腐蚀可去除小噪声
        erode(Gaussian, dilateImg, kernel);  // 腐蚀：消除细小白色杂点
        dilate(dilateImg, dilateImg, kernel);  // 膨胀：恢复灯条原有轮廓，避免过度腐蚀


        // 6. 轮廓检测：从预处理后的图像中提取红色灯条轮廓
        // 函数说明：findContours(输入二值图, 轮廓集合, 层级, 检索模式, 逼近方法)
        // 使用处理后的dilateImg而非原始Gaussian，减少噪声轮廓
        findContours(dilateImg, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);


        // 7. 红色灯条筛选：保留符合形态特征的轮廓
        int index = 0;  // 记录符合条件的灯条数量
        for (int i = 0; i < contours.size(); i++) {  // 遍历所有检测到的轮廓
            // 计算轮廓的外接矩形（最小轴对齐矩形）
            boundRect = boundingRect(Mat(contours[i]));

            try {  // 异常捕获：避免矩形宽度为0导致除零错误
                // 筛选条件（针对红色灯条形态优化）：
                // 1. 高宽比≥1.5：红色灯条通常更细长（原代码1.3，此处提高以过滤宽扁干扰）
                // 2. 高度>30：过滤过小噪声（低于原代码36，因红色灯条可能稍短）
                // 3. 宽度>15：过滤过窄噪声（低于原代码20，适应红色灯条可能的窄边）
                if (double(boundRect.height / boundRect.width) >= 1.5 && 
                    boundRect.height > 30 && 
                    boundRect.width > 15) {
                    
                    point_array[index] = boundRect;  // 符合条件的灯条存入数组
                    index++;  // 计数递增
                }
            }
            catch (const char* msg) {
                cout << "灯条筛选出错：" << msg << endl;  // 输出错误信息
            }
        }


        // 8. 匹配成对灯条（假设灯条成对出现，如装甲板两侧）
        int point_near[2] = {0, 0};  // 存储最相似的两个灯条索引
        int min_area_diff = 10000;   // 初始化最小面积差（用于寻找最相似灯条）

        // 仅当检测到至少2个灯条时才进行匹配
        if (index >= 2) {
            // 遍历所有灯条对，计算面积差，寻找最相似的一对
            for (int i = 0; i < index - 1; i++) {
                for (int j = i + 1; j < index; j++) {
                    // 面积差越小，两个灯条越可能是一对
                    int area_diff = abs(point_array[i].area() - point_array[j].area());
                    if (area_diff < min_area_diff) {
                        min_area_diff = area_diff;
                        point_near[0] = i;  // 更新第一个灯条索引
                        point_near[1] = j;  // 更新第二个灯条索引
                    }
                }
            }
        }


        // 9. 绘制灯条包围四边形
        try {
            // 检查是否有足够的灯条进行绘制
            if (index < 2) {
                throw "未检测到足够的红色灯条（至少需要2个）";
            }

            // 获取最相似的两个灯条矩形
            Rect rectangle_1 = point_array[point_near[0]];
            Rect rectangle_2 = point_array[point_near[1]];

            // 简单校验矩形有效性（避免x坐标为0的无效矩形）
            if (rectangle_2.x == 0 || rectangle_1.x == 0) {
                throw "检测到无效灯条位置（x坐标为0）";
            }

            // 计算两个灯条的关键顶点（取矩形中心线上的上下点，确保连线对称）
            Point point1 = Point(rectangle_1.x + rectangle_1.width/2, rectangle_1.y);  // 灯条1顶部中点
            Point point2 = Point(rectangle_1.x + rectangle_1.width/2, rectangle_1.y + rectangle_1.height);  // 灯条1底部中点
            Point point3 = Point(rectangle_2.x + rectangle_2.width/2, rectangle_2.y);  // 灯条2顶部中点
            Point point4 = Point(rectangle_2.x + rectangle_2.width/2, rectangle_2.y + rectangle_2.height);  // 灯条2底部中点

            // 定义四边形顶点顺序（确保闭合）
            Point p[4] = { point1, point2, point4, point3 };

            // 绘制四边形：使用红色线条（与灯条颜色一致，便于观察）
            for (int i = 0; i < 4; i++) {
                // 函数说明：line(目标图像, 起点, 终点, 颜色, 线宽)
                // Scalar(0,0,255)为红色（BGR格式），线宽2像素
                line(frame, p[i%4], p[(i+1)%4], Scalar(0, 0, 255), 2);
            }           
        }
        catch (const char* msg) {
            cout << "绘制出错：" << msg << endl;  // 捕获并输出异常信息
        }


        // 10. 显示结果与交互
        imshow("红色通道二值化", binary);  // 新增调试窗口：观察红色灯条提取效果
        imshow("视频结果", frame);         // 显示最终绘制结果

        // 等待10ms，若有按键输入则退出循环（按任意键退出）
        if (waitKey(10) >= 0) {
            break;
        }
    }


    // 11. 释放资源
    video.release();          // 释放视频捕获资源
    cv::destroyAllWindows();  // 关闭所有OpenCV窗口
    return 0;
}
