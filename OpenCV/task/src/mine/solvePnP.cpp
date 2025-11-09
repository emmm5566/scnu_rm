#include <opencv2/opencv.hpp>
#include <vector>



cv::Mat preProcessing(cv::Mat & img);
std::vector<cv::RotatedRect> getLightBars(cv::Mat & img);
std::vector<cv::Point2f> getArmor(std::vector<cv::RotatedRect> & lightBars);
void pnp(cv::Mat & img, std::vector<cv::Point2f> & armorPoints);



int main()
{
    //读取图像
    cv::Mat img = cv::imread("../img/pnp.jpg"); //../回到上一级目录task
    if(img.empty())
    {
        std::cout << "Error opening" << std::endl;
        return -1;
    }

    //预处理
    cv::Mat imgPre = preProcessing(img);

    //最小外接矩形
    std::vector<cv::RotatedRect> lightBars = getLightBars(imgPre);
    for(auto & lightBar : lightBars)
    {
        cv::Point2f vertices[4];
        lightBar.points(vertices);
        for(int i=0; i<4; i++)
        {
            cv::line(img, vertices[i], vertices[(i+1)%4], cv::Scalar(0,255,0), 3);
        }
    }

    //大外接矩形
    std::vector< cv::Point2f > armorPoints = getArmor(lightBars);
    if(!armorPoints.empty())
    {
        cv::RotatedRect rect = cv::minAreaRect(armorPoints);
        for(int i=0; i<4; i++)
        {
            line(img, armorPoints[i], armorPoints[(i+1)%4], cv::Scalar(0,0,255), 2);
        }
    }

    //pnp解算
    pnp(img, armorPoints);

    cv::imshow("Image", img);
    cv::imshow("ImagePre", imgPre);
    cv::waitKey(0);

    return 0;
}



cv::Mat preProcessing(cv::Mat & img)
{
    cv::Mat channels[3];
    cv::split(img, channels);
    cv::Mat binary, Gaussian;
    cv::threshold(channels[2], binary, 200, 255, 0);
    cv::GaussianBlur(binary, Gaussian, cv::Size(5,5), 0);

    return Gaussian;
}

std::vector<cv::RotatedRect> getLightBars(cv::Mat & img)
{
    //轮廓检测
    std::vector< std::vector <cv::Point> > contours;
    std::vector< cv::Vec4i > hierarchy;
    cv::findContours(img, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    std::vector< cv::RotatedRect > lightBars;

    //最小外接矩形
    for(auto & contour : contours)
    {
        double area = cv::contourArea(contour);
        if(area < 50.0) continue;

        cv::RotatedRect minRect = cv::minAreaRect(contour);

        float width = minRect.size.width;
        float height = minRect.size.height;
        if(width == 0 || height == 0) continue;
        float aspectRatio = std::max(width, height) / std::min(width, height);
        if(aspectRatio >= 1.3 && std::max(height, width)>36)
        {
            lightBars.push_back(minRect);
        }
    }

    return lightBars;
}

std::vector<cv::Point2f> getArmor(std::vector<cv::RotatedRect> & lightBars)
{
    if(lightBars.size() < 2) 
        return {}; //返回一个空向量

    //收集所有顶点
    std::vector< cv::Point2f > allVertices;
    cv::Point2f pts[4];
    lightBars[0].points(pts);
    for(int i=0; i<4; i++)
    {
        allVertices.push_back(pts[i]);
    } 
    lightBars[1].points(pts);
    for(int i=0; i<4; i++)
    {
        allVertices.push_back(pts[i]);
    }

    //大矩形各顶点
    float minX = allVertices[0].x, maxX = allVertices[0].x;
    float minY = allVertices[0].y, maxY = allVertices[0].y;
    for(auto & allVertice : allVertices)
    {
        minX = std::min(minX, allVertice.x);
        maxX = std::max(maxX, allVertice.x);
        minY = std::min(minY, allVertice.y);
        maxY = std::max(maxY, allVertice.y);
    }
    std::vector< cv::Point2f > rectVertices; 
    rectVertices.push_back(cv::Point2f(minX, minY));
    rectVertices.push_back(cv::Point2f(maxX, minY));
    rectVertices.push_back(cv::Point2f(maxX, maxY));
    rectVertices.push_back(cv::Point2f(minX, maxY));   

    return rectVertices;
}

void pnp(cv::Mat & img, std::vector<cv::Point2f> & armorPoints)
{
    //世界坐标系中三维点
    float LIGHTBAR_LENGTH = 0.056; //灯条长度m
    float ARMOR_WIDTH = 0.135; //装甲板宽度m
    std::vector< cv::Point3f> objectPoints
    {
        {-ARMOR_WIDTH/2, -LIGHTBAR_LENGTH/2, 0},
        {ARMOR_WIDTH/2, -LIGHTBAR_LENGTH/2, 0},
        {ARMOR_WIDTH/2, LIGHTBAR_LENGTH/2, 0},
        {-ARMOR_WIDTH/2, LIGHTBAR_LENGTH/2, 0}
    };

    //相机内参
    //cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);  
    //cv::Mat_<double>(3,3)
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 
        2422.61547, 0, 706.68406,
        0, 2420.80771, 564.29241,
        0, 0, 1);

    //畸变系数
    //cv::Mat distort_coeffs = (cv::Mat_<double>(1,5) << k1, k2, p1, p2, k3);
    //cv::Mat_<double>(1,5)
    cv::Mat distCoeffs = (cv::Mat_<double>(5,1) << -0.018647, 0.084359, -0.000925, 0.000312, 0.000000);

    //解算PnP
    cv::Mat rvec, tvec;
    cv::solvePnP(objectPoints, armorPoints, cameraMatrix, distCoeffs, rvec, tvec);

    std::cout << "tvec: " << tvec.at<double>(0) << ", " << tvec.at<double>(1) << ", " << tvec.at<double>(2) << std::endl;
    std::cout << "rvec: " << rvec.at<double>(0) << ", " << rvec.at<double>(1) << ", " << rvec.at<double>(2) << std::endl;
    std::string tvecText = "tvec  X:" + std::to_string(tvec.at<double>(0)) 
        + "  Y:" + std::to_string(tvec.at<double>(1))
        + "  Z:" + std::to_string(tvec.at<double>(2));
    std::string rvecText = "rvec  X:" + std::to_string(rvec.at<double>(0)) 
        + "  Y:" + std::to_string(rvec.at<double>(1))
        + "  Z:" + std::to_string(rvec.at<double>(2));
    cv::putText(img, tvecText, cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0,255,255), 1);
    cv::putText(img, rvecText, cv::Point(20, 50), cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0,255,255), 1);
    // tvec.at<double>(0) 得到一个double变量，它是tvec中首个元素的值

    //计算欧拉角（YXZ顺序，单位：弧度→角度）
    cv::Mat rmat; //旋转矩阵
    cv::Rodrigues(rvec, rmat);
    // 1. pitch（俯仰角，绕Y轴）：θ₁ = atan2(R[0][2], R[2][2])
    // double pitch = std::atan2(rmat.at<double>(0, 2), rmat.at<double>(2, 2)) * 180 / CV_PI;
    // 乘以180 / CV_PI将弧度转为角度
    double pitch = std::atan2(rmat.at<double>(0,2), rmat.at<double>(2,2));
    // 2. roll（滚转角，绕X轴）：θ₂ = -arcsin(R[1][2])
    double roll = - std::asin(rmat.at<double>(1,2));
    // 3. yaw（偏航角，绕Z轴）：θ₃ = atan2(R[1][0], R[1][1])
    double yaw = std::atan2(rmat.at<double>(1,0), rmat.at<double>(1,1));
    std::cout << "euler angles  pitch:" << pitch << ", roll:" << roll << ", yaw:" << yaw << std::endl;
    //std::to_string((int)(pitch*10)/10.0):  (int)(angle*10)/10.0保留 1 位小数，避免文本过长
    std::string eulerText = "euler angles  pitch:" + std::to_string(pitch) 
        + "  roll:" + std::to_string(roll)
        + "  yaw:" + std::to_string(yaw);    
    cv::putText(img, eulerText, cv::Point(20, 70), cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0,255,255), 1);
}



// void cv::Rodrigues(
//     InputArray src,  // 输入：旋转向量（3×1 或 1×3）或旋转矩阵（3×3）
//     OutputArray dst, // 输出：转换后的旋转矩阵（3×3）或旋转向量（3×1 或 1×3）
//     OutputArray jacobian = noArray()  // 可选：雅可比矩阵（一般不用）
// );

//INT_YXZ pitch=arctan2(m13,m33) roll=-arcsin(m33) yaw=arctan2(m21,m22)

//atan2 计算两个参数的反正切值
//<cmath>
//double atan2(double y, double x);  // 计算 y/x 的反正切，返回值范围：[-π, π] 弧度


//旋转向量rvec  通过 罗德里格斯公式cv::Rodrigues  变为 旋转矩阵rmat
//旋转矩阵rmat  通过 反三角函数  变为 欧拉角
//欧拉角  变为 四元数

// //（1）旋转向量—>旋转矩阵—>欧拉角
//
// 旋转向量转旋转矩阵
// theta = np.linalg.norm(rvec)
// r = rvec / theta
// R_ = np.array([[0, -r[2][0], r[1][0]],
//                [r[2][0], 0, -r[0][0]],
//                [-r[1][0], r[0][0], 0]])
// R = np.cos(theta) * np.eye(3) + (1 - np.cos(theta)) * r * r.T + np.sin(theta) * R_
// print('旋转矩阵')
// print(R)
//
//
// 旋转矩阵转欧拉角
// def isRotationMatrix(R):
//     Rt = np.transpose(R)   #旋转矩阵R的转置
//     shouldBeIdentity = np.dot(Rt, R)   #R的转置矩阵乘以R
//     I = np.identity(3, dtype=R.dtype)           # 3阶单位矩阵
//     n = np.linalg.norm(I - shouldBeIdentity)   #np.linalg.norm默认求二范数
//     return n < 1e-6                            # 目的是判断矩阵R是否正交矩阵（旋转矩阵按道理须为正交矩阵，如此其返回值理论为0）
//
//
// def rotationMatrixToAngles(R):
//     assert (isRotationMatrix(R))   #判断是否是旋转矩阵（用到正交矩阵特性）
// 
//     sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])  #矩阵元素下标都从0开始（对应公式中是sqrt(r11*r11+r21*r21)），sy=sqrt(cosβ*cosβ)
// 
//     singular = sy < 1e-6   # 判断β是否为正负90°
// 
//     if not singular:   #β不是正负90°
//         x = math.atan2(R[2, 1], R[2, 2])
//         y = math.atan2(-R[2, 0], sy)
//         z = math.atan2(R[1, 0], R[0, 0])
//     else:              #β是正负90°
//         x = math.atan2(-R[1, 2], R[1, 1])
//         y = math.atan2(-R[2, 0], sy)   #当z=0时，此公式也OK，上面图片中的公式也是OK的
//         z = 0
//    
//     x = x*180.0/3.141592653589793
//     y = y*180.0/3.141592653589793
//     z = z*180.0/3.141592653589793
// 
//     return np.array([x, y, z])

// // （2）旋转向量—>四元数—>欧拉角
// # 从旋转向量转换为欧拉角
// def get_euler_angle(rotation_vector):
//     # calculate rotation angles
//     theta = cv2.norm(rotation_vector, cv2.NORM_L2)
//    
//     # transformed to quaterniond
//     w = math.cos(theta / 2)
//     x = math.sin(theta / 2)*rotation_vector[0][0] / theta
//     y = math.sin(theta / 2)*rotation_vector[1][0] / theta
//     z = math.sin(theta / 2)*rotation_vector[2][0] / theta
//    
//     ysqr = y * y
//     # pitch (x-axis rotation)
//     t0 = 2.0 * (w * x + y * z)
//     t1 = 1.0 - 2.0 * (x * x + ysqr)
//     print('t0:{}, t1:{}'.format(t0, t1))
//     pitch = math.atan2(t0, t1)
//    
//     # yaw (y-axis rotation)
//     t2 = 2.0 * (w * y - z * x)
//     if t2 > 1.0:
//         t2 = 1.0
//     if t2 < -1.0:
//         t2 = -1.0
//     yaw = math.asin(t2)
//    
//     # roll (z-axis rotation)
//     t3 = 2.0 * (w * z + x * y)
//     t4 = 1.0 - 2.0 * (ysqr + z * z)
//     roll = math.atan2(t3, t4)
//    
//     print('pitch:{}, yaw:{}, roll:{}'.format(pitch, yaw, roll))
//    
// 	# 单位转换：将弧度转换为度
//     Y = int((pitch/math.pi)*180)
//     X = int((yaw/math.pi)*180)
//     Z = int((roll/math.pi)*180)
//    
//     return 0, Y, X, Z



//欧拉角 
//pitch：俯仰角，表示物体绕x轴旋转
//yaw：偏航角，表示物体绕y轴旋转
//roll：翻滚角，表示物体绕z轴旋转
//Z-Y-X，即 偏航角yaw-俯仰角pitch-滚转角roll
// Z 轴：相机光轴方向（指向拍摄物体，向前为正），yaw；
// Y 轴：相机竖直方向（向上为正，与 Z 轴垂直），pitch；
// X 轴：相机水平方向（向右为正，与 Z、Y 轴构成右手坐标系），roll