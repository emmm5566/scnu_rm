#include <opencv2/opencv.hpp>

#define HALF_LENGTH 29.5

int main()
{
    // // 世界坐标系中的三维点
    // // double LIGHTBAR_LENGTH = 0.056; // 灯条长度（m）
    // // double ARMOR_WIDTH = 0.135; //装甲板宽度（m）
    // // std::vector<cv::Point3f> object_points
    // // {
    // //     {-ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2, 0}, //1（x，y，z）
    // //     {ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2, 0}, //2
    // //     {ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2, 0}, //3
    // //     {-ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2, 0} //4
    // // };
    // // 图像上的二维点
    // // std::vector<cv::Point2f> img_points
    // // {
    // //     armor.left.top,
    // //     armor.right.top,
    // //     armor.right.bottom,
    // //     armor.left.bottom
    // // };
    // // 添加 objectPoints 和 imagePoints 的数据

    // //自定义的物体世界坐标系（mm）
    // std::vector<cv::Point3f> obj = std::vector<cv::Point3f>{
    //     cv::Point3f(-HALF_LENGTH, -HALF_LENGTH, 0), //1
    //     cv::Point3f(HALF_LENGTH, -HALF_LENGTH, 0), //2
    //     cv::Point3f(HALF_LENGTH, HALF_LENGTH, 0), //3
    //     cv::Point3f(-HALF_LENGTH, HALF_LENGTH, 0) //4
    // };

    // // 创建相机内参数矩阵
    // // cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);  
    // // 创建相机畸变系数矩阵
    // // cv::Mat distort_coeffs = (cv::Mat_<double>(1,5) << k1, k2, p1, p2, k3);
    // // 完善内参参数＆畸变系数参数 

    // // cv::Mat rvec; // 旋转向量
    // // cv::Mat tvec; // 平移向量
    // cv::Mat rVec = cv::Mat::zero(3, 1, CV_64FC1); // init rvec
    // cv::Mat tVec = cv::Mat::zero(3, 1, CV_64FC1); // init tvec

    // // bool success = cv::solvePnP(object_points, img_points, camera_matrix, distort_coeffs, rvec, tvec);
    // // if(success)
    // // {
    // //     // 获取旋转向量和平移向量的结果
    // //     cv::Mat rotationMatris;
    // //     cv::Rodrigues(revet, rotationMatris);

    // //     std::cout << "Rotation Vector:" << std::endl << rvec << std::endl;
    // //     std::cout << "Translation Vector:" << std::endl << tvec << std::endl;
    // //     std::cout << "Rotation Matrix:" << std::endl << rotationMatrix << std::endl;
    // // }

    // //进行位置解算
    // cv::solvePnP(obj, pnts, cam, dis, rVec, tVec, false, cv::SOLVEPNP_ITERATIVE);
    // //输出平移向量
    // std::cout << "tvec: " << tVec << std::endl;
    
    return 0;
}





// bool solvePnP(
//     InputArray objectPoints,   // 3D世界点集合（vector<Point3f>）
//     InputArray imagePoints,    // 对应的2D图像点集合（vector<Point2f>）
//     InputArray cameraMatrix,   // 相机内参矩阵（标定得到）
//     InputArray distCoeffs,     // 相机畸变系数（标定得到，无畸变时可传Mat::zeros）
//     OutputArray rvec,          // 输出旋转向量（Rodrigues向量，可转换为旋转矩阵）
//     OutputArray tvec,          // 输出平移向量（相机坐标系下物体的位置）
//     bool useExtrinsicGuess = false,  // 是否使用外部位姿初值（一般为false）
//     int flags = SOLVEPNP_ITERATIVE   // 求解方法（如迭代法、P3P法等）
// );
//
// cameraMatrix：相机内参矩阵，通过cv::calibrateCamera标定得到(fx/fy是焦距，cx/cy是图像中心) 
// cv::Mat_<double>(3,3)
// [fx,  0,  cx;
//  0,  fy, cy;
//  0,   0,  1]
// cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1); 
//
// distCoeffs：相机畸变系数，标定后得到（如[k1, k2, p1, p2, k3]）
// cv::Mat distort_coeffs = (cv::Mat_<double>(1,5) << k1, k2, p1, p2, k3);
//
// flags：
// SOLVEPNP_ITERATIVE：迭代法（鲁棒性强，需至少 4 个点）
// SOLVEPNP_P3P：P3P 法（仅需 3 个点，对噪声敏感）