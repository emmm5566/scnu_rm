#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
    //初始化rclcpp
    rclcpp::init(argc, argv);
    //产生一个cpp_node节点
    auto node = std::make_shared<rclcpp::Node>("cpp_node");
    //打印一句介绍
    RCLCPP_INFO(node->get_logger(), "你好,c++节点！");
    //运行节点，并检测退出信号 ctrl+c
    rclcpp::spin(node);
    //停止运行
    rclcpp::shutdown();

    return 0;
}

//ldd [选项] 目标文件
//ldd,List Dynamic Dependencies,用于列出可执行文件或共享库(.so 文件)所依赖的动态链接库

//"... not found"经常是因为环境原因，检查环境
//printenv | grep
//ros2 pkg prefix

//注：这里的工作空间是session3，要在工作空间下colcon build，不能在功能包里面
//session3/src/pkg_name   #session3是工作空间
//chapter2/pkg_name   #chapter2是工作空间

//ctrl+左键单击 查看定义（头文件）