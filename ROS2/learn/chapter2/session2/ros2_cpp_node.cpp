#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("cpp_node");
    RCLCPP_INFO(node->get_logger(), "你好,c++节点！");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

//ctrl + shift + 5 打开新终端
//ros2 node list 查看节点列表
//ros2 node info node_name 查看节点信息

//colcon build
//colcon build --package-select pkg_name
//source install/setup.bash
//ros2 run pkg_name node_name