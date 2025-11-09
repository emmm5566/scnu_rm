#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp" //导入消息接口

class TopicPublisher : public rclcpp::Node
{
private:
    // 声明话题发布者
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_publisher_; 
    // 声明定时器
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 回调函数
    void timer_callback()
    {
        // 创建消息
        std_msgs::msg::String message;
        message.data = "forward";
        // 日志打印
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        // 发布消息
        command_publisher_->publish(message);
    }

public:
    // 构造函数,有一个参数为节点名称
    TopicPublisher(std::string node_name) : Node(node_name)
    {
        RCLCPP_INFO(this->get_logger(), "Node '%s' has been started.", node_name.c_str());
        // 创建话题发布者
        command_publisher_ = this->create_publisher<std_msgs::msg::String>("command", 10);
        // 创建定时器，周期为500ms，定时发布
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), 
            std::bind(&TopicPublisher::timer_callback, this));
    }
};



int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<TopicPublisher>("topic_publisher");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}





// 测试：
// # 查看列表
// ros2 topic list
// # 输出内容
// ros2 topic echo /command

// 查看接口：
// ros2 interface show std_msgs/msg/String 