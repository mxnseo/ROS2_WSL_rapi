#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/time_source.hpp"
#include <memory>
#include <chrono>

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("pub_node");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    auto mypub = node->create_publisher<std_msgs::msg::String>("hello_topic", qos_profile );
    std_msgs::msg::String message;
    message.data = "Hello world!";
    rclcpp::WallRate loop_rate(1.0);

    while(rclcpp::ok()) {
        RCLCPP_INFO(node->get_logger(), "Publish: %s",
        message.data.c_str());
        mypub->publish(message);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}