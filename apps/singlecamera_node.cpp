#include <rclcpp/rclcpp.hpp>

#include "argus_camera_ros/test_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<TestNode> camera_node = std::make_shared<TestNode>();

    rclcpp::spin(camera_node);

    rclcpp::shutdown();
    return 0;
}
