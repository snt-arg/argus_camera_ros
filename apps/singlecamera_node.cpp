#include <rclcpp/rclcpp.hpp>

#include "argus_camera_ros/single_camera.hpp"

using namespace argus_camera_ros;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<SingleCameraNode> camera_node =
        std::make_shared<SingleCameraNode>();

    camera_node->startCapturing();

    rclcpp::spin(camera_node);

    rclcpp::shutdown();
    return 0;
}
