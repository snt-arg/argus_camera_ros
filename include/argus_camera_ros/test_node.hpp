#ifndef TEST_NODE_HPP
#define TEST_NODE_HPP

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "argus_camera_ros/argus_camera.hpp"

using namespace argus_camera_ros;

class TestNode : public rclcpp::Node {
   public:
    TestNode();

    void callback();

   private:
    void pub_callback_();
    void build_camera_msg_(std::vector<sensor_msgs::msg::Image::SharedPtr> &msgs,
                           std::vector<cv::Mat> &camera_frames);

    rclcpp::Node::SharedPtr node_handle_;

    std::unique_ptr<ArgusCamera> camera_;

    std::shared_ptr<image_transport::ImageTransport> img_it_;
    image_transport::Publisher img_pub_;
    rclcpp::TimerBase::SharedPtr pub_timer_;
    int count = 0;
};

#endif  // TEST_NODE_HPP!
