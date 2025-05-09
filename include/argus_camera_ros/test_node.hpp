#ifndef TEST_NODE_HPP
#define TEST_NODE_HPP

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "argus_camera_ros/argus_camera.hpp"
#include "argus_camera_ros/logger.hpp"
#include "argus_camera_ros/stdout_silencer.hpp"

using namespace argus_camera_ros;

class TestNode : public rclcpp::Node {
   public:
    TestNode();
    ~TestNode() { cameraProvider.reset(); }

    void callback();

   private:
    void pub_callback_();
    UniqueObj<CameraProvider> cameraProvider;

    rclcpp::Node::SharedPtr node_handle_;
    Logger logger;

    StdoutSilencer silencer;
    std::unique_ptr<ArgusCamera> camera0;
    std::unique_ptr<ArgusCamera> camera1;

    std::shared_ptr<image_transport::ImageTransport> img_it_;
    image_transport::Publisher img_pub_;
    rclcpp::TimerBase::SharedPtr pub_timer_;
    int count = 0;
};

#endif  // TEST_NODE_HPP!
