#include "argus_camera_ros/test_node.hpp"

#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <memory>

#include "argus_camera_ros/argus_camera.hpp"

TestNode::TestNode()
    : Node("test_node"), node_handle_(std::shared_ptr<TestNode>(this, [](auto *) {})) {
    Argus::UniqueObj<Argus::CameraProvider> cameraProvider =
        UniqueObj<CameraProvider>(CameraProvider::create());
    camera_ = std::make_unique<ArgusCamera>(cameraProvider);

    camera_->init();
    camera_->startCapture();
    img_it_ = std::make_shared<image_transport::ImageTransport>(node_handle_);
    img_pub_ = img_it_->advertise("camera/image_raw", 10);

    pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / 30),
                                         std::bind(&TestNode::callback, this));
}

void TestNode::callback() {
    cv::Mat frame;
    if (!camera_->getLatestFrame(frame)) {
        std::cout << "NO FRAME: " << count << std::endl;
        count++;
        return;
    }
    count = 0;
    std_msgs::msg::Header header;
    header.stamp = this->get_clock()->now();

    header.frame_id = "camera";
    sensor_msgs::msg::Image::SharedPtr msg =
        cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();

    img_pub_.publish(msg);
}
