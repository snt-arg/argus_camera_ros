#include "argus_camera_ros/test_node.hpp"

#include <cv_bridge/cv_bridge.h>
#include <fcntl.h>
#include <unistd.h>

#include <chrono>
#include <memory>
#include <rclcpp/time.hpp>

#include "argus_camera_ros/argus_camera.hpp"

// #include "argus_camera_ros/argus_camera.hpp"

TestNode::TestNode()
    : Node("test_node"), node_handle_(std::shared_ptr<TestNode>(this, [](auto*) {})) {
    silencer.enable();
    cameraProvider = UniqueObj<CameraProvider>(CameraProvider::create());
    silencer.disable();

    logger.setLogger([this](LogLevel level, const std::string& msg) {
        switch (level) {
            case LogLevel::DEBUG:
                RCLCPP_DEBUG(this->get_logger(), "%s", msg.c_str());
                break;
            case LogLevel::INFO:
                RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
                break;
            case LogLevel::WARN:
                RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
                break;
            case LogLevel::ERROR:
                RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
                break;
        }
    });

    logger.info("Starting node");

    CameraConfig config0 = {.id = 0, .mode = 2, .width = 640, .height = 480, .fps = 30};
    camera0 = std::make_unique<ArgusCamera>(cameraProvider.get(), config0, logger);
    img_it_ = std::make_shared<image_transport::ImageTransport>(node_handle_);
    img_pub_ = img_it_->advertise("camera/image_raw", 10);

    camera0->init();
    camera0->startCapture();
    camera0->setFrameCallback([this](const CVFrameStamped& stampedFrame) {
        auto msg =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", stampedFrame.frame)
                .toImageMsg();
        msg->header.stamp = rclcpp::Time(stampedFrame.sofTS);
        img_pub_.publish(msg);
    });

    // pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(33),
    //                                      std::bind(&TestNode::callback, this));
}

void TestNode::callback() {
    CVFrameStamped stampedFrame = camera0->getLatestFrame();
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(stampedFrame.sofTS);
    header.frame_id = "camera";

    sensor_msgs::msg::Image::SharedPtr msg =
        cv_bridge::CvImage(header, "bgr8", stampedFrame.frame).toImageMsg();

    img_pub_.publish(msg);
}
