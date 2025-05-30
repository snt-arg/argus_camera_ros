#ifndef SINGLE_CAMERA_HPP
#define SINGLE_CAMERA_HPP

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "argus_camera_ros/argus_camera.hpp"
#include "argus_camera_ros/logger.hpp"
#include "argus_camera_ros/ptp_converter.hpp"
#include "argus_camera_ros/stdout_silencer.hpp"

namespace argus_camera_ros {

using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;

enum TimestampMode { TIME_FROM_TSC = 0, TIME_FROM_PTP = 1, TIME_FROM_ROS = 2 };

class SingleCameraNode : public rclcpp::Node {
   public:
    SingleCameraNode();
    ~SingleCameraNode() = default;

    void startCapturing(void);

   private:
    StdoutSilencer stdoutSilencer_;

   protected:
    void initImageTransport(void);
    void initCameras(void);
    void readParameters(void);
    void declareParameters(void);
    void publishImageCallback(void);
    void setupLogger(void);

    void publishImage(const CVFrameStamped &stampedFrame);
    void publishCameraInfo(const int cameraIdx, const std_msgs::msg::Header &header);

    TimestampMode convertStringToTimestampMode(std::string mode) {
        TimestampMode tsMode;

        if (mode == "TIME_FROM_TSC") {
            tsMode = TIME_FROM_TSC;
        } else if (mode == "TIME_FROM_PTP") {
            tsMode = TIME_FROM_PTP;
        } else if (mode == "TIME_FROM_ROS") {
            tsMode = TIME_FROM_ROS;
        } else {
            throw std::runtime_error(
                "Unknown timestamp mode: " + mode +
                "Availabe are TIME_FROM_TSC, TIME_FROM_PTP and TIME_FROM_ROS.");
        }
        return tsMode;
    }

   protected:
    // Image Transport
    rclcpp::Node::SharedPtr nodeHandle_;
    shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Publisher imgPub_;
    string cameraName_;

    // Camera Calibration
    unique_ptr<camera_info_manager::CameraInfoManager> cameraInfo_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cameraInfoPub_;
    string cameraInfoUrl_;

    // Cameras
    UniqueObj<CameraProvider> cameraProvider_;
    shared_ptr<ArgusCamera> camera_;
    CameraConfig cameraConfig_;
    int deviceID_;

    rclcpp::TimerBase::SharedPtr pubTimer_;

    Logger logger_;

    // Timestamps
    TimestampMode timestampMode = TimestampMode::TIME_FROM_TSC;
    PTPConverter ptpConverter;
};

}  // namespace argus_camera_ros

#endif  // SINGLE_CAMERA_HPP!
