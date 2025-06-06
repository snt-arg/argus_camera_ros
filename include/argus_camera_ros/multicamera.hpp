/**
 * @file include/argus_camera_ros/multicamera.hpp
 * @brief Defines a ROS node which controls 4 instances of ArgusCamera and publishes
 * their camera frames with their respective CameraInfo.
 *
 * No sync is applied.
 */

#ifndef MULTICAMERA_HPP
#define MULTICAMERA_HPP

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>

#include "argus_camera_ros/argus_camera.hpp"
#include "argus_camera_ros/logger.hpp"
#include "argus_camera_ros/ptp_converter.hpp"
#include "argus_camera_ros/stdout_silencer.hpp"

namespace argus_camera_ros {

#define NUM_CAMS 4

using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;

enum TimestampMode { TIME_FROM_TSC = 0, TIME_FROM_PTP = 1, TIME_FROM_ROS = 2 };

class MultiCameraNode : public rclcpp::Node {
   public:
    MultiCameraNode();
    ~MultiCameraNode() = default;

    void startCapturing(void);

   protected:
    void initImageTransport(void);
    void initCameras(void);
    void readParameters(void);
    void declareParameters(void);
    void publishImageCallback(void);
    void setupLogger(void);

    void publishImage(const int cameraIdx, const CVFrameStamped &stampedFrame);
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

   private:
    StdoutSilencer stdoutSilencer_;

   protected:
    // Image Transport
    rclcpp::Node::SharedPtr nodeHandle_;
    shared_ptr<image_transport::ImageTransport> it_;
    vector<image_transport::Publisher> imgPubs_;
    vector<string> cameraNames_;

    // Camera Calibration
    vector<unique_ptr<camera_info_manager::CameraInfoManager>> cameraInfos_;
    vector<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> cameraInfoPubs_;
    vector<string> cameraInfoUrls_;

    // Cameras
    UniqueObj<CameraProvider> cameraProvider_;
    vector<shared_ptr<ArgusCamera>> cameras_;
    CameraConfig cameraConfig_;

    rclcpp::TimerBase::SharedPtr pubTimer_;

    Logger logger_;

    // Timestamps
    TimestampMode timestampMode = TimestampMode::TIME_FROM_TSC;
    PTPConverter ptpConverter;
};

}  // namespace argus_camera_ros

#endif  // !MULTICAMERA_HPP
