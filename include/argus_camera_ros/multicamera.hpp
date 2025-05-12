#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>

#include "argus_camera_ros/argus_camera.hpp"
#include "argus_camera_ros/logger.hpp"

namespace argus_camera_ros {

#define NUM_CAMS 4

using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;
//
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

    // Configuration
    bool useRosTime;
};

}  // namespace argus_camera_ros
