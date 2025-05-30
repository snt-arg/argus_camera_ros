#include "argus_camera_ros/single_camera.hpp"

#include <cv_bridge/cv_bridge.h>
#include <fcntl.h>
#include <unistd.h>

#include <chrono>
#include <memory>
#include <rclcpp/time.hpp>

#include "argus_camera_ros/argus_camera.hpp"

namespace argus_camera_ros {

SingleCameraNode::SingleCameraNode()
    : Node("single_camera"),
      nodeHandle_(std::shared_ptr<SingleCameraNode>(this, [](auto*) {})) {
    setupLogger();

    declareParameters();
    readParameters();

    initCameras();
    initImageTransport();
}

void SingleCameraNode::startCapturing(void) {
    camera_->startCapture();

    // pubTimer_ = pubTimer_ =
    //     create_wall_timer(std::chrono::milliseconds(HZ_TO_MS(cameraConfig_.fps)),
    //                       std::bind(&SingleCameraNode::publishImageCallback, this));
}

void SingleCameraNode::initCameras(void) {
    stdoutSilencer_.enable();
    cameraProvider_ = UniqueObj<CameraProvider>(CameraProvider::create());
    stdoutSilencer_.disable();

    // TODO: use index from config
    camera_ =
        std::make_shared<ArgusCamera>(deviceID_, cameraProvider_.get(), cameraConfig_);
    if (!camera_->init()) {
        throw std::runtime_error("Failed to initilize a camera");
    }
    camera_->setNewFrameHook(
        [this](const CVFrameStamped& stampedFrame) { publishImage(stampedFrame); });
}

void SingleCameraNode::initImageTransport(void) {
    it_ = std::make_shared<image_transport::ImageTransport>(nodeHandle_);

    std::string topicName = "/camera/" + cameraName_ + "/image_raw";

    imgPub_ = it_->advertise(topicName, 10);

    cameraInfoPub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        "/camera/" + cameraName_ + "/camera_info", 10);

    cameraInfo_ = std::make_unique<camera_info_manager::CameraInfoManager>(
        this, cameraName_, cameraInfoUrl_);
}

void SingleCameraNode::publishImageCallback(void) {
    publishImage(camera_->getLatestFrame());
}

void SingleCameraNode::publishImage(const CVFrameStamped& stampedFrame) {
    std_msgs::msg::Header header;
    header.frame_id = cameraName_;

    switch (timestampMode) {
        case TimestampMode::TIME_FROM_PTP:
            header.stamp =
                rclcpp::Time(ptpConverter.convertTSCTimeToPTP(stampedFrame.sofTS));
            break;
        case TimestampMode::TIME_FROM_ROS:
            header.stamp = get_clock()->now();
            break;
        default:
            header.stamp = rclcpp::Time(stampedFrame.sofTS);
            break;
    }

    if (stampedFrame.frame.empty()) {
        RCLCPP_DEBUG(get_logger(), "EMPTY");
        return;
    }

    sensor_msgs::msg::Image::SharedPtr msg =
        cv_bridge::CvImage(header, "bgr8", stampedFrame.frame).toImageMsg();

    imgPub_.publish(msg);
    publishCameraInfo(deviceID_, header);
}

void SingleCameraNode::publishCameraInfo(const int cameraIdx,
                                         const std_msgs::msg::Header& header) {
    sensor_msgs::msg::CameraInfo camera_info = cameraInfo_->getCameraInfo();
    camera_info.header = header;
    cameraInfoPub_->publish(camera_info);
}

void SingleCameraNode::setupLogger(void) {
    logger_.setLogger([this](LogLevel level, const std::string& msg) {
        switch (level) {
            case LogLevel::INFO:
                RCLCPP_INFO(get_logger(), "%s", msg.c_str());
                break;
            case LogLevel::WARN:
                RCLCPP_WARN(get_logger(), "%s", msg.c_str());
                break;
            case LogLevel::ERROR:
                RCLCPP_ERROR(get_logger(), "%s", msg.c_str());
                break;
            case LogLevel::DEBUG:
                RCLCPP_DEBUG(get_logger(), "%s", msg.c_str());
                break;
        }
    });
}

void SingleCameraNode::declareParameters(void) {
    declare_parameter("device_id", 0);
    declare_parameter("camera_settings.mode", 2);
    declare_parameter("camera_settings.fps", 30);
    declare_parameter("camera_settings.resolution", std::vector<int64_t>({640, 480}));
    declare_parameter("camera_settings.exposure",
                      std::vector<int64_t>({4800000, 400000000}));
    declare_parameter("camera_settings.gain", std::vector<int64_t>({1, 16}));
    declare_parameter("camera_settings.ae_lock", false);
    declare_parameter("camera_settings.awb_lock", true);

    declare_parameter("camera_name", "front_left");
    declare_parameter("camera_url",
                      "package://argus_camera_ros/config/calib/front_left_info.yaml");

    declare_parameter("timestamp_mode", "TIME_FROM_TSC");
}

void SingleCameraNode::readParameters(void) {
    using std::vector;
    get_parameter("device_id", deviceID_);
    get_parameter("camera_settings.mode", cameraConfig_.mode);
    get_parameter("camera_settings.fps", cameraConfig_.fps);
    vector<int64_t> resolution =
        get_parameter("camera_settings.resolution").as_integer_array();
    vector<int64_t> exposure =
        get_parameter("camera_settings.exposure").as_integer_array();
    vector<int64_t> gain = get_parameter("camera_settings.gain").as_integer_array();
    get_parameter("camera_settings.ae_lock", cameraConfig_.aeLock);
    get_parameter("camera_settings.awb_lock", cameraConfig_.awbLock);

    if (resolution.size() == 2) {
        cameraConfig_.resolution =
            Size2D<uint32_t>({(uint32_t)resolution[0], (uint32_t)resolution[1]});
    } else {
        RCLCPP_WARN(get_logger(),
                    "camera_settings.resolution MUST be an array of 2 integers, "
                    "[width, height]");
    }

    if (exposure.size() == 2) {
        cameraConfig_.exposure =
            Range<uint64_t>({(uint64_t)exposure[0], (uint64_t)exposure[1]});
    } else {
        RCLCPP_WARN(
            get_logger(),
            "camera_settings.exposure MUST be a range of 2 integers, [min, max]");
    }

    if (gain.size() == 2) {
        cameraConfig_.gain = Range<float>({(float)gain[0], (float)gain[1]});
    } else {
        RCLCPP_WARN(get_logger(),
                    "camera_settings.gain MUST be a range of 2 floats, [min, max]");
    }

    get_parameter("camera_name", cameraName_);
    get_parameter("camera_url", cameraInfoUrl_);
    timestampMode =
        convertStringToTimestampMode(get_parameter("timestamp_mode").as_string());
}

}  // namespace argus_camera_ros
