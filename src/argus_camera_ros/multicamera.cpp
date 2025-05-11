#include "argus_camera_ros/multicamera.hpp"

#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui.hpp>

#include "argus_camera_ros/argus_camera.hpp"

namespace argus_camera_ros {

MultiCameraNode::MultiCameraNode()
    : Node("multicamera_node"),
      nodeHandle_(std::shared_ptr<MultiCameraNode>(this, [](auto*) {})) {
    setupLogger();

    declareParameters();
    readParameters();

    initCameras();
    initImageTransport();
}

void MultiCameraNode::startCapturing(void) {
    for (auto camera : cameras_) {
        camera->startCapture();
    }

    // pubTimer_ = pubTimer_ =
    //     create_wall_timer(std::chrono::milliseconds(HZ_TO_MS(cameraConfig_.fps)),
    //                       std::bind(&MultiCameraNode::publishImageCallback, this));
}

void MultiCameraNode::initCameras(void) {
    cameraProvider_ = UniqueObj<CameraProvider>(CameraProvider::create());

    for (int i = 0; i < NUM_CAMS; i++) {
        cameras_.push_back(
            std::make_shared<ArgusCamera>(i, cameraProvider_.get(), cameraConfig_));
    }

    for (int i = 0; i < cameras_.size(); i++) {
        auto camera = cameras_[i];
        if (!camera->init()) {
            throw std::runtime_error("Failed to initilize a camera");
        }
        camera->setFrameCallback([i, this](const CVFrameStamped& stampedFrame) {
            publishImage(i, stampedFrame);
        });
    }
}

void MultiCameraNode::initImageTransport(void) {
    it_ = std::make_shared<image_transport::ImageTransport>(nodeHandle_);

    for (int i = 0; i < cameras_.size(); i++) {
        std::string topicName = "/camera/" + cameraNames_[i] + "/image_raw";

        imgPubs_.push_back(it_->advertise(topicName, 10));

        cameraInfoPubs_.push_back(create_publisher<sensor_msgs::msg::CameraInfo>(
            "/camera/" + cameraNames_[i] + "/camera_info", 10));

        cameraInfos_.push_back(std::make_unique<camera_info_manager::CameraInfoManager>(
            this, cameraNames_[i], cameraInfoUrls_[i]));
    }
}

void MultiCameraNode::publishImageCallback(void) {
    std::vector<CVFrameStamped> stampedFrames;
    for (auto camera : cameras_) {
        stampedFrames.push_back(camera->getLatestFrame());
    }

    for (int i = 0; i < cameras_.size(); i++) {
        publishImage(i, stampedFrames[i]);
    }
}

void MultiCameraNode::publishImage(const int cameraIdx,
                                   const CVFrameStamped& stampedFrame) {
    std_msgs::msg::Header header;
    // TODO: add a param to switch between ros time and HW time
    header.stamp = rclcpp::Time(stampedFrame.sofTS);
    header.frame_id = cameraNames_[cameraIdx];

    if (stampedFrame.frame.empty()) {
        RCLCPP_DEBUG(get_logger(), "EMPTY");
        return;
    }

    sensor_msgs::msg::Image::SharedPtr msg =
        cv_bridge::CvImage(header, "bgr8", stampedFrame.frame).toImageMsg();

    imgPubs_[cameraIdx].publish(msg);
    publishCameraInfo(cameraIdx, header);
}

void MultiCameraNode::publishCameraInfo(const int cameraIdx,
                                        const std_msgs::msg::Header& header) {
    sensor_msgs::msg::CameraInfo camera_info = cameraInfos_[cameraIdx]->getCameraInfo();
    camera_info.header = header;
    cameraInfoPubs_[cameraIdx]->publish(camera_info);
}

void MultiCameraNode::setupLogger(void) {
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

void MultiCameraNode::declareParameters(void) {
    declare_parameter("camera_settings.mode", 2);
    declare_parameter("camera_settings.fps", 30);
    declare_parameter("camera_settings.resolution", std::vector<int64_t>({640, 480}));
    declare_parameter("camera_settings.exposure",
                      std::vector<int64_t>({4800000, 400000000}));
    declare_parameter("camera_settings.gain", std::vector<int64_t>({1, 16}));
    declare_parameter("camera_settings.ae_lock", false);
    declare_parameter("camera_settings.awb_lock", true);

    declare_parameter(
        "camera_names",
        std::vector<std::string>({"front_left", "front_right", "right", "left"}));
    declare_parameter("camera_urls",
                      std::vector<std::string>(
                          {"package://argus_camera_ros/config/front_left_info.yaml",
                           "package://argus_camera_ros/config/front_right_info.yaml",
                           "package://argus_camera_ros/config/right_info.yaml",
                           "package://argus_camera_ros/config/left_info.yaml"}));
}

void MultiCameraNode::readParameters(void) {
    using std::vector;
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

    get_parameter("camera_names", cameraNames_);
    get_parameter("camera_urls", cameraInfoUrls_);
}

}  // namespace argus_camera_ros
