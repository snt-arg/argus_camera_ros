#ifndef ARGUS_CAMERA_HPP
#define ARGUS_CAMERA_HPP

#include <Argus/Argus.h>
#include <EGLStream/EGLStream.h>
#include <EGLStream/NV/ImageNativeBuffer.h>
#include <NvBufSurface.h>

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <thread>
#include <vector>

#include "argus_camera_ros/logger.hpp"

namespace argus_camera_ros {

using namespace EGLStream;
using namespace Argus;

enum class CameraState {
    NOT_INITIALIZED,
    INITIALIZING,
    READY,
    RUNNING,
    RESTARTING,
    STOPPED,
    ERROR
};

struct CameraConfig {
    int id;
    int mode;
    int width;
    int height;
    float fps;
    // TODO: Sensor settings
    // float exposureTime;  // in seconds
    // float gain;
};

struct ArgusState {
    UniqueObj<CameraProvider> provider;
    UniqueObj<CaptureSession> captureSession;
    UniqueObj<OutputStream> outputStream;
    UniqueObj<Request> request;
};

class ArgusCamera {
   public:
    using FrameCallback = std::function<void(const cv::Mat&)>;

    ArgusCamera(CameraProvider* provider, const CameraConfig& config);
    ArgusCamera(CameraProvider* provider, const CameraConfig& config, Logger& logger);
    ~ArgusCamera();

    // Lifecycle
    bool init();
    bool startCapture();
    void stopCapture();
    bool restartCapture();

    // Accessors
    cv::Mat getLatestFrame();
    void getLatestFrame(cv::Mat& out);
    void setFrameCallback(FrameCallback cb);
    CameraState getState() const;

   private:
    // Configuration
    CameraConfig config_;
    CameraProvider* provider_;

    ArgusState argusState_;
    UniqueObj<FrameConsumer> frameConsumer_;

    // Frame handling
    std::mutex frameMutex_;
    FrameCallback frameCallback_ = nullptr;
    std::thread captureThread_;
    std::atomic<bool> capturing_{false};
    Argus::UniqueObj<Frame> lastFrame_;

    // State
    CameraState state_ = CameraState::NOT_INITIALIZED;
    void setState_(CameraState newState);
    std::string getStateString(CameraState& state);

    // Helpers
    bool setupCamera(CameraDevice* cameraDevice, std::vector<SensorMode*>& sensorModes);
    CameraDevice* getCameraDeviceById(int id);
    std::vector<SensorMode*> getCameraSensorModes(CameraDevice* cameraDevice);
    void captureLoop();
    bool readFrame(cv::Mat& out);
    void printSensorModes(std::vector<SensorMode*>& modes);

    bool createCaptureSession(CameraDevice* cameraDevice);
    bool configureOutputStream();
    bool createCaptureRequest();
    bool configureRequest(const std::vector<SensorMode*>& sensorModes);
    bool initializeFrameConsumer();

    cv::Mat convertFrameToMat(IFrame* iFrame, NvBufSurface* bufSurface, int dmaBufFd);

    // Logging
    Logger logger_;
    std::string loggerPrefix_ = "";
};

}  // namespace argus_camera_ros

#endif  // ARGUS_CAMERA_HPP
