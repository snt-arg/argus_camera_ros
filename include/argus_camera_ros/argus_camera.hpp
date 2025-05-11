#ifndef ARGUS_CAMERA_HPP
#define ARGUS_CAMERA_HPP

#include <Argus/Argus.h>
#include <EGLStream/EGLStream.h>
#include <EGLStream/NV/ImageNativeBuffer.h>
#include <NvBufSurface.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
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

#define ONE_NS 1e9
#define HZ_TO_MS(hz) (1000 / hz)
#define HZ_TO_NS(hz) (ONE_NS / hz)

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
    int mode;
    int fps;
    Size2D<uint32_t> resolution;
    Range<uint64_t> exposure;
    Range<float> gain;
    bool aeLock;
    bool awbLock;
};

struct ArgusState {
    UniqueObj<CameraProvider> provider;
    UniqueObj<CaptureSession> captureSession;
    UniqueObj<OutputStream> outputStream;
    UniqueObj<Request> request;
};

struct FrameStamped {
    IFrame* iFrame;
    uint64_t sofTS;
    uint64_t eofTS;
};

struct CVFrameStamped {
    cv::Mat frame;
    uint64_t sofTS;
    uint64_t eofTS;
};

class ArgusCamera {
   public:
    using FrameCallback = std::function<void(const CVFrameStamped&)>;

    ArgusCamera(int id, CameraProvider* provider, const CameraConfig& config);
    ArgusCamera(int id,
                CameraProvider* provider,
                const CameraConfig& config,
                Logger& logger);
    ~ArgusCamera();

    // Lifecycle
    bool init();
    bool startCapture();
    void stopCapture();
    bool restartCapture();

    // Accessors
    CVFrameStamped getLatestFrame();
    void getLatestFrame(CVFrameStamped& out);
    void setFrameCallback(FrameCallback cb);
    CameraState getState() const;

   private:
    // Configuration
    int id_;
    CameraConfig config_;
    CameraProvider* provider_;

    ArgusState argusState_;
    UniqueObj<FrameConsumer> frameConsumer_;

    // Frame handling
    std::mutex frameMutex_;
    FrameCallback frameCallback_ = nullptr;
    std::thread captureThread_;
    std::atomic<bool> capturing_{false};
    CVFrameStamped lastStampedFrame_;

    // State
    CameraState state_ = CameraState::NOT_INITIALIZED;
    void setState_(CameraState newState);
    std::string getStateString(CameraState& state);

    // Helpers
    CameraDevice* getCameraDeviceById(int id);
    std::vector<SensorMode*> getCameraSensorModes(CameraDevice* cameraDevice);
    void captureLoop();
    void printSensorModes(std::vector<SensorMode*>& modes);

    bool setupCamera(CameraDevice* cameraDevice, std::vector<SensorMode*>& sensorModes);
    bool createCaptureSession(CameraDevice* cameraDevice);
    bool configureOutputStream();
    bool createCaptureRequest();
    bool configureRequest(const std::vector<SensorMode*>& sensorModes);
    bool initializeFrameConsumer();

    cv::Mat convertFrameToMat(IFrame* iFrame, NvBufSurface* bufSurface);
    bool readFrame(cv::Mat& out);

    // Logging
    Logger logger_;
    std::string loggerPrefix_ = "";
};

}  // namespace argus_camera_ros

#endif  // ARGUS_CAMERA_HPP
