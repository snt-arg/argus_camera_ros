#ifndef ARGUS_CAMERA_HPP
#define ARGUS_CAMERA_HPP

#include <Argus/Argus.h>
#include <EGLStream/EGLStream.h>
#include <EGLStream/NV/ImageNativeBuffer.h>
#include <NvBufSurface.h>
#include <opencv2/core/hal/interface.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <queue>
#include <thread>
#include <vector>

namespace argus_camera_ros {

using namespace EGLStream;
using namespace Argus;

template <typename T>
bool assert_not_null(T *ptr, const char *message) {
    if (ptr == nullptr) {
        std::cerr << "Assertion failed: " << message << " (null pointer detected)"
                  << std::endl;
        return false;
    }
    return true;
}

#define ASSERT_NOT_NULL(ptr, message) assert_not_null((ptr), (message))

class ArgusCamera {
    struct CameraConfig {
        int64 cameraID;   ///< ID of the camera as listed in /dev/videox
        int sensorMode;   ///< Sensor mode for image capture (0-3).
        int imageWidth;   ///< Target width for resized images.
        int imageHeight;  ///< Target height for resized images.
        int framerate;    ///< Framerate at which images are captured.
        // TODO: Missing the exposure, and other settings of each camera sensor
    };

    enum CameraState {
        NOT_INITIALIZED = 0,
        INITIALIZING = 1,
        READY = 2,
        INITIALIZING_CAPTURE = 3,
        CAPTURING = 4,
        RESTARTING = 5,
        FAILURE = 6,
    };

   public:
    ArgusCamera(UniqueObj<CameraProvider> &provider);
    ~ArgusCamera(void);

    bool init(void);
    void startCapture(void);
    void stopCapture(void);

    bool getLatestFrame(cv::Mat &frame);
    // cv::Mat getLatestFrame(void);
    CameraState getCameraState(void);

    std::string getCameraStateAsString();

   private:
    ICameraProvider *ICameraProvider_;
    UniqueObj<CaptureSession> caputureSession_;
    UniqueObj<OutputStream> outputStream_;
    UniqueObj<FrameConsumer> FrameConsumer_;
    UniqueObj<Request> request_;

    CameraDevice *getCameraDeviceByID(int id);
    std::vector<SensorMode *> getCameraSensorModes(CameraDevice *cameraDevice);

    void setupArgusProducer(CameraDevice *cameraDevice);
    void setupCaptureConsumer(void);

    void consumerThreadCallback(void);

   private:
    CameraState currentState;
    CameraConfig currentConfig;
    std::thread consumerThread;
    std::atomic_bool isCapturing;
    std::mutex bufferMutex;

    std::queue<cv::Mat> buffer_;
};
}  // namespace argus_camera_ros

#endif  // ARGUS_CAMERA_HPP
