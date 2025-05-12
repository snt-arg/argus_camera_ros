/**
 * @file include/argus_camera_ros/argus_camera.hpp
 * @brief Defines the ArgusCamera class for managing a single camera using NVIDIA's
 * libArgus.
 */

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

/**
 * @enum CameraState
 * @brief Enum representing the internal state of a camera.
 */
enum class CameraState {
    NOT_INITIALIZED,  ///< Camera object constructed but not initialized
    INITIALIZING,     ///< Initialization in progress
    READY,            ///< Ready to start capturing
    RUNNING,          ///< Actively capturing frames
    RESTARTING,       ///< Restarting capture process
    STOPPED,          ///< Stopped manually or via error
    ERROR             ///< An error occurred during operation
};

/**
 * @struct CameraConfig
 * @brief Configuration parameters for the camera sensor.
 */
struct CameraConfig {
    int mode;  ///< Sensor mode index (use printSensorModes() to get available modes)
    int fps;   ///< Desired frames per second
    Size2D<uint32_t> resolution;  ///< Image resolution [width, height]
    Range<uint64_t> exposure;     ///< Exposure time range (in ns) [min, max]
    Range<float> gain;            ///< Gain range [min, max]
    bool aeLock;                  ///< Auto-exposure lock
    bool awbLock;                 ///< Auto-white balance lock
};

/**
 * @struct ArgusState
 * @brief Internal Argus-specific state required for capture.
 */
struct ArgusState {
    UniqueObj<CameraProvider> provider;
    UniqueObj<CaptureSession> captureSession;
    UniqueObj<OutputStream> outputStream;
    UniqueObj<Request> request;
};

/**
 * @struct FrameStamped
 * @brief Holds a raw Argus IFrame with associated SOF and EOF timestamps.
 */
struct FrameStamped {
    IFrame* iFrame;  ///< Raw IFrame pointer from Argus
    uint64_t sofTS;  ///< Start-of-frame timestamp (ns)
    uint64_t eofTS;  ///< End-of-frame timestamp (ns)
};

/**
 * @struct CVFrameStamped
 * @brief Holds an OpenCV frame with associated SOF and EOF timestamps.
 */
struct CVFrameStamped {
    cv::Mat frame;   ///< OpenCV image in BGR8 format
    uint64_t sofTS;  ///< Start-of-frame timestamp (ns)
    uint64_t eofTS;  ///< End-of-frame timestamp (ns)
};

/**
 * @class ArgusCamera
 * @brief Manages a single camera using NVIDIA's libArgus and converts frames to OpenCV
 * format.
 *
 * It offers a few possibilities on how to retrieve OpenCV frames, either getting the
 * last frame captured, or by specifying a hook which is called whenever a new frame is
 * captured.
 *
 * It uses a thread for consuming frames produced by the producer (libArgus +
 * EGLStream).
 */
class ArgusCamera {
   public:
    /**
     * @brief Callback/Hook type for receiving captured frames.
     */
    using FrameCallback = std::function<void(const CVFrameStamped&)>;

    /**
     * @brief Constructor.
     * @param id Sensor ID (e.g., camera index)
     * @param provider Pointer to shared CameraProvider object
     * @param config Camera configuration parameters
     */
    ArgusCamera(int id, CameraProvider* provider, const CameraConfig& config);

    /**
     * @brief Constructor with custom logger.
     * @param id Sensor ID
     * @param provider Pointer to CameraProvider
     * @param config Camera configuration
     * @param logger Custom logger instance
     */
    ArgusCamera(int id,
                CameraProvider* provider,
                const CameraConfig& config,
                Logger& logger);

    /**
     * @brief Destructor. Ensures capture is stopped and threads are joined.
     */
    ~ArgusCamera();

    // --------- Lifecycle ---------

    /**
     * @brief Initializes the camera and prepares it for capture.
     * @return True if initialization succeeds.
     */
    bool init();

    /**
     * @brief Starts the frame capture loop in a background thread.
     * @return True if capture started successfully.
     */
    bool startCapture();

    /**
     * @brief Stops the capture loop and joins the thread.
     */
    void stopCapture();

    /**
     * @brief Attempts to restart the capture loop (e.g., after error).
     * @return True if restart succeeded.
     */
    bool restartCapture();

    // --------- Accessors ---------

    /**
     * @brief Retrieves the latest captured frame.
     * @return The most recent frame (thread-safe copy).
     */
    CVFrameStamped getLatestFrame();

    /**
     * @brief Fills the provided frame object with the latest captured frame.
     * @param out Reference to a CVFrameStamped to fill.
     */
    void getLatestFrame(CVFrameStamped& out);

    /**
     * @brief Returns the current internal state of the camera.
     * @return CameraState enum value.
     */
    CameraState getState() const;

    /**
     * @brief Registers a callback to be invoked each time a new frame is captured.
     * @param cb Function to call with the captured frame.
     */
    void setFrameCallback(FrameCallback cb);

    void printSensorModes(std::vector<SensorMode*>& modes);

   private:
    // Configuration
    int id_;                    ///< Camera index
    CameraConfig config_;       ///< User-provided configuration
    CameraProvider* provider_;  ///< External camera provider (shared)

    ArgusState argusState_;                   ///< Argus objects for session and request
    UniqueObj<FrameConsumer> frameConsumer_;  ///< Frame consumer for reading frames

    // Frame handling
    std::mutex frameMutex_;                  ///< Mutex for thread-safe frame access
    FrameCallback frameCallback_ = nullptr;  ///< Optional frame callback
    std::thread captureThread_;              ///< Capture thread
    std::atomic<bool> capturing_{false};     ///< Whether capture is active
    CVFrameStamped lastStampedFrame_;        ///< Last captured frame

    // State tracking
    CameraState state_ = CameraState::NOT_INITIALIZED;

    /**
     * @brief Sets the internal camera state with logging.
     * @param newState New state to transition to.
     */
    void setState_(CameraState newState);

    /**
     * @brief Converts a CameraState to a string representation.
     * @param state Camera state.
     * @return String version of the state.
     */
    std::string getStateString(CameraState& state);

    // Setup helpers
    CameraDevice* getCameraDeviceById(int id);
    std::vector<SensorMode*> getCameraSensorModes(CameraDevice* cameraDevice);

    /**
     * @brief Main capture thread loop that reads frames and invokes the callback.
     */
    void captureLoop();

    bool setupCamera(CameraDevice* cameraDevice, std::vector<SensorMode*>& sensorModes);
    bool createCaptureSession(CameraDevice* cameraDevice);
    bool configureOutputStream();
    bool createCaptureRequest();
    bool configureRequest(const std::vector<SensorMode*>& sensorModes);
    bool initializeFrameConsumer();

    /**
     * @brief Converts an Argus IFrame into an OpenCV Mat using NvBufSurface.
     * @param iFrame Input Argus frame.
     * @param bufSurface Associated NVMM surface.
     * @return Converted cv::Mat.
     */
    cv::Mat convertFrameToMat(IFrame* iFrame, NvBufSurface* bufSurface);

    // Logging
    Logger logger_;                  ///< Internal logger instance
    std::string loggerPrefix_ = "";  ///< Used to prefix log messages
};

}  // namespace argus_camera_ros

#endif  // ARGUS_CAMERA_HPP
