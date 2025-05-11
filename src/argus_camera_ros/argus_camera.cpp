#include "argus_camera_ros/argus_camera.hpp"

#include <Argus/Ext/SensorTimestampTsc.h>
#include <tegra.h>

#include <algorithm>
#include <cstdlib>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include <vector>

namespace argus_camera_ros {

ArgusCamera::ArgusCamera(int id, CameraProvider *provider, const CameraConfig &config)
    : id_(id), config_(config) {
    loggerPrefix_ = "[Camera(" + std::to_string(id_) + ")] ";
    logger_.info("ArgusCamera instance created", loggerPrefix_);
    if (provider == nullptr) {
        logger_.error("Received CameraProvider is a nullptr", loggerPrefix_);
        setState_(CameraState::ERROR);
        return;
    }
    argusState_.provider.reset(provider);
}

ArgusCamera::ArgusCamera(int id,
                         CameraProvider *provider,
                         const CameraConfig &config,
                         Logger &logger)
    : id_(id), config_(config), logger_(logger) {
    loggerPrefix_ = "[Camera(" + std::to_string(id_) + ")] ";
    logger_.info("Creating ArgusCamera instance", loggerPrefix_);

    if (provider == nullptr) {
        logger_.error("Received CameraProvider is a nullptr", loggerPrefix_);
        setState_(CameraState::ERROR);
        return;
    }
    argusState_.provider.reset(provider);
}

ArgusCamera::~ArgusCamera() {
    logger_.info("Destroying ArgusCamera instance", loggerPrefix_);

    stopCapture();
}

bool ArgusCamera::init(void) {
    logger_.info("Initializing Argus Camera", loggerPrefix_);
    setState_(CameraState::INITIALIZING);

    CameraDevice *cameraDevice = getCameraDeviceById(id_);
    if (cameraDevice == nullptr) return false;

    std::vector<SensorMode *> sensorModes = getCameraSensorModes(cameraDevice);
    if (sensorModes.empty()) return false;

    // Print available sensor modes
    // printSensorModes(sensorModes);

    if (!setupCamera(cameraDevice, sensorModes)) return false;

    setState_(CameraState::READY);

    return true;
}

bool ArgusCamera::startCapture(void) {
    logger_.info("Starting Capturing", loggerPrefix_);

    if (state_ != CameraState::READY) {
        logger_.error(
            "Camera is not READY to start capture. Make sure you have called init()",
            loggerPrefix_);
        return false;
    }

    ICaptureSession *iCaptureSession =
        interface_cast<ICaptureSession>(argusState_.captureSession);
    iCaptureSession->repeat(argusState_.request.get());

    capturing_ = true;
    captureThread_ = std::thread(&ArgusCamera::captureLoop, this);
    setState_(CameraState::RUNNING);

    return true;
}

void ArgusCamera::stopCapture(void) {
    logger_.info("Stopping Capturing", loggerPrefix_);
    if (capturing_) {
        capturing_ = false;
        if (captureThread_.joinable()) {
            captureThread_.join();
        }
    }

    ICaptureSession *iCaptureSession =
        interface_cast<ICaptureSession>(argusState_.captureSession);
    iCaptureSession->stopRepeat();
    iCaptureSession->waitForIdle(1000000000);

    setState_(CameraState::STOPPED);
}

bool ArgusCamera::setupCamera(CameraDevice *cameraDevice,
                              std::vector<SensorMode *> &sensorModes) {
    if (!createCaptureSession(cameraDevice)) return false;
    if (!configureOutputStream()) return false;
    if (!createCaptureRequest()) return false;
    if (!configureRequest(sensorModes)) return false;
    if (!initializeFrameConsumer()) return false;

    return true;
}

void ArgusCamera::captureLoop() {
    IEGLOutputStream *eglOutputStream =
        interface_cast<IEGLOutputStream>(argusState_.outputStream);
    IFrameConsumer *consumer = interface_cast<IFrameConsumer>(frameConsumer_);

    if (!consumer ||
        eglOutputStream->waitUntilConnected(1 * ONE_NS) != Argus::STATUS_OK) {
        logger_.error("Stream or Consumer setup failed.", loggerPrefix_);
        capturing_ = false;
        setState_(CameraState::ERROR);
        return;
    }

    logger_.info("Argus Stream Capture is now ready", loggerPrefix_);

    int dmaBufFd = -1;
    NvBufSurface *bufSurface = new NvBufSurface();

    while (capturing_) {
        Argus::Status status;
        UniqueObj<Frame> frame(consumer->acquireFrame(1000000000, &status));
        if (status == Argus::STATUS_TIMEOUT) {
            logger_.warn("Timeout reached after waiting for new camera frame",
                         loggerPrefix_);
            continue;
        }

        IFrame *iFrame = interface_cast<IFrame>(frame);
        if (!iFrame) continue;

        CaptureMetadata *captureMetadataLeft =
            interface_cast<IArgusCaptureMetadata>(frame)->getMetadata();
        ICaptureMetadata *iMetadataLeft =
            interface_cast<ICaptureMetadata>(captureMetadataLeft);
        if (captureMetadataLeft == nullptr || iMetadataLeft == nullptr) {
            throw std::runtime_error(
                "Sensor Capture Metadata has not been enabled or is not supported");
        }

        Ext::ISensorTimestampTsc *iSensorTS =
            interface_cast<Ext::ISensorTimestampTsc>(captureMetadataLeft);

        NV::IImageNativeBuffer *iNativeBuffer =
            interface_cast<NV::IImageNativeBuffer>(iFrame->getImage());

        if (!iNativeBuffer)
            throw std::runtime_error("IImageNativeBuffer not supported.");

        if (dmaBufFd == -1) {
            dmaBufFd = iNativeBuffer->createNvBuffer(eglOutputStream->getResolution(),
                                                     NVBUF_COLOR_FORMAT_RGBA,
                                                     NVBUF_LAYOUT_PITCH);
            if (NvBufSurfaceFromFd(dmaBufFd, (void **)(&bufSurface)) != 0)
                throw std::runtime_error("Failed to get NvBufSurface.");
        } else if (iNativeBuffer->copyToNvBuffer(dmaBufFd) != STATUS_OK) {
            throw std::runtime_error("Failed to copy to NvBuffer.");
        }

        cv::Mat matImage = convertFrameToMat(iFrame, bufSurface);
        std::unique_lock<std::mutex> lock(frameMutex_);
        lastStampedFrame_.frame = std::move(matImage);
        lastStampedFrame_.sofTS = iSensorTS->getSensorSofTimestampTsc();
        lastStampedFrame_.eofTS = iSensorTS->getSensorEofTimestampTsc();
        lock.unlock();

        if (frameCallback_) {
            try {
                frameCallback_(
                    lastStampedFrame_);  // Call the hook with converted OpenCV frame
            } catch (const std::exception &e) {
                logger_.warn(std::string("Frame conversion failed: ") + e.what(),
                             loggerPrefix_);
            }
        }
    }

    delete bufSurface;
    if (dmaBufFd != -1) close(dmaBufFd);
}

cv::Mat ArgusCamera::convertFrameToMat(IFrame *iFrame, NvBufSurface *bufSurface) {
    NvBufSurfaceMap(bufSurface, -1, 0, NVBUF_MAP_READ);
    NvBufSurfaceSyncForCpu(bufSurface, -1, 0);

    cv::Mat rgba(config_.resolution.height(),
                 config_.resolution.width(),
                 CV_8UC4,
                 bufSurface->surfaceList->mappedAddr.addr[0]);
    cv::Mat bgr;
    cv::cvtColor(rgba, bgr, cv::COLOR_RGBA2BGR);

    NvBufSurfaceUnMap(bufSurface, -1, 0);
    return bgr;
}

bool ArgusCamera::createCaptureSession(CameraDevice *cameraDevice) {
    ICameraProvider *iCameraProvider =
        interface_cast<ICameraProvider>(argusState_.provider);
    argusState_.captureSession.reset(
        iCameraProvider->createCaptureSession(cameraDevice));
    if (!argusState_.captureSession) {
        logger_.error("Failed to create the CaptureSession", loggerPrefix_);
        setState_(CameraState::ERROR);
        return false;
    }

    if (!interface_cast<ICaptureSession>(argusState_.captureSession) ||
        !interface_cast<IEventProvider>(argusState_.captureSession)) {
        logger_.error("Failed to create ICaptureSession/IEventProvider", loggerPrefix_);
        setState_(CameraState::ERROR);
        return false;
    }

    return true;
}

bool ArgusCamera::configureOutputStream() {
    ICaptureSession *iCaptureSession =
        interface_cast<ICaptureSession>(argusState_.captureSession);

    OutputStreamSettings *streamSettings =
        iCaptureSession->createOutputStreamSettings(STREAM_TYPE_EGL);
    IEGLOutputStreamSettings *iEGLStreamSettings =
        interface_cast<IEGLOutputStreamSettings>(streamSettings);

    if (!streamSettings || !iEGLStreamSettings) {
        logger_.error("Failed to create OutputStreamSettings", loggerPrefix_);
        setState_(CameraState::ERROR);
        return false;
    }

    iEGLStreamSettings->setPixelFormat(PIXEL_FMT_YCbCr_420_888);
    iEGLStreamSettings->setResolution(config_.resolution);
    iEGLStreamSettings->setMode(EGL_STREAM_MODE_FIFO);
    iEGLStreamSettings->setFifoLength(4);
    iEGLStreamSettings->setMetadataEnable(true);

    argusState_.outputStream.reset(iCaptureSession->createOutputStream(streamSettings));
    if (!argusState_.outputStream) {
        logger_.error("Failed to create the OutputStream", loggerPrefix_);
        setState_(CameraState::ERROR);
        return false;
    }

    return true;
}

bool ArgusCamera::createCaptureRequest() {
    ICaptureSession *iCaptureSession =
        interface_cast<ICaptureSession>(argusState_.captureSession);

    argusState_.request.reset(iCaptureSession->createRequest());
    IRequest *iRequest = interface_cast<IRequest>(argusState_.request);

    if (!argusState_.request || !iRequest) {
        logger_.error("Failed to create the iRequest", loggerPrefix_);
        setState_(CameraState::ERROR);
        return false;
    }

    iRequest->enableOutputStream(argusState_.outputStream.get());
    return true;
}

bool ArgusCamera::configureRequest(const std::vector<SensorMode *> &sensorModes) {
    IRequest *iRequest = interface_cast<IRequest>(argusState_.request);
    ISourceSettings *iSourceSettings =
        interface_cast<ISourceSettings>(iRequest->getSourceSettings());
    IAutoControlSettings *iAutoControlSettings =
        interface_cast<IAutoControlSettings>(iRequest->getAutoControlSettings());

    if (!iSourceSettings || !iAutoControlSettings) {
        logger_.error("Failed to get ISourceSettings/IAutoControlSettings",
                      loggerPrefix_);
        setState_(CameraState::ERROR);
        return false;
    }

    iSourceSettings->setSensorMode(sensorModes[config_.mode]);
    iSourceSettings->setFrameDurationRange(Range<uint64_t>(HZ_TO_NS(config_.fps)));
    iSourceSettings->setExposureTimeRange(config_.exposure);
    iSourceSettings->setGainRange(config_.gain);
    // iSourceSettings->setApertureFNumber();
    // iSourceSettings->setAperturePosition();

    iAutoControlSettings->setAeLock(config_.aeLock);
    iAutoControlSettings->setAwbLock(config_.awbLock);

    return true;
}

bool ArgusCamera::initializeFrameConsumer() {
    frameConsumer_.reset(FrameConsumer::create(argusState_.outputStream.get()));
    if (!frameConsumer_) {
        logger_.error("Failed to create FrameConsumer", loggerPrefix_);
        setState_(CameraState::ERROR);
        return false;
    }
    return true;
}

CVFrameStamped ArgusCamera::getLatestFrame() {
    std::lock_guard<std::mutex> lock(frameMutex_);
    return lastStampedFrame_;
}
void ArgusCamera::getLatestFrame(CVFrameStamped &out) {
    std::lock_guard<std::mutex> lock(frameMutex_);
    out = lastStampedFrame_;
}

CameraState ArgusCamera::getState() const { return state_; }

std::vector<SensorMode *> ArgusCamera::getCameraSensorModes(
    CameraDevice *cameraDevice) {
    std::vector<SensorMode *> modes;
    ICameraProperties *iCameraProperties =
        interface_cast<ICameraProperties>(cameraDevice);

    if (iCameraProperties == nullptr) {
        logger_.error("Failed to obtain the ICameraProperties", loggerPrefix_);
        setState_(CameraState::ERROR);
        return modes;
    }

    iCameraProperties->getAllSensorModes(&modes);

    if (modes.empty()) {
        logger_.error("Failed to get sensor modes", loggerPrefix_);
        setState_(CameraState::ERROR);
        return modes;
    }
    return modes;
}

CameraDevice *ArgusCamera::getCameraDeviceById(int id) {
    std::vector<CameraDevice *> devices;
    ICameraProvider *iCameraProvider =
        interface_cast<ICameraProvider>(argusState_.provider);

    if (iCameraProvider == nullptr) {
        logger_.error("Failed to obtain the ICameraProvider", loggerPrefix_);
        setState_(CameraState::ERROR);
        return nullptr;
    }

    iCameraProvider->getCameraDevices(&devices);
    if (devices.empty()) {
        logger_.error("Failed to get camera devices ", loggerPrefix_);
        setState_(CameraState::ERROR);
        return nullptr;
    }

    return devices[id];
}

std::string ArgusCamera::getStateString(CameraState &state) {
    switch (state) {
        case CameraState::NOT_INITIALIZED:
            return "NOT_INITIALIZED";
        case CameraState::INITIALIZING:
            return "INITIALIZING";
        case CameraState::READY:
            return "READY";
        case CameraState::STOPPED:
            return "STOPPED";
        case CameraState::RUNNING:
            return "RUNNING";
        case CameraState::RESTARTING:
            return "RESTARTING";
        case CameraState::ERROR:
            return "ERROR";
        default:
            return "UNKNOWN";
    }
}

void ArgusCamera::setState_(CameraState newState) {
    if (newState != state_) {
        std::ostringstream oss;
        oss << "State Transition : " << getStateString(state_) << " --> "
            << getStateString(newState);
        logger_.info(oss.str(), loggerPrefix_);

        state_ = newState;
    }
}

void ArgusCamera::setFrameCallback(FrameCallback cb) { frameCallback_ = std::move(cb); }

void ArgusCamera::printSensorModes(std::vector<SensorMode *> &modes) {
    size_t index = 0;
    std::cout << "Camera " << id_ << " Sensor Modes:\n";
    for (auto mode : modes) {
        ISensorMode *iMode = interface_cast<ISensorMode>(mode);

        std::cout << "  Mode " << index << ":\n";
        std::cout << "    Resolution: (" << iMode->getResolution()[0] << ", "
                  << iMode->getResolution()[1] << ")\n";
        std::cout << "    Sensor Type: " << iMode->getSensorModeType().getName()
                  << "\n";
        std::cout << "    Gain Range: (" << iMode->getAnalogGainRange()[0] << ", "
                  << iMode->getAnalogGainRange()[1] << ")\n";
        std::cout << "    Exposure Range: (" << iMode->getExposureTimeRange()[0] << ", "
                  << iMode->getExposureTimeRange()[1] << ")\n";
        std::cout << "    Frame Duration Range: (" << iMode->getFrameDurationRange()[0]
                  << ", " << iMode->getFrameDurationRange()[1] << ")\n";
        std::cout << "    HDR Ratio Range: (" << iMode->getHdrRatioRange()[0] << ", "
                  << iMode->getHdrRatioRange()[1] << ")\n";
        std::cout << "    Input Bit Depth: " << iMode->getInputBitDepth() << "\n";
        std::cout << "    Output Bit Depth: " << iMode->getOutputBitDepth() << "\n";
        std::cout << "    Bayer Phase: " << iMode->getBayerPhase().getName() << "\n";
        std::cout << "\n";

        index++;
    }
}

}  // namespace argus_camera_ros
