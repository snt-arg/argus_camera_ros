#include "argus_camera_ros/argus_camera.hpp"

#include <cstdlib>
#include <opencv2/core/mat.hpp>
#include <vector>

namespace argus_camera_ros {

ArgusCamera::ArgusCamera(UniqueObj<CameraProvider> &provider) {
    ICameraProvider_ = interface_cast<ICameraProvider>(provider);

    if (!ASSERT_NOT_NULL(ICameraProvider_,
                         "Failed to create Camera Provider Interface")) {
        currentState = FAILURE;
    }

    // currentConfig = config;
}

ArgusCamera::~ArgusCamera() { stopCapture(); }

bool ArgusCamera::init(void) {
    currentState = INITIALIZING;

    CameraDevice *cameraDevice = getCameraDeviceByID(0);
    setupArgusProducer(cameraDevice);
    currentState = READY;
    return true;
}

void ArgusCamera::startCapture(void) {
    if (currentState != READY) {
        std::cerr << "Unable to start capture. Current state is "
                  << getCameraStateAsString() << "!= READY" << std::endl;
    }

    isCapturing = true;

    consumerThread = std::thread(&ArgusCamera::consumerThreadCallback, this);
    ICaptureSession *iCaptureSession =
        interface_cast<ICaptureSession>(caputureSession_);
    iCaptureSession->repeat(request_.get());
}

void ArgusCamera::stopCapture(void) {
    isCapturing = false;
    ICaptureSession *iCaptureSession =
        interface_cast<ICaptureSession>(caputureSession_);
    iCaptureSession->stopRepeat();
    consumerThread.join();
}

std::vector<SensorMode *> ArgusCamera::getCameraSensorModes(
    CameraDevice *cameraDevice) {
    std::vector<SensorMode *> modes;
    ICameraProperties *properties = interface_cast<ICameraProperties>(cameraDevice);

    properties->getAllSensorModes(&modes);
    return modes;
}

void ArgusCamera::setupArgusProducer(CameraDevice *cameraDevice) {
    std::vector<SensorMode *> sensor_modes = getCameraSensorModes(cameraDevice);
    caputureSession_.reset(ICameraProvider_->createCaptureSession(cameraDevice));

    ICaptureSession *capture_session_i =
        interface_cast<ICaptureSession>(caputureSession_);

    IEventProvider *event_provider_i = interface_cast<IEventProvider>(caputureSession_);
    if (!event_provider_i || !event_provider_i) {
        std::cerr << "[ERROR] Failed to create CaptureSession" << std::endl;
    }

    UniqueObj<OutputStreamSettings> stream_settings(
        capture_session_i->createOutputStreamSettings(STREAM_TYPE_EGL));
    IEGLOutputStreamSettings *egl_stream_settings_i =
        interface_cast<IEGLOutputStreamSettings>(stream_settings);
    if (!egl_stream_settings_i) {
        std::cerr << "[ERROR] Failed to create EglOutputStreamSettings" << std::endl;
    }

    egl_stream_settings_i->setPixelFormat(PIXEL_FMT_YCbCr_420_888);
    egl_stream_settings_i->setResolution(Size2D<uint32_t>(640, 480));

    outputStream_.reset(capture_session_i->createOutputStream(stream_settings.get()));

    /* Create capture request and enable the output stream */
    request_.reset(capture_session_i->createRequest());
    IRequest *iRequest = interface_cast<IRequest>(request_);
    if (!iRequest) {
        std::cerr << "[ERROR] Failed to create Request" << std::endl;
    }
    iRequest->enableOutputStream(outputStream_.get());

    ISourceSettings *iSourceSettings =
        interface_cast<ISourceSettings>(iRequest->getSourceSettings());
    if (!iSourceSettings) {
        std::cerr << "[ERROR] Failed to get ISourceSettings interface" << std::endl;
    }

    // IAutoControlSettings *control_settings =
    // interface_cast<IAutoControlSettings>(iRequest->getAutoControlSettings());

    iSourceSettings->setSensorMode(sensor_modes[2]);
    iSourceSettings->setFrameDurationRange(Range<uint64_t>(1e9 / 30));
    FrameConsumer_.reset(FrameConsumer::create(outputStream_.get()));
}

void ArgusCamera::consumerThreadCallback(void) {
    IEGLOutputStream *egl_output_stream =
        interface_cast<IEGLOutputStream>(outputStream_);
    IFrameConsumer *frame_consumer = interface_cast<IFrameConsumer>(FrameConsumer_);
    if (!frame_consumer) {
        std::cerr << "[ERROR] Failed to get IFrameConsumer Interface" << std::endl;
        return;
    }

    if (egl_output_stream->waitUntilConnected(1 * 100000000) != Argus::STATUS_OK) {
        std::cerr << "[ERROR] Stream failed to connect." << std::endl;
        return;
    }

    std::cout << "[INFO] Argus Stream Capture is now ready" << std::endl;

    int dma_buf;
    NvBufSurface *buf_surface = new NvBufSurface();
    Size2D<uint32_t> resolution(640, 480);
    bool failed_aquire = false;

    while (isCapturing) {
        failed_aquire = false;
        // Acquire frames
        Argus::Status status;
        UniqueObj<Frame> frame(frame_consumer->acquireFrame(1000000000, &status));

        if (status == Argus::STATUS_TIMEOUT) {
            failed_aquire = true;
            std::cout << "[WARN] Failed to aquire frame after timeout" << std::endl;
            break;
        }

        IFrame *iFrame = interface_cast<IFrame>(frame);
        if (!iFrame) {
            failed_aquire = true;
            std::cout << "[Warn] Failed to get iframe: " << std::endl;
            break;
        }

        NV::IImageNativeBuffer *iNativeBuffer =
            interface_cast<NV::IImageNativeBuffer>(iFrame->getImage());
        if (!iNativeBuffer) {
            delete buf_surface;
            std::cerr << "[ERROR] IImageNativeBuffer not supported by Image."
                      << std::endl;
            return;
        }

        if (!dma_buf) {
            dma_buf = iNativeBuffer->createNvBuffer(egl_output_stream->getResolution(),
                                                    NVBUF_COLOR_FORMAT_RGBA,
                                                    NVBUF_LAYOUT_PITCH);

            if (-1 == NvBufSurfaceFromFd(dma_buf, (void **)(&buf_surface))) {
                delete buf_surface;
                std::cerr << "[ERROR] Cannot get NvBufSurface from fd" << std::endl;
                return;
            }
        } else if (iNativeBuffer->copyToNvBuffer(dma_buf) != STATUS_OK) {
            delete buf_surface;
            std::cerr << "[ERROR] Cannot get NvBufSurface from fd" << std::endl;
            return;
        }
        if (!failed_aquire) {
            std::vector<cv::Mat> processed_frame;
            // Map and sync each buffer for CPU access
            NvBufSurfaceMap(buf_surface, -1, 0, NVBUF_MAP_READ);
            NvBufSurfaceSyncForCpu(buf_surface, -1, 0);

            // Convert the buffer to a cv::Mat
            cv::Mat imgbuf(
                480, 640, CV_8UC4, buf_surface->surfaceList->mappedAddr.addr[0]);

            // Convert from RGBA to BGR
            cv::Mat display_img;
            cvtColor(imgbuf, display_img, cv::COLOR_RGBA2BGR);

            std::lock_guard<std::mutex> lock(
                bufferMutex);  // Automatically locks and unlocks
            buffer_.push(display_img);

            // Unmap the buffer
            NvBufSurfaceUnMap(buf_surface, -1, 0);
        }
    }
}

bool ArgusCamera::getLatestFrame(cv::Mat &frame) {
    std::lock_guard<std::mutex> lock(bufferMutex);  // Automatically locks and unlocks
    if (buffer_.empty()) {
        return false;  // No frame available
    }

    // Copy or move the frame from the queue
    frame = buffer_.front();  // This is a shallow copy, sharing reference to the image
    buffer_.pop();            // Remove the frame from the queue

    return true;
}

ArgusCamera::CameraState ArgusCamera::getCameraState() { return currentState; }

CameraDevice *ArgusCamera::getCameraDeviceByID(int id) {
    std::vector<CameraDevice *> devices;
    ICameraProvider_->getCameraDevices(&devices);
    if (devices.empty()) {
        std::cerr << "[ERROR] Failed to get devices" << std::endl;
    }

    return devices[id];
}

std::string ArgusCamera::getCameraStateAsString() {
    switch (currentState) {
        case NOT_INITIALIZED:
            return "NOT_INITIALIZED";
        case INITIALIZING:
            return "INITIALIZING";
        case READY:
            return "READY";
        case INITIALIZING_CAPTURE:
            return "INITIALIZING_CAPTURE";
        case CAPTURING:
            return "CAPTURING";
        case RESTARTING:
            return "RESTARTING";
        case FAILURE:
            return "FAILURE";
        default:
            return "UNKNOWN";
    }
}

}  // namespace argus_camera_ros
