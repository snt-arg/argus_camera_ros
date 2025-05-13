#ifndef PTP_CONVERTER_HPP
#define PTP_CONVERTER_HPP

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>

#include "nvidia/nvpps_ioctl.h"

namespace argus_camera_ros {

/**
 * @brief Utility class for converting Tegra Wide timestamps (TSC) to PTP-synchronized
 * system time.
 *
 * Uses the NvPPS module exposed via /dev/nvpps0 to perform precise timestamp
 * correlation.
 */
class PTPConverter {
   public:
    /**
     * @brief Constructs a PTP converter using the given NvPPS device.
     * @param device Path to the NvPPS device (default: /dev/nvpps0)
     * @throws std::runtime_error if the device cannot be opened.
     */
    explicit PTPConverter(const std::string& device = "/dev/nvpps0") {
        fd_ = open(device.c_str(), O_RDWR);
        if (fd_ < 0) {
            throw std::runtime_error("Failed to open NvPPS device: " + device);
        }
    }

    /**
     * @brief Closes the NvPPS device.
     */
    ~PTPConverter() {
        if (fd_ >= 0) close(fd_);
    }

    /**
     * @brief Converts a given TSC timestamp to PTP time.
     * @param tsc_ts Timestamp in the TSC domain (e.g., sofTS or eofTS)
     * @return Corresponding PTP time in nanoseconds.
     */
    uint64_t convertTSCTimeToPTP(uint64_t tsc_ts) {
        struct nvpps_timeevent timestamp{};
        if (ioctl(fd_, NVPPS_GETEVENT, &timestamp) < 0) {
            throw std::runtime_error("NVPPS_GETEVENT ioctl failed");
        }

        int64_t delta =
            static_cast<int64_t>(tsc_ts) - static_cast<int64_t>(timestamp.tsc);

        return timestamp.ptp + delta;
    }

   private:
    int fd_;
};

}  // namespace argus_camera_ros

#endif  // PTP_CONVERTER_HPP
