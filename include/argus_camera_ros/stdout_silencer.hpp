/**
 * @file include/argus_camera_ros/stdout_silencer.hpp
 * @brief Defines a hack to remove from stdout low-level related warnings coming from
 * libArgus.
 */

#ifndef STDOUT_SILENCER_HPP
#define STDOUT_SILENCER_HPP
#include <fcntl.h>
#include <unistd.h>

#include <cstdio>

class StdoutSilencer {
   public:
    StdoutSilencer() = default;

    void enable() {
        if (enabled_) return;
        devNull_ = open("/dev/null", O_WRONLY);
        stdoutCopy_ = dup(STDOUT_FILENO);
        stderrCopy_ = dup(STDERR_FILENO);
        dup2(devNull_, STDOUT_FILENO);
        dup2(devNull_, STDERR_FILENO);
        enabled_ = true;
    }

    void disable() {
        if (!enabled_) return;
        dup2(stdoutCopy_, STDOUT_FILENO);
        dup2(stderrCopy_, STDERR_FILENO);
        close(stdoutCopy_);
        close(stderrCopy_);
        close(devNull_);
        enabled_ = false;
    }

    ~StdoutSilencer() {
        if (enabled_) enable();
    }

   private:
    int devNull_{-1};
    int stdoutCopy_{-1};
    int stderrCopy_{-1};
    bool enabled_ = false;
};

#endif
