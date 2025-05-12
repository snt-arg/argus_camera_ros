/**
 * @file include/argus_camera_ros/logger.hpp
 * @brief Defines a simple logger which can work with rclcpp::Logger or standalone
 * to be used mainly for the ArgusCamera class.
 */

#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <functional>
#include <iostream>
#include <string>

namespace argus_camera_ros {

enum class LogLevel { DEBUG, INFO, WARN, ERROR };

class Logger {
   public:
    using LogFunction = std::function<void(LogLevel, const std::string&)>;

    Logger() = default;

    void setLogger(LogFunction func) { logFunc_ = std::move(func); }

    void log(LogLevel level, const std::string& msg) {
        if (logFunc_) {
            logFunc_(level, msg);
        } else {
            defaultLog(level, msg);
        }
    }

    void debug(const std::string& msg, const std::string& prefix = "") {
        log(LogLevel::DEBUG, prefix + msg);
    }
    void info(const std::string& msg, const std::string& prefix = "") {
        log(LogLevel::INFO, prefix + msg);
    }
    void warn(const std::string& msg, const std::string& prefix = "") {
        log(LogLevel::WARN, prefix + msg);
    }
    void error(const std::string& msg, const std::string& prefix = "") {
        log(LogLevel::ERROR, prefix + msg);
    }

   private:
    LogFunction logFunc_ = nullptr;

    static void defaultLog(LogLevel level, const std::string& msg) {
        const char* prefix = "";
        switch (level) {
            case LogLevel::DEBUG:
                prefix = "[DEBUG]";
                break;
            case LogLevel::INFO:
                prefix = "[INFO] ";
                break;
            case LogLevel::WARN:
                prefix = "[WARN] ";
                break;
            case LogLevel::ERROR:
                prefix = "[ERROR]";
                break;
        }
        std::cout << prefix << " " << msg << std::endl;
    }
};

};  // namespace argus_camera_ros

#endif  // LOGGER_HPP
