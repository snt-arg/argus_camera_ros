# Argus Cameras ROS Package

The `argus_camera_ros` package provides a custom C++ ROS driver for NVIDIA Jetson platforms, specifically designed for high-performance applications where precise timestamping and low latency are critical. This driver was developed as an alternative to existing solutions that couldn't meet the demanding requirements for real-time robotic systems.

### Features

- **Event-Driven Architecture:** Each camera is treated as an independent object, minimizing latency and maximizing performance.
- **Precise Timestamping:** The driver captures hardware timestamps for each frame, ensuring accuracy.
- **Time Synchronization:** The driver supports two primary time synchronization modes:
    - **`TIME_FROM_ROS` (Default):** The system uses the ROS time domain. This is the default operational mode and provides reasonable accuracy due to the low-latency driver design.
    - **`TIME_FROM_PTP` (Precision Time Protocol):** This mode allows the driver to convert hardware timestamps into the PTP time domain. It is designed to work with a setup where the Jetson is the PTP master.

### System Requirements

- **ROS:** Humble
- **OS:** Jetpack 6.0 (or newer, but has not been tested on versions beyond Jetpack 6.0)
- **Hardware:** NVIDIA Jetson platform with a compatible camera.

### Getting Started

#### Building and Installation

**Note:** This package is designed to be built and run exclusively on NVIDIA Jetson platforms with the specified Jetpack versions.

1.  Clone the repository into your ROS workspace.

    ```bash
    cd <your_ros_workspace>/src
    git clone [https://github.com/snt-arg/argus_camera_ros.git](https://github.com/snt-arg/argus_camera_ros.git)
    ```

2.  Build the package using `colcon`.

    ```bash
    cd <your_ros_workspace>
    colcon build --packages-select argus_camera_ros
    ```

### Usage

```bash
ros2 launch argus_camera_ros argus_camera.launch.py
```

### Using PTP Timestamping

To use the `TIME_FROM_PTP` timestamping mode, you must first ensure that the system's PTP configuration is correct and then enable read/write access to the PTP hardware clock.

1.  Edit the udev rules to grant permissions to the PTP device.

    ```bash
    sudo vim /etc/udev/rules.d/99-nvpps.rules
    ```

2.  Add the following line to the file:

    ```
    KERNEL=="nvpps0", MODE="0666"
    ```

3.  Apply the new udev rules by running the following commands or by rebooting your system.

    ```bash
    sudo udevadm control --reload
    sudo udevadm trigger
    ```

With these steps complete, you can now configure the `argus_camera_ros` package to use the `TIME_FROM_PTP` timestamping mode.
