# FindJetsonArgus.cmake

# This module helps locate Jetson Multimedia and Argus libraries and headers
# for use with NVIDIA Jetson platforms.

# Optional: Allow user to specify JETSON_MULTIMEDIA_ROOT
set(JETSON_MULTIMEDIA_ROOT "" CACHE PATH "Path to Jetson Multimedia API root directory")

if (NOT JETSON_MULTIMEDIA_ROOT)
  set(JETSON_MULTIMEDIA_ROOT "/usr/src/jetson_multimedia_api")
endif()

set(TEGRA_ARMABI aarch64-linux-gnu)

# Paths
set(JETSON_API_DIR        ${JETSON_MULTIMEDIA_ROOT})
set(JETSON_CLASS_DIR      ${JETSON_API_DIR}/samples/common/classes)
set(JETSON_UTILS_DIR      ${JETSON_API_DIR}/argus/samples/utils)
set(JETSON_ALGO_CUDA_DIR  ${JETSON_API_DIR}/samples/common/algorithm/cuda)
set(JETSON_ALGO_TRT_DIR   ${JETSON_API_DIR}/samples/common/algorithm/trt)

# Header paths
set(JETSON_ARGUS_INCLUDE_DIRS
  ${JETSON_API_DIR}/include
  ${JETSON_API_DIR}/include/libjpeg-8b
  /usr/include/${TEGRA_ARMABI}
  /usr/include/libdrm/
  /usr/local/cuda/include
  ${JETSON_CLASS_DIR}
  ${JETSON_UTILS_DIR}
  ${JETSON_ALGO_CUDA_DIR}
  ${JETSON_ALGO_TRT_DIR}
)

# Library paths
set(JETSON_ARGUS_LIBRARY_DIRS
  /usr/lib/${TEGRA_ARMABI}
  /usr/lib/${TEGRA_ARMABI}/nvidia
  /usr/lib/${TEGRA_ARMABI}/tegra
  /usr/local/cuda/lib64
)

# Libraries
set(JETSON_ARGUS_LIBRARIES
  nvargus
  argussampleutils
  nvbufsurface
  nvargus_socketclient
  pthread
  nvv4l2
  EGL
  GLESv2
  X11
  nvbufsurftransform
  nvjpeg
  nvosd
  drm
  cuda
  cudart
)

# Sources (optional)
file(GLOB JETSON_ARGUS_SOURCES CONFIGURE_DEPENDS "${JETSON_CLASS_DIR}/*.cpp")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(JetsonArgus DEFAULT_MSG JETSON_ARGUS_INCLUDE_DIRS JETSON_ARGUS_LIBRARIES)

mark_as_advanced(JETSON_ARGUS_INCLUDE_DIRS JETSON_ARGUS_LIBRARIES JETSON_ARGUS_SOURCES JETSON_ARGUS_LIBRARY_DIRS)
