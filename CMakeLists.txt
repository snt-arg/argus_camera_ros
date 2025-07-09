cmake_minimum_required(VERSION 3.8)
project(argus_camera_ros)

# Add cmake/ path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# Find dependencies
find_package(JetsonArgus REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}/include/
    ${JETSON_ARGUS_INCLUDE_DIRS}
)

link_directories(
  ${PROJECT_SOURCE_DIR}/lib
  ${JETSON_ARGUS_LIBRARY_DIRS}
)

# Dependencies
SET(DEPENDENCIES 
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  OpenCV
  cv_bridge
  image_transport
  camera_info_manager
)

SET(LIBS ${JETSON_ARGUS_LIBRARIES})

file(GLOB SRC_FILES CONFIGURE_DEPENDS "src/argus_camera_ros/*.cpp")
SET(SOURCES ${SRC_FILES} ${JETSON_ARGUS_SOURCES})

# Executables
add_executable(multicamera_node apps/multicamera_node.cpp ${SOURCES})
ament_target_dependencies(multicamera_node ${DEPENDENCIES})
target_link_libraries(multicamera_node ${LIBS})

add_executable(singlecamera_node apps/singlecamera_node.cpp ${SOURCES})
ament_target_dependencies(singlecamera_node ${DEPENDENCIES})
target_link_libraries(singlecamera_node ${LIBS})

install(
  DIRECTORY include
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS 
  multicamera_node
  singlecamera_node
  DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
