cmake_minimum_required(VERSION 3.16)

project(VISION)
list(APPEND CMAKE_PREFIX_PATH "/opt/ros/humble")
# 设置构建类型为 Release
set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找必要的包
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(fmt REQUIRED)
add_subdirectory(tinympc)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 设置包含目录
include_directories(
    ${CMAKE_SOURCE_DIR}/Backbone/include
    ${CMAKE_SOURCE_DIR}/io/include
    ${CMAKE_SOURCE_DIR}/Recognition/include
    ${CMAKE_SOURCE_DIR}/Pose_solution/include
    ${CMAKE_SOURCE_DIR}/Predict/include
    ${CMAKE_SOURCE_DIR}/Buff/include
    ${CMAKE_SOURCE_DIR}/Test/include
    ${CMAKE_SOURCE_DIR}/tools
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${CERES_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
    "/opt/MVS/include"
    ${CMAKE_SOURCE_DIR}/tinympc
    ${LIBUSB_INCLUDE_DIRS}  
)

# 设置库文件目录
link_directories("/opt/MVS/lib")

# 查找源文件
file(GLOB_RECURSE sourceFiles
    "${CMAKE_SOURCE_DIR}/Backbone/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/io/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/Recognition/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/Pose_solution/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/Predict/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/Buff/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/Test/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/tools/*.cpp"
)
list(FILTER sourceFiles EXCLUDE REGEX ".*/build/.*")

add_executable(action ${sourceFiles})

find_library(MVS libMvCameraControl.so PATHS /opt/MVS/lib /opt/MVS/lib/64)

target_link_libraries(action
    ${OpenCV_LIBS}
    fmt::fmt
    Threads::Threads
    ${EIGEN3_LIBRARIES}
    ${CERES_LIBRARIES}
    ${MVS}
    -lgxiapi
    ${YAML_CPP_LIBRARIES}
    tinympcstatic
    ${LIBUSB_LIBRARIES}  
    rclcpp::rclcpp
    std_msgs::std_msgs
)

# 设置Release模式的编译选项，使用-O2优化
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 -Wall")

# 输出包含目录的路径
message(STATUS "OpenCV include directories: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "Eigen include directories: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "Ceres include directories: ${CERES_INCLUDE_DIRS}")
message(STATUS "yaml-cpp include directories: ${YAML_CPP_INCLUDE_DIRS}")
message(STATUS "libusb libraries: ${LIBUSB_LIBRARIES}")
message(STATUS "libusb include dirs: ${LIBUSB_INCLUDE_DIRS}")