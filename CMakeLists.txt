cmake_minimum_required(VERSION 3.16)

project(VISION)
option(AUTO_AIM_CORE_ONLY "Build hardware-free sentry regression checks" OFF)
if(AUTO_AIM_CORE_ONLY)
    enable_testing()
    add_subdirectory(Test/sentry)
    return()
endif()
list(APPEND CMAKE_PREFIX_PATH "/opt/ros/humble")
# 设置构建类型为 Release
set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 在设置全局标志之前，先设置Release模式的优化级别
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -Wall")
set(CMAKE_C_FLAGS_RELEASE "-O2")

# 全局编译标志（会被特定构建类型的标志覆盖）
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# 查找必要的包
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(fmt REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(OPENVINO REQUIRED openvino)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl REQUIRED)
find_package(rmw REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(rosidl_typesupport_interface REQUIRED)

# 设置包含目录
include_directories(
    ${CMAKE_SOURCE_DIR}/Backbone/include
    ${CMAKE_SOURCE_DIR}/io/include
    ${CMAKE_SOURCE_DIR}/Armor/include
    ${CMAKE_SOURCE_DIR}/Armor
    ${CMAKE_SOURCE_DIR}/Buff/include
    ${CMAKE_SOURCE_DIR}/Test/include
    ${CMAKE_SOURCE_DIR}/tools
    ${CMAKE_SOURCE_DIR}/Omniperception
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${CERES_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
    "/opt/MVS/include"
    ${LIBUSB_INCLUDE_DIRS}
    ${OPENVINO_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${rcl_INCLUDE_DIRS}
    ${rmw_INCLUDE_DIRS}
    ${rosidl_runtime_c_INCLUDE_DIRS}
    ${rosidl_typesupport_interface_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
)

# 设置库文件目录
link_directories("/opt/MVS/lib")

# 查找共同源文件（不含main）
file(GLOB_RECURSE commonSources
    "${CMAKE_SOURCE_DIR}/Backbone/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/io/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/Armor/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/Armor/sentry/*.cpp"
    "${CMAKE_SOURCE_DIR}/Buff/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/tools/*.cpp"
    "${CMAKE_SOURCE_DIR}/Omniperception/*.cpp"
)
list(FILTER commonSources EXCLUDE REGEX ".*/build/.*")
# 从common中排除所有含main的文件，避免重复定义
list(FILTER commonSources EXCLUDE REGEX ".*main\\.cpp$")
# Replaced by the coherent B tracker/target/aimer stack; retain the source for reference.
list(FILTER commonSources EXCLUDE REGEX "/Armor/src/Armor(Target|EKF|Predictor|Planner)\\.cpp$")

# 实时运行程序 (live)
add_executable(action
    "${CMAKE_SOURCE_DIR}/Backbone/src/main.cpp"
    ${commonSources}
)

# 离线复盘程序 (review)
add_executable(review
    "${CMAKE_SOURCE_DIR}/Test/src/test.cpp"
    ${commonSources}
)

find_library(MVS libMvCameraControl.so PATHS /opt/MVS/lib /opt/MVS/lib/64)

target_link_libraries(action
    ${OpenCV_LIBS}
    fmt::fmt
    Threads::Threads
    ${EIGEN3_LIBRARIES}
    ${CERES_LIBRARIES}
    ${MVS}
    ${YAML_CPP_LIBRARIES}
    ${LIBUSB_LIBRARIES}
    ${OPENVINO_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${rcl_LIBRARIES}
    ${rmw_LIBRARIES}
    ${rosidl_runtime_c_LIBRARIES}
    ${rosidl_typesupport_interface_LIBRARIES}
)

target_link_libraries(review
    ${OpenCV_LIBS}
    fmt::fmt
    Threads::Threads
    ${EIGEN3_LIBRARIES}
    ${CERES_LIBRARIES}
    ${MVS}
    ${YAML_CPP_LIBRARIES}
    ${LIBUSB_LIBRARIES}
    ${OPENVINO_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${rcl_LIBRARIES}
    ${rmw_LIBRARIES}
    ${rosidl_runtime_c_LIBRARIES}
    ${rosidl_typesupport_interface_LIBRARIES}
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
message(STATUS "rclcpp include dirs: ${rclcpp_INCLUDE_DIRS}")
message(STATUS "rcl include dirs: ${rcl_INCLUDE_DIRS}")
message(STATUS "rmw include dirs: ${rmw_INCLUDE_DIRS}")
message(STATUS "rosidl_runtime_c include dirs: ${rosidl_runtime_c_INCLUDE_DIRS}")
message(STATUS "rosidl_typesupport_interface include dirs: ${rosidl_typesupport_interface_INCLUDE_DIRS}")
message(STATUS "std_msgs include dirs: ${std_msgs_INCLUDE_DIRS}")
message(STATUS "rclcpp libraries: ${rclcpp_LIBRARIES}")
message(STATUS "rcl libraries: ${rcl_LIBRARIES}")
message(STATUS "rmw libraries: ${rmw_LIBRARIES}")

# 独立单相机自瞄测试，不包含整车main入口
add_executable(aim_preview
    "${CMAKE_SOURCE_DIR}/Test/src/aim_preview.cpp"
    ${commonSources}
)

target_link_libraries(aim_preview
    ${OpenCV_LIBS}
    fmt::fmt
    Threads::Threads
    ${EIGEN3_LIBRARIES}
    ${CERES_LIBRARIES}
    ${MVS}
    ${YAML_CPP_LIBRARIES}
    ${LIBUSB_LIBRARIES}
    ${OPENVINO_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${rcl_LIBRARIES}
    ${rmw_LIBRARIES}
    ${rosidl_runtime_c_LIBRARIES}
    ${rosidl_typesupport_interface_LIBRARIES}
)
