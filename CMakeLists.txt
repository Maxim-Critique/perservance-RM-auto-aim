cmake_minimum_required(VERSION 3.0)

project(VISION)

# 设置构建类型为 Debug
set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build." FORCE)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找必要的包
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(yaml-cpp REQUIRED)

# 设置包含目录
include_directories(
    ${CMAKE_SOURCE_DIR}/Backbone/include
    ${CMAKE_SOURCE_DIR}/Cameras/include
    ${CMAKE_SOURCE_DIR}/Recognition/include
    ${CMAKE_SOURCE_DIR}/Pose_solution/include
    ${CMAKE_SOURCE_DIR}/Predict/include
    ${CMAKE_SOURCE_DIR}/Buff/include
    ${CMAKE_SOURCE_DIR}/Test/include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${CERES_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
    "/opt/MVS/include"
)

# 设置库文件目录
link_directories("/opt/MVS/lib")
# 查找源文件
file(GLOB_RECURSE sourceFiles
    "${CMAKE_SOURCE_DIR}/Backbone/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/Cameras/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/Recognition/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/Pose_solution/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/Predict/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/Buff/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/Test/src/*.cpp"
)
list(FILTER sourceFiles EXCLUDE REGEX ".*/build/.*")
# 添加可执行目标
add_executable(action ${sourceFiles})

# 查找 MVS 库
find_library(MVS libMvCameraControl.so PATHS /opt/MVS/lib /opt/MVS/lib/64)

# 链接库
target_link_libraries(action
    ${OpenCV_LIBS}
    Threads::Threads
    ${EIGEN3_LIBRARIES}
    ${CERES_LIBRARIES}
    ${MVS}
    -lgxiapi
    ${YAML_CPP_LIBRARIES}
)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0 -Wall")
# 输出包含目录的路径
message(STATUS "OpenCV include directories: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "Eigen include directories: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "Ceres include directories: ${CERES_INCLUDE_DIRS}")
message(STATUS "yaml-cpp include directories: ${YAML_CPP_INCLUDE_DIRS}")
