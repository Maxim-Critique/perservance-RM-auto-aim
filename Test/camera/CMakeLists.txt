find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui dnn videoio)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(PREVIEW_LIBUSB REQUIRED IMPORTED_TARGET libusb-1.0)
find_path(PREVIEW_MVS_INCLUDE MvCameraControl.h PATHS /opt/MVS/include REQUIRED)
find_library(PREVIEW_MVS_LIBRARY MvCameraControl PATHS /opt/MVS/lib/64 /opt/MVS/lib REQUIRED)

# Only camera, detector and generic filter code: no SerialPort, AutoArmor, ROS or shooter sources.
add_executable(camera_detection_preview
    ../src/camera_detection_preview.cpp
    ../../Armor/src/ArmorDetector.cpp
    ../../io/src/hikrobot.cpp
    ../../tools/math_tools.cpp
    ../../tools/extended_kalman_filter.cpp
    ../../tools/logger.cpp
    ../../tools/exiter.cpp
)
target_compile_features(camera_detection_preview PRIVATE cxx_std_17)
target_compile_options(camera_detection_preview PRIVATE -Wall -Wextra)
target_compile_definitions(camera_detection_preview PRIVATE
    PREVIEW_MODEL_DIR="${PROJECT_SOURCE_DIR}/model"
)
target_include_directories(camera_detection_preview PRIVATE
    ../../Armor/include ../../Backbone/include ../../io/include ../../tools
    ${OpenCV_INCLUDE_DIRS} ${PREVIEW_MVS_INCLUDE}
)
target_link_libraries(camera_detection_preview PRIVATE
    ${OpenCV_LIBS} Eigen3::Eigen Threads::Threads yaml-cpp spdlog::spdlog
    PkgConfig::PREVIEW_LIBUSB ${PREVIEW_MVS_LIBRARY}
)
set_target_properties(camera_detection_preview PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
