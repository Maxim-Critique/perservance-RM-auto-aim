find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui dnn calib3d)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(KEYBOARD_LIBUSB REQUIRED IMPORTED_TARGET libusb-1.0)
find_package(QT NAMES Qt5 Qt6 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)
find_path(KEYBOARD_MVS_INCLUDE MvCameraControl.h PATHS /opt/MVS/include REQUIRED)
find_library(KEYBOARD_MVS_LIBRARY MvCameraControl PATHS /opt/MVS/lib/64 /opt/MVS/lib REQUIRED)

add_executable(keyboard_gimbal_test
    ../src/keyboard_gimbal_test.cpp
    ../../Armor/src/ArmorDetector.cpp
    ../../Backbone/src/AutoArmor.cpp
    ../../Armor/src/ArmorSolver.cpp
    ../../Armor/sentry/config.cpp
    ../../Armor/sentry/adapter.cpp
    ../../Armor/sentry/target.cpp
    ../../Armor/sentry/tracker.cpp
    ../../Armor/sentry/aimer.cpp
    ../../Armor/sentry/shooter.cpp
    ../../io/src/hikrobot.cpp
    ../../io/src/SerialPort.cpp
    ../../tools/math_tools.cpp
    ../../tools/trajectory.cpp
    ../../tools/extended_kalman_filter.cpp
    ../../tools/logger.cpp
    ../../tools/exiter.cpp
)
target_compile_features(keyboard_gimbal_test PRIVATE cxx_std_17)
target_compile_options(keyboard_gimbal_test PRIVATE -Wall -Wextra)
target_include_directories(keyboard_gimbal_test PRIVATE
    ../../Armor ../../Armor/include ../../Backbone/include ../../io/include ../../tools
    ${OpenCV_INCLUDE_DIRS} ${KEYBOARD_MVS_INCLUDE}
)
target_link_libraries(keyboard_gimbal_test PRIVATE
    Qt${QT_VERSION_MAJOR}::Widgets ${OpenCV_LIBS} Eigen3::Eigen Threads::Threads
    yaml-cpp spdlog::spdlog PkgConfig::KEYBOARD_LIBUSB ${KEYBOARD_MVS_LIBRARY}
)
set_target_properties(keyboard_gimbal_test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)
