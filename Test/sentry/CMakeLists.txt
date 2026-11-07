set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)
find_package(Threads REQUIRED)

option(SENTRY_SANITIZE "Run core checks with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
set(aim_root "${CMAKE_CURRENT_SOURCE_DIR}/../..")
add_library(sentry_regression_core STATIC
    ${aim_root}/Armor/sentry/config.cpp
    ${aim_root}/Armor/sentry/adapter.cpp
    ${aim_root}/Armor/sentry/target.cpp
    ${aim_root}/Armor/sentry/tracker.cpp
    ${aim_root}/Armor/sentry/aimer.cpp
    ${aim_root}/Armor/sentry/shooter.cpp
    ${aim_root}/tools/math_tools.cpp
    ${aim_root}/tools/trajectory.cpp
    ${aim_root}/tools/extended_kalman_filter.cpp
    ${aim_root}/tools/logger.cpp
    ${aim_root}/tools/player.cpp
    ${aim_root}/io/src/SerialPort.cpp
    ${aim_root}/Armor/src/ArmorSolver.cpp
    ${aim_root}/Armor/src/ArmorDetector.cpp
    ${aim_root}/Backbone/src/AutoArmor.cpp
    ${aim_root}/Omniperception/perceptron.cpp
    ${aim_root}/io/src/usbcamera.cpp
)
target_include_directories(sentry_regression_core PUBLIC
    ${aim_root}/Armor ${aim_root}/Armor/include ${aim_root}/Backbone/include
    ${aim_root}/io/include ${aim_root}/tools ${aim_root}/Omniperception ${OpenCV_INCLUDE_DIRS})
target_link_libraries(sentry_regression_core PUBLIC Eigen3::Eigen yaml-cpp
    spdlog::spdlog ${OpenCV_LIBS} Threads::Threads)
target_compile_options(sentry_regression_core PRIVATE -Wall -Wextra)
if(SENTRY_SANITIZE)
    target_compile_options(sentry_regression_core PUBLIC -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(sentry_regression_core PUBLIC -fsanitize=address,undefined)
endif()
add_executable(sentry_regression regression.cpp)
target_link_libraries(sentry_regression PRIVATE sentry_regression_core)
target_compile_definitions(sentry_regression PRIVATE AIM_CONFIG_PATH="${aim_root}/text.yaml")
foreach(check IN ITEMS bridge wire model association tracker timing priority shooter static_aim
                       moving_aim trajectory hold config solver blank_pipeline
                       baseline_parameters outpost_gate player)
    add_test(NAME sentry_${check} COMMAND sentry_regression ${check})
endforeach()

# Compile the updated real review entry without linking any MVS/ROS runtime.
add_library(sentry_review_compile OBJECT ${aim_root}/Test/src/test.cpp)
target_include_directories(sentry_review_compile PRIVATE ${aim_root}/Buff/include)
target_link_libraries(sentry_review_compile PRIVATE sentry_regression_core)

# Compile updated Buff code without constructing it or linking vehicle hardware.
add_library(sentry_buff_compile OBJECT
    ${aim_root}/Backbone/src/AutoBuff.cpp
    ${aim_root}/Buff/src/BuffDetect.cpp
    ${aim_root}/Buff/src/BuffSlove.cpp
    ${aim_root}/Buff/src/BuffPlan.cpp
    ${aim_root}/Buff/src/BuffTarget.cpp
    ${aim_root}/Buff/src/BuffType.cpp)
target_include_directories(sentry_buff_compile PRIVATE ${aim_root}/Buff/include)
target_link_libraries(sentry_buff_compile PRIVATE sentry_regression_core)

add_executable(sentry_offline_replay offline_replay.cpp)
target_link_libraries(sentry_offline_replay PRIVATE sentry_regression_core)
