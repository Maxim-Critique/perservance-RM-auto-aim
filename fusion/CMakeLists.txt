find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenVINO REQUIRED COMPONENTS Runtime)
set(TONGJI_ROOT ${PROJECT_SOURCE_DIR}/third_party/tongji)
set(TONGJI_FILES armor classifier detector solver target tracker yolo)
set(TONGJI_SOURCES)
foreach(name IN LISTS TONGJI_FILES)
  list(APPEND TONGJI_SOURCES ${TONGJI_ROOT}/tasks/auto_aim/${name}.cpp)
endforeach()
foreach(name yolov5 yolov8 yolo11)
  list(APPEND TONGJI_SOURCES ${TONGJI_ROOT}/tasks/auto_aim/yolos/${name}.cpp)
endforeach()
foreach(name extended_kalman_filter math_tools logger img_tools)
  list(APPEND TONGJI_SOURCES ${TONGJI_ROOT}/tools/${name}.cpp)
endforeach()
add_library(tongji_core STATIC ${TONGJI_SOURCES} ${PROJECT_SOURCE_DIR}/Armor/tongji_adapter.cpp)
target_include_directories(tongji_core PUBLIC ${TONGJI_ROOT} ${PROJECT_SOURCE_DIR}/Armor
  ${PROJECT_SOURCE_DIR}/Backbone/include ${PROJECT_SOURCE_DIR}/Armor/include ${PROJECT_SOURCE_DIR}/io/include ${PROJECT_SOURCE_DIR}/tools)
target_link_libraries(tongji_core PUBLIC ${OpenCV_LIBS} Eigen3::Eigen yaml-cpp
  ${CERES_LIBRARIES} fmt::fmt spdlog::spdlog Threads::Threads openvino::runtime)
target_compile_options(tongji_core PRIVATE -Wall -Wextra)
if(FUSION_SANITIZE)
  target_compile_options(tongji_core PUBLIC -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(tongji_core PUBLIC -fsanitize=address,undefined)
endif()
if(FUSION_OFFLINE_ONLY)
  file(GLOB A_ARMOR ${PROJECT_SOURCE_DIR}/Armor/src/*.cpp)
  # Use the ORIGINAL protocol methods byte-for-byte, without serial implementation/constructors.
  file(READ ${PROJECT_SOURCE_DIR}/io/src/SerialPort.cpp SERIAL_SOURCE)
  string(FIND "${SERIAL_SOURCE}" "void Data_Get::Vision2Control" PROTOCOL_START)
  if(PROTOCOL_START LESS 0)
    message(FATAL_ERROR "Original protocol methods not found")
  endif()
  string(SUBSTRING "${SERIAL_SOURCE}" ${PROTOCOL_START} -1 PROTOCOL_SOURCE)
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/original_protocol.cpp
    "#include \"Base.h\"\n#include \"logger.hpp\"\n${PROTOCOL_SOURCE}")
  add_library(fusion_test_core OBJECT ${A_ARMOR}
    ${PROJECT_SOURCE_DIR}/Backbone/src/AutoArmor.cpp
    ${PROJECT_SOURCE_DIR}/tools/math_tools.cpp ${PROJECT_SOURCE_DIR}/tools/trajectory.cpp
    ${PROJECT_SOURCE_DIR}/tools/logger.cpp ${PROJECT_SOURCE_DIR}/tools/player.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/original_protocol.cpp)
  target_link_libraries(fusion_test_core PUBLIC tongji_core)
  add_executable(fusion_regression ${PROJECT_SOURCE_DIR}/Test/rebuild/regression.cpp)
  target_link_libraries(fusion_regression PRIVATE fusion_test_core)
  add_executable(fusion_offline ${PROJECT_SOURCE_DIR}/Test/rebuild/offline.cpp)
  target_link_libraries(fusion_offline PRIVATE fusion_test_core)
  add_library(fusion_review_compile OBJECT ${PROJECT_SOURCE_DIR}/Test/src/test.cpp
    ${PROJECT_SOURCE_DIR}/Backbone/src/AutoBuff.cpp
    ${PROJECT_SOURCE_DIR}/Buff/src/BuffDetect.cpp ${PROJECT_SOURCE_DIR}/Buff/src/BuffSlove.cpp
    ${PROJECT_SOURCE_DIR}/Buff/src/BuffPlan.cpp ${PROJECT_SOURCE_DIR}/Buff/src/BuffTarget.cpp
    ${PROJECT_SOURCE_DIR}/Buff/src/BuffType.cpp ${PROJECT_SOURCE_DIR}/io/src/SerialPort.cpp)
  target_include_directories(fusion_review_compile PRIVATE ${PROJECT_SOURCE_DIR}/Buff/include
    ${PROJECT_SOURCE_DIR}/Omniperception)
  target_link_libraries(fusion_review_compile PRIVATE tongji_core)
  add_executable(fusion_serial_mock ${PROJECT_SOURCE_DIR}/Test/rebuild/serial_mock.cpp
    ${PROJECT_SOURCE_DIR}/io/src/SerialPort.cpp ${PROJECT_SOURCE_DIR}/tools/math_tools.cpp
    ${PROJECT_SOURCE_DIR}/tools/logger.cpp)
  target_link_libraries(fusion_serial_mock PRIVATE tongji_core util)
  add_test(NAME fusion_serial_mock COMMAND fusion_serial_mock)
  set_tests_properties(fusion_serial_mock PROPERTIES TIMEOUT 5)
  foreach(case calibration type pose tracker outpost planner ekf protocol time empty player hold model)
    add_test(NAME fusion_${case} COMMAND fusion_regression ${case}
      ${PROJECT_SOURCE_DIR}/text.yaml ${PROJECT_SOURCE_DIR}/configs/tongji_sentry.yaml)
    set_tests_properties(fusion_${case} PROPERTIES WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  endforeach()
endif()
