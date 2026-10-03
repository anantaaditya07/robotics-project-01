cmake_minimum_required(VERSION 3.8)
project(semnav_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc dnn)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

# --- ONNX Runtime (pinned, fetched by scripts/setup_ort.sh, D-12) ---
set(SEMNAV_ORT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/onnxruntime"
    CACHE PATH "ONNX Runtime install root (from scripts/setup_ort.sh)")
get_filename_component(SEMNAV_ORT_ROOT "${SEMNAV_ORT_ROOT}" ABSOLUTE)
if(NOT EXISTS "${SEMNAV_ORT_ROOT}/lib/libonnxruntime.so"
   OR NOT EXISTS "${SEMNAV_ORT_ROOT}/include/onnxruntime_cxx_api.h")
  message(FATAL_ERROR
    "ONNX Runtime not found in ${SEMNAV_ORT_ROOT}.\n"
    "Run: scripts/setup_ort.sh   (from the repo root), then rebuild.")
endif()
add_library(onnxruntime::onnxruntime SHARED IMPORTED)
set_target_properties(onnxruntime::onnxruntime PROPERTIES
  IMPORTED_LOCATION "${SEMNAV_ORT_ROOT}/lib/libonnxruntime.so.1.20.1"
  IMPORTED_SONAME "libonnxruntime.so.1"
  INTERFACE_INCLUDE_DIRECTORIES "${SEMNAV_ORT_ROOT}/include"
  INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${SEMNAV_ORT_ROOT}/include")

# Ship the runtime library with the package so `ros2 run` finds it.
file(GLOB SEMNAV_ORT_LIBS "${SEMNAV_ORT_ROOT}/lib/libonnxruntime.so*")
install(FILES ${SEMNAV_ORT_LIBS} DESTINATION lib)

set(SEMNAV_MODEL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../models/yolov8n.onnx")
get_filename_component(SEMNAV_MODEL_PATH "${SEMNAV_MODEL_PATH}" ABSOLUTE)

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Hello-world Ort::Session before any ROS code (architecture section 11).
  ament_add_gtest(test_ort_session test/test_ort_session.cpp)
  target_link_libraries(test_ort_session onnxruntime::onnxruntime)
  target_compile_definitions(test_ort_session PRIVATE
    SEMNAV_TEST_MODEL_PATH="${SEMNAV_MODEL_PATH}")
endif()

ament_package()
