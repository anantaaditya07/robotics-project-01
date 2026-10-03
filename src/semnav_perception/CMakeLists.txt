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

add_executable(yolo_onnx_node src/yolo_onnx_node.cpp)
target_include_directories(yolo_onnx_node PRIVATE include)
target_link_libraries(yolo_onnx_node onnxruntime::onnxruntime ${OpenCV_LIBS})
ament_target_dependencies(yolo_onnx_node cv_bridge rclcpp sensor_msgs std_msgs vision_msgs)
# Default model_path parameter: the repo's models/ (model files are not in git, D-12).
target_compile_definitions(yolo_onnx_node PRIVATE
  SEMNAV_DEFAULT_MODEL_PATH="${SEMNAV_MODEL_PATH}")
set_target_properties(yolo_onnx_node PROPERTIES INSTALL_RPATH "$ORIGIN/..")
install(TARGETS yolo_onnx_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Hello-world Ort::Session before any ROS code (architecture section 11).
  ament_add_gtest(test_ort_session test/test_ort_session.cpp)
  target_link_libraries(test_ort_session onnxruntime::onnxruntime)
  target_compile_definitions(test_ort_session PRIVATE
    SEMNAV_TEST_MODEL_PATH="${SEMNAV_MODEL_PATH}")

  # ROS-free pre/post-processing (architecture 7.1 tests).
  foreach(t test_letterbox test_yolo_decode test_rolling_stats test_yolo_detector)
    ament_add_gtest(${t} test/${t}.cpp)
    target_include_directories(${t} PRIVATE include)
    target_link_libraries(${t} onnxruntime::onnxruntime ${OpenCV_LIBS})
  endforeach()
  target_compile_definitions(test_yolo_detector PRIVATE
    SEMNAV_TEST_MODEL_PATH="${SEMNAV_MODEL_PATH}")
endif()

ament_package()
