cmake_minimum_required(VERSION 3.8)
project(semnav_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch maps models rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

# Object models fetched by scripts/fetch_models.sh (git-ignored, D-14). Optional at build time.
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/models_external)
  install(DIRECTORY models_external DESTINATION share/${PROJECT_NAME})
else()
  message(WARNING "models_external/ missing: run scripts/fetch_models.sh for person/chair/bottle")
endif()

ament_package()
