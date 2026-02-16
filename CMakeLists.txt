cmake_minimum_required(VERSION 3.16.3)

project(sp_vision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")

option(SP_VISION_BUILD_XROBOT "Build xrobot-only architecture" ON)

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui videoio dnn calib3d)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake/")
find_package(OpenVINO REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/Modules)
include_directories(${PROJECT_SOURCE_DIR}/Modules/SpVisionCommon)
include_directories(${PROJECT_SOURCE_DIR}/Modules/SpVisionCamera)
include_directories(${PROJECT_SOURCE_DIR}/Modules/SpVisionState)
include_directories(${PROJECT_SOURCE_DIR}/Modules/SpVisionAutoAimCore)
include_directories(${PROJECT_SOURCE_DIR}/Modules/SpVisionAutoBuffCore)
include_directories(${PROJECT_SOURCE_DIR}/Modules/SpVisionOmniCore)

add_subdirectory(${PROJECT_SOURCE_DIR}/Modules/SpVisionCommon/tools ${CMAKE_BINARY_DIR}/spvision_tools)
add_subdirectory(
  ${PROJECT_SOURCE_DIR}/Modules/SpVisionState/io/serial ${CMAKE_BINARY_DIR}/spvision_serial)
add_subdirectory(${PROJECT_SOURCE_DIR}/Modules/SpVisionState/io ${CMAKE_BINARY_DIR}/spvision_io)
add_subdirectory(
  ${PROJECT_SOURCE_DIR}/Modules/SpVisionAutoAimCore/tasks/auto_aim
  ${CMAKE_BINARY_DIR}/spvision_auto_aim)
add_subdirectory(
  ${PROJECT_SOURCE_DIR}/Modules/SpVisionAutoBuffCore/tasks/auto_buff
  ${CMAKE_BINARY_DIR}/spvision_auto_buff)
add_subdirectory(
  ${PROJECT_SOURCE_DIR}/Modules/SpVisionOmniCore/tasks/omniperception
  ${CMAKE_BINARY_DIR}/spvision_omniperception)

if(SP_VISION_BUILD_XROBOT)
  set(XROBOT_MODULES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Modules)
  set(LIBXR_SYSTEM Linux)
  set(LIBXR_DRIVER Linux)
  set(LIBXR_LOG_LEVEL 4)
  set(LIBXR_NO_EIGEN True)

  add_subdirectory(libxr)

  add_executable(sp_vision_xrobot User/main.cpp)
  target_link_libraries(
    sp_vision_xrobot
    PUBLIC xr
    PUBLIC fmt::fmt
    PUBLIC yaml-cpp
  )
  target_link_options(sp_vision_xrobot PRIVATE -Wl,--no-as-needed)
  # Keep OpenCV libs at the end of link line to satisfy symbols introduced by static libs.
  target_link_libraries(sp_vision_xrobot PRIVATE ${OpenCV_LIBS})
  target_include_directories(
    sp_vision_xrobot
    PUBLIC $<TARGET_PROPERTY:xr,INTERFACE_INCLUDE_DIRECTORIES>
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/User
  )
endif()
