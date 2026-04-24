cmake_minimum_required(VERSION 2.8.0)

project(cvi_sample_detector C CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

if(NOT DEFINED TPU_SDK_PATH)
  message(FATAL_ERROR "Please set TPU_SDK_PATH to point to the TPU_SDK installation")
endif()
include_directories(${TPU_SDK_PATH}/include)
link_directories(${TPU_SDK_PATH}/lib)

if(NOT DEFINED OPENCV_PATH)
  message(FATAL_ERROR "Please set OPENCV_PATH to point to the opencvn installation")
endif()
include_directories(${OPENCV_PATH}/include)
link_directories(${OPENCV_PATH}/lib)

# 打印log等级

message(STATUS "Log level set to: ${LOG_LEVEL}")
if(NOT DEFINED LOG_LEVEL)
  set(LOG_LEVEL "INFO")
endif()

if(LOG_LEVEL STREQUAL "DEBUG")
  add_definitions(-DCURRENT_LOG_LEVEL=0)
elseif(LOG_LEVEL STREQUAL "INFO")
  add_definitions(-DCURRENT_LOG_LEVEL=1)
elseif(LOG_LEVEL STREQUAL "WARN")
  add_definitions(-DCURRENT_LOG_LEVEL=2)
elseif(LOG_LEVEL STREQUAL "ERROR")
  add_definitions(-DCURRENT_LOG_LEVEL=3)
else()
  add_definitions(-DCURRENT_LOG_LEVEL=1) # Default to INFO
endif()

# Add sensor include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/sensor/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/sensor/include/isp/sg200x)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/sensor/include/common)

# Add parent directory include path for test files in tests/
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Add sensor library path
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/sensor/lib)

set(CVI_LIBS ${CVI_LIBS} cviruntime cvikernel)
if(NOT CMAKE_CROSSCOMPILING)
  set(CVI_LIBS ${CVI_LIBS} cvicmodel)
endif()

set(OPENCV_LIBS ${OPENCV_LIBS} opencv_core opencv_imgcodecs opencv_imgproc)
if(NOT CMAKE_CROSSCOMPILING)
  set(OPENCV_LIBS ${OPENCV_LIBS} opencv_highgui)
endif()

set(EXTRA_LIBS ${EXTRA_LIBS} dl stdc++ pthread z atomic)

# Author: Zhihang Shao <dio_ro@outlook.com>
# Source: aka0-ref commit 755a885
# 添加机械臂支持
add_executable(tennis tennis.cpp motor.cpp arm.cpp vi_capture.cpp)
target_link_libraries(tennis
    ${CVI_LIBS}
    ${OPENCV_LIBS}
    -Wl,--start-group
    sample isp isp_algo ae awb af
    vpu cvi_ispd2 cvi_bin_isp cvi_bin
    sns_full sys ini json-c
    -Wl,--end-group
    ${EXTRA_LIBS})
install(TARGETS tennis DESTINATION samples_extra/bin)

# =====================================================
# Test utilities (in tests/ directory)
# Author: Zhihang Shao <dio_ro@outlook.com>
# Source: aka0-ref commits 755a885, 64f8dab
# =====================================================

# Arm test utility (no SDK dependencies needed)
add_executable(test_arm tests/test_arm.cpp arm.cpp)

# Motor test utility (no SDK dependencies needed)
add_executable(test_motor tests/test_motor.cpp motor.cpp)

# Single-frame capture + detection utility
add_executable(test_capture_detect tests/test_capture_detect.cpp vi_capture.cpp)
target_link_libraries(test_capture_detect
    ${CVI_LIBS}
    ${OPENCV_LIBS}
    -Wl,--start-group
    sample isp isp_algo ae awb af
    vpu cvi_ispd2 cvi_bin_isp cvi_bin
    sns_full sys ini json-c
    -Wl,--end-group
    ${EXTRA_LIBS})