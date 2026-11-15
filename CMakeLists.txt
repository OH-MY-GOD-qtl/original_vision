cmake_minimum_required(VERSION 3.16)
project(hik_view LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  set(SDK_ARCH amd64)
else()
  set(SDK_ARCH arm64)
endif()

set(SDK ${CMAKE_SOURCE_DIR}/third_party/camera_sdk/hikrobot)

add_executable(hik_capture test/camera_test.cpp)
target_include_directories(hik_capture PRIVATE ${SDK}/include ${OpenCV_INCLUDE_DIRS})
target_link_directories(hik_capture PRIVATE ${SDK}/lib/${SDK_ARCH})
target_link_libraries(hik_capture PRIVATE MvCameraControl ${OpenCV_LIBS} yaml-cpp)
set_target_properties(hik_capture PROPERTIES BUILD_RPATH "${SDK}/lib/${SDK_ARCH}")
 