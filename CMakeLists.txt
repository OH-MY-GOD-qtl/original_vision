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

# 导入 SDK：用绝对路径直接链，-l 依赖的库目录不会从静态库传播出去
add_library(MvCameraControl SHARED IMPORTED)
set_target_properties(MvCameraControl PROPERTIES
  IMPORTED_LOCATION ${SDK}/lib/${SDK_ARCH}/libMvCameraControl.so
  INTERFACE_INCLUDE_DIRECTORIES ${SDK}/include)

# 相机模块：SDK 头与库只出现在它的实现里，调用方只需要 include/
add_library(camera STATIC src/camera.cpp)
target_include_directories(camera PUBLIC ${CMAKE_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(camera PUBLIC ${OpenCV_LIBS} PRIVATE MvCameraControl)

add_executable(camera_test test/camera_test.cpp)
target_link_libraries(camera_test PRIVATE camera yaml-cpp)
set_target_properties(camera_test PROPERTIES BUILD_RPATH "${SDK}/lib/${SDK_ARCH}")

add_subdirectory(calibration)
 