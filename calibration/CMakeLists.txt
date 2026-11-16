add_executable(calibrate_transport transport_delay.cpp)
target_link_libraries(calibrate_transport PRIVATE vision_camera yaml-cpp)
set_target_properties(calibrate_transport PROPERTIES BUILD_RPATH "${SDK}/lib/${SDK_ARCH}")
