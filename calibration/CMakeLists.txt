add_executable(transport_delay transport_delay.cpp)
target_link_libraries(transport_delay PRIVATE camera yaml-cpp)
set_target_properties(transport_delay PROPERTIES BUILD_RPATH "${SDK}/lib/${SDK_ARCH}")

add_executable(detector_tuning detector_tuning.cpp)
target_link_libraries(detector_tuning PRIVATE camera detector)
set_target_properties(detector_tuning PROPERTIES BUILD_RPATH "${SDK}/lib/${SDK_ARCH}")

add_executable(capture capture.cpp)
target_link_libraries(capture PRIVATE camera)
set_target_properties(capture PROPERTIES BUILD_RPATH "${SDK}/lib/${SDK_ARCH}")

add_executable(camera_calib camera_calib.cpp)
target_link_libraries(camera_calib PRIVATE ${OpenCV_LIBS} yaml-cpp)
