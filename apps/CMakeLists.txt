add_executable(auto_aim auto_aim.cpp)
target_link_libraries(auto_aim PRIVATE camera detector solver tracker aimer gimbal)
set_target_properties(auto_aim PROPERTIES BUILD_RPATH "${SDK}/lib/${SDK_ARCH}")
