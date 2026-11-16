add_executable(transport_delay transport_delay.cpp)
target_link_libraries(transport_delay PRIVATE camera yaml-cpp)
set_target_properties(transport_delay PROPERTIES BUILD_RPATH "${SDK}/lib/${SDK_ARCH}")
