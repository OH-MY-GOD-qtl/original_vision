#include "camera.hpp"

#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

#include "MvCameraControl.h"

namespace
{

// 设备时间戳：高 32 位拼低 32 位，单位 10ns，标记曝光开始
unsigned long long device_time(const MV_FRAME_OUT &frame)
{
    return (static_cast<unsigned long long>(frame.stFrameInfo.nDevTimeStampHigh) << 32) |
           frame.stFrameInfo.nDevTimeStampLow;
}

} // namespace

namespace vision
{

Camera::Camera(const std::string &config_path)
{
    const YAML::Node yaml = YAML::LoadFile(config_path); // 缺少配置项或类型错误会抛异常
    const double exposure_ms = yaml["exposure_ms"].as<double>();
    const double gain = yaml["gain"].as<double>();
    const double transport_ms = yaml["transport_ms"].as<double>();

    const double exposure_us = exposure_ms * 1000.0; // 配置为 ms，SDK 为 us
    exposure_half = std::chrono::microseconds(static_cast<long long>(exposure_us / 2));
    transport = std::chrono::microseconds(static_cast<long long>(transport_ms * 1000.0));

    MV_CC_DEVICE_INFO_LIST list{};
    MV_CC_EnumDevices(MV_USB_DEVICE, &list);
    MV_CC_CreateHandle(&handle, list.pDeviceInfo[0]);

    MV_CC_OpenDevice(handle);

    MV_CC_SetEnumValue(handle, "PixelFormat", PixelType_Gvsp_BayerRG8); // read() 的转码依赖它
    MV_CC_SetEnumValue(handle, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    MV_CC_SetFloatValue(handle, "ExposureTime", exposure_us);
    MV_CC_SetEnumValue(handle, "GainAuto", MV_GAIN_MODE_OFF);
    MV_CC_SetFloatValue(handle, "Gain", gain);
    MV_CC_SetEnumValue(handle, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);

    MV_CC_StartGrabbing(handle); // 参数全部下发后才开流
}

Camera::~Camera()
{
    MV_CC_StopGrabbing(handle);
    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);
    MV_CC_Finalize();
}

void Camera::sync_clock(unsigned long long dev_ts)
{
    if (clock_synced)
        return;

    // 此刻帧已传完，减掉一个曝光和传输才是它的曝光开始
    dev_epoch = dev_ts;
    host_epoch = std::chrono::steady_clock::now() - exposure_half * 2 - transport;
    clock_synced = true;
}

void Camera::read(Frame &frame)
{
    MV_FRAME_OUT raw{};
    MV_CC_GetImageBuffer(handle, &raw, 1000);

    const unsigned long long dev_ts = device_time(raw);
    sync_clock(dev_ts);

    const auto since_epoch = std::chrono::nanoseconds(static_cast<long long>(dev_ts - dev_epoch) * 10);
    const auto timestamp = host_epoch + since_epoch + exposure_half; // 曝光开始加一半即曝光中点

    const cv::Mat bayer(raw.stFrameInfo.nHeight, raw.stFrameInfo.nWidth, CV_8U, raw.pBufAddr);
    cv::cvtColor(bayer, frame.img, cv::COLOR_BayerRG2RGB); // 写进 frame.img，复用其内存

    frame.timestamp = timestamp; // 转码成功后才与图像一起更新，两者永远同一帧

    MV_CC_FreeImageBuffer(handle, &raw);
}

} // namespace vision
