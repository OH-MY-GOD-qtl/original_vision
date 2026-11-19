#include "camera.hpp"

#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

#include "MvCameraControl.h"

namespace
{

constexpr int warmup_frames = 100; // 对时预热帧数，实测 100 帧取到的最小值与 400 帧相差不到 25us

// 设备时间戳：高 32 位拼低 32 位，单位 10ns，标记曝光开始
unsigned long long device_time(const MV_FRAME_OUT &frame)
{
    return (static_cast<unsigned long long>(frame.stFrameInfo.nDevTimeStampHigh) << 32) |
           frame.stFrameInfo.nDevTimeStampLow;
}

// 设备时间戳（10ns 计数）转成时长
std::chrono::nanoseconds ticks(unsigned long long dev)
{
    return std::chrono::nanoseconds(static_cast<long long>(dev) * 10);
}

} // namespace

namespace vision
{

Camera::Camera(const std::string &config_path)
{
    const YAML::Node yaml = YAML::LoadFile(config_path); // 缺少配置项或类型错误会抛异常
    const double exposure_ms = yaml["exposure_ms"].as<double>();
    const double gain = yaml["gain"].as<double>();

    const double exposure_us = exposure_ms * 1000.0; // 配置为 ms，SDK 为 us
    exposure_half = std::chrono::microseconds(static_cast<long long>(exposure_us / 2));

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

    anchor_clock();
}

Camera::~Camera()
{
    MV_CC_StopGrabbing(handle);
    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);
    MV_CC_Finalize();
}

void Camera::anchor_clock()
{
    // 首帧带着流建立开销，拿它对时会让之后每一帧都偏晚几百微秒，取预热里最快的一帧
    auto anchor = std::chrono::steady_clock::time_point::max();
    for (int i = 0; i < warmup_frames; ++i)
    {
        MV_FRAME_OUT raw{};
        MV_CC_GetImageBuffer(handle, &raw, 1000);

        const auto now = std::chrono::steady_clock::now();
        const auto epoch = now - ticks(device_time(raw)); // 这一帧算出来的设备零点
        MV_CC_FreeImageBuffer(handle, &raw);

        if (epoch < anchor)
            anchor = epoch;
    }

    host_epoch = anchor - exposure_half * 2; // 到手是曝光结束，退掉一个曝光就是曝光开始
}

void Camera::read(Frame &frame)
{
    MV_FRAME_OUT raw{};
    MV_CC_GetImageBuffer(handle, &raw, 1000);

    const cv::Mat bayer(raw.stFrameInfo.nHeight, raw.stFrameInfo.nWidth, CV_8U, raw.pBufAddr);
    cv::cvtColor(bayer, frame.img, cv::COLOR_BayerRG2RGB); // 写进 frame.img，复用其内存

    // 设备戳打在曝光开始，加一半才是曝光中点
    // 转码成功后才写时间戳，图像与时间戳永远同一帧
    frame.timestamp = host_epoch + ticks(device_time(raw)) + exposure_half;

    MV_CC_FreeImageBuffer(handle, &raw);
}

} // namespace vision
