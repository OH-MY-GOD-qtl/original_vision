#include "camera.hpp"

#include <opencv2/imgproc.hpp>

#include "MvCameraControl.h"

namespace
{

// 设备时间戳：高 32 位拼低 32 位，单位 10ns，标记曝光开始
unsigned long long device_time(const MV_FRAME_OUT &frame)
{
    return (static_cast<unsigned long long>(frame.stFrameInfo.nDevTimeStampHigh) << 32) |
           frame.stFrameInfo.nDevTimeStampLow;
}

// 转码：BayerRG8 转三通道 BGR，零拷贝视图用完即弃
cv::Mat to_bgr(const MV_FRAME_OUT &frame)
{
    const cv::Mat bayer(frame.stFrameInfo.nHeight, frame.stFrameInfo.nWidth, CV_8U, frame.pBufAddr);

    cv::Mat bgr;
    cv::cvtColor(bayer, bgr, cv::COLOR_BayerRG2RGB); // 相机相位与 OpenCV 命名相反，用 RG2RGB 才对
    return bgr;
}

} // namespace

namespace vision
{

Camera::Camera(const CameraConfig &cfg)
{
    MV_CC_DEVICE_INFO_LIST list{};
    MV_CC_EnumDevices(MV_USB_DEVICE, &list);
    MV_CC_CreateHandle(&handle, list.pDeviceInfo[0]);

    MV_CC_OpenDevice(handle);

    const double exposure_us = cfg.exposure_ms * 1000.0; // 配置为 ms，SDK 为 us
    exposure_half = std::chrono::microseconds(static_cast<long long>(exposure_us / 2));
    transport = std::chrono::microseconds(static_cast<long long>(cfg.transport_ms * 1000.0));

    MV_CC_SetEnumValue(handle, "PixelFormat", PixelType_Gvsp_BayerRG8); // grab() 的转码依赖它
    MV_CC_SetEnumValue(handle, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    MV_CC_SetFloatValue(handle, "ExposureTime", exposure_us);
    MV_CC_SetEnumValue(handle, "GainAuto", MV_GAIN_MODE_OFF);
    MV_CC_SetFloatValue(handle, "Gain", cfg.gain);
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

Frame Camera::grab()
{
    MV_FRAME_OUT frame{};
    MV_CC_GetImageBuffer(handle, &frame, 1000);

    const unsigned long long dev_ts = device_time(frame);
    sync_clock(dev_ts);

    const auto since_epoch = std::chrono::nanoseconds(static_cast<long long>(dev_ts - dev_epoch) * 10);
    const auto timestamp = host_epoch + since_epoch + exposure_half; // 曝光开始加一半即曝光中点

    cv::Mat bgr = to_bgr(frame);
    MV_CC_FreeImageBuffer(handle, &frame);

    return {bgr, timestamp};
}

} // namespace vision
