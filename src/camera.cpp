#include "camera.hpp"

#include <opencv2/imgproc.hpp>

#include "MvCameraControl.h"

namespace vision
{

Camera::Camera(const CameraConfig &cfg)
{
    MV_CC_DEVICE_INFO_LIST list{};
    MV_CC_EnumDevices(MV_USB_DEVICE, &list);
    MV_CC_CreateHandle(&handle, list.pDeviceInfo[0]);

    MV_CC_OpenDevice(handle);
    MV_CC_StartGrabbing(handle);

    MV_CC_SetEnumValue(handle, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    MV_CC_SetFloatValue(handle, "ExposureTime", cfg.exposure_ms * 1000.0); // 配置为 ms，SDK 为 us
    MV_CC_SetEnumValue(handle, "GainAuto", MV_GAIN_MODE_OFF);
    MV_CC_SetFloatValue(handle, "Gain", cfg.gain);
    MV_CC_SetEnumValue(handle, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
}

Camera::~Camera()
{
    MV_CC_StopGrabbing(handle);
    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);
    MV_CC_Finalize();
}

cv::Mat Camera::grab()
{
    MV_FRAME_OUT frame{};
    MV_CC_GetImageBuffer(handle, &frame, 1000);

    const cv::Mat bayer(frame.stFrameInfo.nHeight, frame.stFrameInfo.nWidth, CV_8U,
                        frame.pBufAddr); // 零拷贝视图

    cv::Mat bgr;
    cv::cvtColor(bayer, bgr, cv::COLOR_BayerRG2RGB); // 相机相位与 OpenCV 命名相反，用 RG2RGB 才对

    MV_CC_FreeImageBuffer(handle, &frame);
    return bgr;
}

} // namespace vision
