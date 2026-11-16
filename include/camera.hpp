#pragma once

#include <opencv2/core.hpp>

namespace vision
{

// 相机参数，值全部来自配置文件
struct CameraConfig
{
    double exposure_ms; // 单位 ms
    double gain;        // 单位 dB
};

// 海康相机：构造即打开并取流，析构自动释放
class Camera
{
  public:
    explicit Camera(const CameraConfig &cfg);
    ~Camera();

    Camera(const Camera &) = delete; // 句柄独占，不许拷贝
    Camera &operator=(const Camera &) = delete;

    cv::Mat grab(); // 取一帧并转成 BGR，SDK 缓冲区在返回前归还

  private:
    void *handle = nullptr; // MV_CC 句柄，用 void* 保持头文件不依赖 SDK
};

} // namespace vision
