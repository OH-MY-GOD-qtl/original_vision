#pragma once

#include <chrono>
#include <string>

#include <opencv2/core.hpp>

namespace vision
{

// 一帧：图像与曝光中点时刻绑在一起，不会配错
struct Frame
{
    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;
};

// 海康相机：构造即打开、取流、对时，析构自动释放
class Camera
{
  public:
    explicit Camera(const std::string &config_path);
    ~Camera();

    Camera(const Camera &) = delete; // 句柄独占，不许拷贝
    Camera &operator=(const Camera &) = delete;

    void read(Frame &frame); // 写入 frame.img（复用其内存）与 frame.timestamp

  private:
    void anchor_clock(); // 预热若干帧，用最快的一帧把设备时钟对到主机时钟

    void *handle = nullptr;                             // MV_CC 句柄，用 void* 保持头文件不依赖 SDK
    std::chrono::microseconds exposure_half{0};         // 设备戳标记曝光开始，加回一半才是曝光中点
    std::chrono::steady_clock::time_point host_epoch{}; // 设备时钟零点对应的主机时刻
};

} // namespace vision
