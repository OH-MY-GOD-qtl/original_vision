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

// 海康相机：构造即打开并取流，析构自动释放
class Camera
{
  public:
    explicit Camera(const std::string &config_path);
    ~Camera();

    Camera(const Camera &) = delete; // 句柄独占，不许拷贝
    Camera &operator=(const Camera &) = delete;

    void read(Frame &frame); // 写入 frame.img（复用其内存）与 frame.timestamp

  private:
    void sync_clock(unsigned long long dev_ts); // 首帧把设备时钟对到主机时钟

    void *handle = nullptr;                             // MV_CC 句柄，用 void* 保持头文件不依赖 SDK
    std::chrono::microseconds exposure_half{0};         // 设备戳标记曝光开始，加回一半才是曝光中点
    std::chrono::microseconds transport{0};             // 曝光结束到主机拿到帧的传输延迟
    std::chrono::steady_clock::time_point host_epoch{}; // 首帧的主机时刻
    unsigned long long dev_epoch = 0;                   // 首帧的设备时刻，单位 10ns
    bool clock_synced = false;                          // 首帧是否已完成对时
};

} // namespace vision
