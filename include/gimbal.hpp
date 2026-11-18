#pragma once

#include <cstdint>
#include <string>

#include "command.hpp"

namespace vision
{

// 云台：指令 ↔ 下位机串口，只管通信，不管弹道
class Gimbal
{
  public:
    explicit Gimbal(const std::string &config_path);
    ~Gimbal();

    Gimbal(const Gimbal &) = delete; // 句柄独占，不许拷贝
    Gimbal &operator=(const Gimbal &) = delete;

    void read(); // 非阻塞读完串口，状态取最新一帧
    void send(const Command &command);

    double bullet_speed() const { return bullet_speed_; }
    double yaw() const { return yaw_; }
    double pitch() const { return pitch_; }

  private:
    int fd_ = -1;    // 串口文件描述符
    std::string rx_; // 尚未凑满一帧的残留字节

    uint8_t mode_ = 0;    // 下位机当前模式：0 空闲 1 自瞄 2 小符 3 大符
    double yaw_ = 0.0;    // 相机光轴的世界偏航角，rad
    double pitch_ = 0.0;  // 相机光轴的世界俯仰角，rad
    double bullet_speed_; // 弹速，m/s
};

} // namespace vision
