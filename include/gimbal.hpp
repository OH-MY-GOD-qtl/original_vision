#pragma once

#include <string>

#include "command.hpp"

namespace vision
{

// 云台：指令 → 下位机，只管通信，不管弹道
class Gimbal
{
  public:
    explicit Gimbal(const std::string &config_path);

    void send(const Command &command);

    double bullet_speed() const { return bullet_speed_; }

  private:
    double bullet_speed_; // 弹速，单位 m/s
};

} // namespace vision
