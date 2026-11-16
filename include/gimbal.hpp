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
    double bullet_speed_ = 0.0; // TODO: 从配置或下位机读数
};

} // namespace vision
