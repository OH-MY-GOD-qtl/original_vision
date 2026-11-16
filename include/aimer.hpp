#pragma once

#include <string>

#include "command.hpp"
#include "target.hpp"

namespace vision
{

// 瞄准：目标 → 云台指令，只管弹道，不管通信
class Aimer
{
  public:
    explicit Aimer(const std::string &config_path);

    Command aim(const Target &target, double bullet_speed) const;
};

} // namespace vision
