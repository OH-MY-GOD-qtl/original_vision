#include "gimbal.hpp"

#include <yaml-cpp/yaml.h>

namespace vision
{

Gimbal::Gimbal(const std::string &config_path)
{
    const YAML::Node yaml = YAML::LoadFile(config_path); // 缺少配置项或类型错误会抛异常
    bullet_speed_ = yaml["bullet_speed"].as<double>();
}

void Gimbal::send(const Command &) {} // TODO: 打包 + CRC

} // namespace vision
