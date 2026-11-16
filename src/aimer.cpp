#include "aimer.hpp"

namespace vision
{

Aimer::Aimer(const std::string &) {} // TODO: 读弹道参数

Command Aimer::aim(const Target &, double) const
{
    return Command::idle(); // TODO: 重力补偿 + 迭代求角
}

} // namespace vision
