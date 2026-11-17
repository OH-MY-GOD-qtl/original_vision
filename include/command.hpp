#pragma once

namespace vision
{

// 云台指令：绝对角度（rad）与弹道距离（m），纯数据
struct Command
{
    double yaw = 0.0;
    double pitch = 0.0;
    double distance = 0.0;
};

} // namespace vision
