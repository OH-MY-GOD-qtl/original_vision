#pragma once

namespace vision
{

// 云台指令：绝对角度（rad）与弹道距离（m），纯数据
struct Command
{
    double yaw = 0.0;       // 相机光轴的世界偏航角，正为向右
    double pitch = 0.0;     // 相机光轴的世界俯仰角，正为向下
    double distance = 0.0;  // 弹道长度，m
    double flight = 0.0;    // 弹丸飞行时间，s
    double armor_yaw = 0.0; // 弹着时被瞄那块板的朝向，只给可视化用

    // 前馈：等 Tracker 出来再填，现在保持 0
    double yaw_vel = 0.0;
    double yaw_acc = 0.0;
    double pitch_vel = 0.0;
    double pitch_acc = 0.0;

    bool shoot = false;    // 是否开火
    bool is_empty = false; // 目标为空（丢失目标）
};

} // namespace vision
