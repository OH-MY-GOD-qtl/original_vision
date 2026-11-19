// 真机全链路可视化：亮绿检测 / 深绿滤波 / 红框瞄准点 / 青色十字为指令指向
// 同时把数据推到 PlotJuggler（UDP JSON），只算不发，按 q 退出
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>

#include "aimer.hpp"
#include "camera.hpp"
#include "detector.hpp"
#include "gimbal.hpp"
#include "solver.hpp"
#include "tracker.hpp"

namespace
{

constexpr double rad2deg = 180.0 / CV_PI; // 串口上下行都是弧度，只有上屏才转度

// 世界系角度 → 单位方向，即云台模型 Ry(yaw)*Rx(-pitch) 的前进轴
Eigen::Vector3d direction(double yaw, double pitch)
{
    return {std::sin(yaw) * std::cos(pitch), std::sin(pitch), std::cos(yaw) * std::cos(pitch)};
}

cv::Point2f center_of(const std::array<cv::Point2f, 4> &corners)
{
    return (corners[0] + corners[1] + corners[2] + corners[3]) * 0.25f;
}

void draw(cv::Mat &img, const std::array<cv::Point2f, 4> &corners, const cv::Scalar &color)
{
    for (size_t i = 0; i < 4; ++i)
        cv::line(img, corners[i], corners[(i + 1) % 4], color, 2);
}

} // namespace

int main(int, char *argv[])
{
    const std::string config_path = argv[1];

    vision::Camera camera(config_path);
    vision::Detector detector(config_path);
    vision::Solver solver(config_path);
    vision::Tracker tracker(config_path);
    vision::Aimer aimer(config_path);
    vision::Gimbal gimbal(config_path);

    // 每条数据报一个 JSON：{"timestamp": 秒, "data": {键: 值}}
    const int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(9870); // 与 PlotJuggler 的 UDP Server 端口一致
    ::inet_pton(AF_INET, "127.0.0.1", &address.sin_addr);

    cv::namedWindow("auto_aim_test", cv::WINDOW_NORMAL);

    vision::Frame frame;
    camera.read(frame); // 首帧只用来定帧率基准
    auto last = frame.timestamp;

    while (true)
    {
        camera.read(frame);
        const double dt = std::chrono::duration<double>(frame.timestamp - last).count();
        last = frame.timestamp;

        gimbal.read();
        solver.set_attitude(gimbal.yaw(), gimbal.pitch());

        auto armors = detector.detect(frame.img);
        for (auto &armor : armors)
            solver.solve(armor);

        const vision::Armor *measured = nullptr; // 本次的观测，取第一块解算成功的
        for (const auto &armor : armors)
        {
            if (!armor.solved)
                continue;
            measured = &armor;
            break;
        }

        const auto tracked = tracker.track(armors, frame.timestamp);
        const auto command = aimer.aim(tracked, gimbal.bullet_speed(), frame.timestamp);

        const double gimbal_yaw = gimbal.yaw() * rad2deg;
        const double gimbal_pitch = gimbal.pitch() * rad2deg;
        const double gimbal_yaw_vel = gimbal.yaw_vel() * rad2deg;
        const double gimbal_pitch_vel = gimbal.pitch_vel() * rad2deg;
        const double cmd_yaw = command.yaw * rad2deg;
        const double cmd_pitch = command.pitch * rad2deg;
        const double cmd_yaw_vel = command.yaw_vel * rad2deg;
        const double cmd_yaw_acc = command.yaw_acc * rad2deg;
        const double cmd_pitch_vel = command.pitch_vel * rad2deg;
        const double cmd_pitch_acc = command.pitch_acc * rad2deg;
        const int mode = command.shoot ? 2 : 1; // 与 Gimbal::send 的 mode 规则一致
        const double time_s =
            std::chrono::duration<double>(frame.timestamp.time_since_epoch()).count();

        char json[1024];
        std::snprintf(
            json, sizeof json,
            "{\"timestamp\":%.6f,\"data\":{"
            "\"loop/fps\":%.1f,"
            "\"detector/armors\":%zu,"
            "\"solver/solved\":%d,"
            "\"target/dist\":%.4f,"
            "\"predict/dist\":%.4f,"
            "\"predict/theta_deg\":%.3f,\"predict/omega_deg\":%.3f,"
            "\"tx/mode\":%d,"
            "\"tx/yaw_deg\":%.3f,\"tx/pitch_deg\":%.3f,\"tx/dist\":%.4f,"
            "\"tx/yaw_vel_deg\":%.3f,\"tx/yaw_acc_deg\":%.3f,"
            "\"tx/pitch_vel_deg\":%.3f,\"tx/pitch_acc_deg\":%.3f,"
            "\"tx/shoot\":%d,\"tx/is_empty\":%d,"
            "\"gimbal/yaw_deg\":%.3f,\"gimbal/pitch_deg\":%.3f,"
            "\"gimbal/yaw_vel_deg\":%.3f,\"gimbal/pitch_vel_deg\":%.3f,"
            "\"err/yaw_deg\":%.3f,\"err/pitch_deg\":%.3f,"
            "\"gimbal/bullet_speed\":%.2f}}",
            time_s, 1 / dt, armors.size(), measured ? 1 : 0, measured ? measured->xyz.norm() : 0.0,
            tracked ? tracked->armor(0.0).norm() : 0.0,
            tracked ? tracked->theta * 180 / CV_PI : 0.0,
            tracked ? tracked->omega * 180 / CV_PI : 0.0, mode, cmd_yaw, cmd_pitch,
            command.distance, cmd_yaw_vel, cmd_yaw_acc, cmd_pitch_vel, cmd_pitch_acc,
            command.shoot ? 1 : 0, command.is_empty ? 1 : 0, gimbal_yaw, gimbal_pitch,
            gimbal_yaw_vel, gimbal_pitch_vel, cmd_yaw - gimbal_yaw, cmd_pitch - gimbal_pitch,
            gimbal.bullet_speed());

        ::sendto(sock, json, std::strlen(json), 0, reinterpret_cast<sockaddr *>(&address),
                 sizeof address);

        for (const auto &armor : armors) // 亮绿：本帧检测且解算成功
        {
            if (armor.solved)
                draw(frame.img, armor.corners, cv::Scalar(0, 255, 0));
        }

        if (tracked) // 深绿：滤波后的位置
            draw(frame.img, solver.project(tracked->armor(0.0)), cv::Scalar(0, 128, 0));

        // 指令方向在瞄准距离处的装甲板位置，归零期没有距离，取 5m 兜底
        const double distance = command.distance > 0.0 ? command.distance : 5.0;
        const auto aim = solver.project(direction(command.yaw, command.pitch) * distance);
        draw(frame.img, aim, cv::Scalar(0, 0, 255)); // 红框：瞄准框

        const auto at = center_of(aim);
        const cv::Scalar cyan(255, 255, 0);
        cv::circle(frame.img, at, 10, cyan, 2); // 青色十字：指令指向的中心
        cv::line(frame.img, at - cv::Point2f(16, 0), at + cv::Point2f(16, 0), cyan, 1);
        cv::line(frame.img, at - cv::Point2f(0, 16), at + cv::Point2f(0, 16), cyan, 1);

        cv::putText(frame.img,
                    cv::format("yaw %+.2f  pitch %+.2f  d %.2f  %s", cmd_yaw, cmd_pitch,
                               command.distance, command.is_empty ? "lost" : "hold"),
                    at + cv::Point2f(-200, 34), cv::FONT_HERSHEY_SIMPLEX, 0.7, cyan, 2);
        cv::putText(frame.img,
                    cv::format("mode %d  shoot %d  empty %d", mode, command.shoot ? 1 : 0,
                               command.is_empty ? 1 : 0),
                    at + cv::Point2f(-200, 66), cv::FONT_HERSHEY_SIMPLEX, 0.7, cyan, 2);

        cv::imshow("auto_aim_test", frame.img);
        if (cv::waitKey(1) == 'q')
            break;
    }

    ::close(sock);
    return 0;
}
