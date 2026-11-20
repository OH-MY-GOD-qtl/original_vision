// 真机调试：绿框检测，深绿框滤波后位置，数据推 PlotJuggler（UDP JSON），按 q 退出
// 只收不发，不会指挥云台
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

#include "angle.hpp"
#include "camera.hpp"
#include "detector.hpp"
#include "gimbal.hpp"
#include "solver.hpp"
#include "tracker.hpp"
#include "viz.hpp"

int main(int, char *argv[])
{
    const std::string config_path = argv[1];

    vision::Camera camera(config_path);
    vision::Detector detector(config_path);
    vision::Solver solver(config_path);
    vision::Tracker tracker(config_path);
    vision::Gimbal gimbal(config_path);

    // 每条数据报一个 JSON：{"timestamp": 秒, "data": {键: 值}}
    const int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(9870); // 与 PlotJuggler 的 UDP Server 端口一致
    ::inet_pton(AF_INET, "127.0.0.1", &address.sin_addr);

    cv::namedWindow("plot_test", cv::WINDOW_NORMAL);

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

        const auto tracked = tracker.track(armors, frame.timestamp); // 跟踪的预测位置

        const vision::Armor *target = nullptr; // 只可视化第一块解算成功的板
        for (const auto &armor : armors)
        {
            if (!armor.solved)
                continue;
            target = &armor;
            break;
        }

        // 没解算成功时角度填 0，用 solver/solved 判断该点是否有效
        const Eigen::Vector2d angles =
            target ? vision::angles_of(target->xyz) : Eigen::Vector2d::Zero();
        const double target_yaw = angles.x() * 180 / CV_PI;
        const double target_pitch = angles.y() * 180 / CV_PI;
        const double gimbal_yaw = gimbal.yaw() * 180 / CV_PI;
        const double gimbal_pitch = gimbal.pitch() * 180 / CV_PI;
        const double gimbal_yaw_vel = gimbal.yaw_vel() * 180 / CV_PI;
        const double gimbal_pitch_vel = gimbal.pitch_vel() * 180 / CV_PI;
        const double time_s =
            std::chrono::duration<double>(frame.timestamp.time_since_epoch()).count();

        char json[1024];
        std::snprintf(
            json, sizeof json,
            "{\"timestamp\":%.6f,\"data\":{"
            "\"loop/fps\":%.1f,"
            "\"detector/armors\":%zu,"
            "\"solver/solved\":%d,"
            "\"target/x\":%.4f,\"target/y\":%.4f,\"target/z\":%.4f,\"target/dist\":%.4f,"
            "\"target/yaw_deg\":%.3f,\"target/pitch_deg\":%.3f,"
            "\"predict/dist\":%.4f,"
            "\"predict/theta_deg\":%.3f,\"predict/omega_deg\":%.3f,"
            "\"gimbal/yaw_deg\":%.3f,\"gimbal/pitch_deg\":%.3f,"
            "\"gimbal/yaw_vel_deg\":%.3f,\"gimbal/pitch_vel_deg\":%.3f,"
            "\"err/yaw_deg\":%.3f,\"err/pitch_deg\":%.3f,"
            "\"gimbal/bullet_speed\":%.2f}}",
            time_s, 1 / dt, armors.size(), target ? 1 : 0, target ? target->xyz.x() : 0.0,
            target ? target->xyz.y() : 0.0, target ? target->xyz.z() : 0.0,
            target ? target->xyz.norm() : 0.0, target_yaw, target_pitch,
            tracked ? tracked->armor(0.0).norm() : 0.0,
            tracked ? tracked->theta * 180 / CV_PI : 0.0,
            tracked ? tracked->omega * 180 / CV_PI : 0.0, gimbal_yaw, gimbal_pitch, gimbal_yaw_vel,
            gimbal_pitch_vel, target_yaw - gimbal_yaw, target_pitch - gimbal_pitch,
            gimbal.bullet_speed());

        ::sendto(sock, json, std::strlen(json), 0, reinterpret_cast<sockaddr *>(&address),
                 sizeof address);

        for (const auto &armor : armors) // 画面上的检测结果
            vision::draw_quad(frame.img, armor.corners,
                              armor.solved ? vision::color_detected : vision::color_unsolved);

        if (tracked)
        {
            const auto corners = solver.project(tracked->armor(0.0), tracked->armor_yaw(0.0));
            vision::draw_quad(frame.img, corners, vision::color_tracked); // 深绿：滤波后的当前位置
        }

        cv::imshow("plot_test", frame.img);
        if (cv::waitKey(1) == 'q')
            break;
    }

    ::close(sock);
    return 0;
}
