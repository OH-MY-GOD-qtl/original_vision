// 真机调试：整条链路的量实时推到 PlotJuggler（UDP JSON），按 q 退出
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

#include "camera.hpp"
#include "detector.hpp"
#include "gimbal.hpp"
#include "solver.hpp"

namespace
{

// 世界系坐标 → 角度（度）
double yaw_deg(const Eigen::Vector3d &p) { return std::atan2(p.x(), p.z()) * 180 / CV_PI; }
double pitch_deg(const Eigen::Vector3d &p) { return std::atan2(p.y(), p.z()) * 180 / CV_PI; }

} // namespace

int main(int, char *argv[])
{
    const std::string config_path = argv[1];

    vision::Camera camera(config_path);
    vision::Detector detector(config_path);
    vision::Solver solver(config_path);
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

        const vision::Armor *target = nullptr; // 只可视化第一块解算成功的板
        for (const auto &armor : armors)
        {
            if (!armor.solved)
                continue;
            target = &armor;
            break;
        }

        // 没解算成功时角度填 0，用 solver/solved 判断该点是否有效
        const double target_yaw = target ? yaw_deg(target->xyz) : 0.0;
        const double target_pitch = target ? pitch_deg(target->xyz) : 0.0;
        const double gimbal_yaw = gimbal.yaw() * 180 / CV_PI;
        const double gimbal_pitch = gimbal.pitch() * 180 / CV_PI;
        const double time_s =
            std::chrono::duration<double>(frame.timestamp.time_since_epoch()).count();

        char json[640];
        std::snprintf(
            json, sizeof json,
            "{\"timestamp\":%.6f,\"data\":{"
            "\"loop/fps\":%.1f,"
            "\"detector/armors\":%zu,"
            "\"solver/solved\":%d,"
            "\"target/x\":%.4f,\"target/y\":%.4f,\"target/z\":%.4f,\"target/dist\":%.4f,"
            "\"target/yaw_deg\":%.3f,\"target/pitch_deg\":%.3f,"
            "\"gimbal/yaw_deg\":%.3f,\"gimbal/pitch_deg\":%.3f,"
            "\"err/yaw_deg\":%.3f,\"err/pitch_deg\":%.3f,"
            "\"gimbal/bullet_speed\":%.2f}}",
            time_s, 1 / dt, armors.size(), target ? 1 : 0, target ? target->xyz.x() : 0.0,
            target ? target->xyz.y() : 0.0, target ? target->xyz.z() : 0.0,
            target ? target->xyz.norm() : 0.0, target_yaw, target_pitch, gimbal_yaw, gimbal_pitch,
            target_yaw - gimbal_yaw, target_pitch - gimbal_pitch, gimbal.bullet_speed());

        ::sendto(sock, json, std::strlen(json), 0, reinterpret_cast<sockaddr *>(&address),
                 sizeof address);

        for (const auto &armor : armors) // 画面上也标一下，方便对照
        {
            for (size_t i = 0; i < 4; ++i)
                cv::line(frame.img, armor.corners[i], armor.corners[(i + 1) % 4],
                         armor.solved ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
        }

        cv::imshow("plot_test", frame.img);
        if (cv::waitKey(1) == 'q')
            break;
    }

    ::close(sock);
    return 0;
}
