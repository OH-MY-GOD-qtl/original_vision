// 标定：时间戳链路自检，验证曝光中点估计够稳、设备时钟相对主机不漂
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "camera.hpp"

int main(int, char *argv[])
{
    const std::string path = argv[1];
    const double exposure_ms = YAML::LoadFile(path)["exposure_ms"].as<double>();

    vision::Camera camera(path); // 构造里已预热并对好时

    std::vector<double> host_ms;  // 每帧到手的时刻
    std::vector<double> stamp_ms; // 每帧的曝光中点
    vision::Frame frame;
    for (int i = 0; i < 600; ++i)
    {
        camera.read(frame);
        const auto now = std::chrono::steady_clock::now();
        host_ms.push_back(
            std::chrono::duration<double, std::milli>(now.time_since_epoch()).count());
        stamp_ms.push_back(
            std::chrono::duration<double, std::milli>(frame.timestamp.time_since_epoch()).count());
    }

    std::vector<double> interval; // 帧间隔，只取决于设备时钟，应当恒定
    std::vector<double> latency;  // 到手延迟 = 曝光半程 + 传输 + 转码
    for (size_t i = 1; i < host_ms.size(); ++i)
    {
        interval.push_back(stamp_ms[i] - stamp_ms[i - 1]);
        latency.push_back(host_ms[i] - stamp_ms[i]);
    }
    std::sort(interval.begin(), interval.end());
    std::sort(latency.begin(), latency.end());

    // 最小二乘斜率：设备走 1ms 时主机走多少 ms，比首尾相除抗单帧排队
    const double n = static_cast<double>(host_ms.size());
    double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
    for (size_t i = 0; i < host_ms.size(); ++i)
    {
        const double x = host_ms[i] - host_ms[0];
        const double y = stamp_ms[i] - stamp_ms[0];
        sx += x;
        sy += y;
        sxx += x * x;
        sxy += x * y;
    }
    const double slope = (n * sxy - sx * sy) / (n * sxx - sx * sx);
    const double drift_ppm = (slope - 1.0) * 1e6;
    const double span = host_ms.back() - host_ms.front();

    std::printf("采样 %zu 帧，曝光 %.3f ms\n", host_ms.size(), exposure_ms);
    std::printf("帧间隔   %.3f ~ %.3f ms  中位 %.3f\n", interval.front(), interval.back(),
                interval[interval.size() / 2]);
    std::printf("到手延迟  %.3f ~ %.3f ms  中位 %.3f  含曝光半程 %.3f\n", latency.front(),
                latency.back(), latency[latency.size() / 2], exposure_ms / 2);
    std::printf("时钟漂移  %+.1f ppm  整段 %.1f s 累积 %.2f ms\n", drift_ppm, span / 1000,
                span * drift_ppm / 1e6);

    return 0;
}
