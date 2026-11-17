// 标定：曝光结束到主机拿到帧的传输延迟，结果写回 camera.yaml 的 transport_ms
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

#include "camera.hpp"

// 转码耗时：与 read() 内部同尺寸同类型的换算，它不属于传输延迟
static double measure_convert_ms(cv::Size size)
{
    cv::Mat bayer(size, CV_8U);
    cv::Mat bgr;

    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < 100; ++i)
        cv::cvtColor(bayer, bgr, cv::COLOR_BayerRG2RGB);

    const auto cost = std::chrono::steady_clock::now() - start;
    return std::chrono::duration<double, std::milli>(cost).count() / 100;
}

int main(int, char *argv[])
{
    const std::string path = argv[1];
    const double exposure_ms = YAML::LoadFile(path)["exposure_ms"].as<double>(); // 半曝光补偿要用

    vision::Camera camera(path);

    // 每帧的残差 = 此刻 - 时间戳 - 半曝光 = 传输延迟 + 转码耗时
    std::vector<double> residual;
    vision::Frame frame;
    for (int i = 0; i < 200; ++i)
    {
        camera.read(frame);
        const auto age = std::chrono::steady_clock::now() - frame.timestamp;
        residual.push_back(std::chrono::duration<double, std::milli>(age).count() - exposure_ms / 2);
    }

    const double convert_ms = measure_convert_ms(frame.img.size());

    // 取最小值：主机最空闲的那一帧几乎不排队，最接近真实传输延迟
    std::sort(residual.begin(), residual.end());
    const double transport_ms = std::max(0.0, residual.front() - convert_ms);

    std::printf("采样 %zu 帧，图像 %dx%d\n", residual.size(), frame.img.cols, frame.img.rows);
    std::printf("残差(传输+转码)  最小 %.3f  中位 %.3f  最大 %.3f ms\n", residual.front(),
                residual[residual.size() / 2], residual.back());
    std::printf("转码耗时 %.3f ms\n", convert_ms);
    std::printf("建议写入 camera.yaml：transport_ms: %.3f\n", transport_ms);
    if (residual.front() - convert_ms <= 0)
        std::printf("测量值已低于噪声，传输延迟可忽略，填 0 即可\n");

    return 0;
}
