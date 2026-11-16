// 真机集成测试：需要 MV-CS016-10UC 在场，按 q 退出

#include <cstdio>
#include <string>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "camera.hpp"

// 参数：取配置文件路径，缺省用默认值
static std::string parse_config_path(int argc, char *argv[])
{
    const std::string keys = "{@config-path   | configs/camera.yaml | yaml配置文件路径 }";
    cv::CommandLineParser cli(argc, argv, keys);
    return cli.get<std::string>(0);
}

// 配置：只保存参数，不碰硬件
static vision::CameraConfig load_config(const std::string &path)
{
    const YAML::Node yaml = YAML::LoadFile(path); // 缺少配置项或类型错误会抛异常

    vision::CameraConfig cfg;
    cfg.exposure_ms = yaml["exposure_ms"].as<double>();
    cfg.gain = yaml["gain"].as<double>();
    return cfg;
}

// 帧率：每次 tick 返回距上次的瞬时值
struct FpsMeter
{
    std::chrono::steady_clock::time_point last = std::chrono::steady_clock::now();

    double tick()
    {
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> dt = now - last;
        last = now;
        return 1 / dt.count();
    }
};

// 显示：窗口、刷新、按键退出
struct Viewer
{
    Viewer() { cv::namedWindow("img", cv::WINDOW_NORMAL); }

    bool show(const cv::Mat &img) // 返回 true 表示用户按 q 退出
    {
        cv::imshow("img", img);
        return cv::waitKey(1) == 'q';
    }
};

int main(int argc, char *argv[])
{
    const std::string config_path = parse_config_path(argc, argv);

    const vision::CameraConfig cfg = load_config(config_path);

    vision::Camera camera(cfg); // 构造即打开并开始取流

    Viewer viewer;

    FpsMeter fps;

    while (true)
    {
        const cv::Mat img = camera.grab(); // 一帧 BGR，SDK 缓冲区已归还

        std::printf("%.2f fps\n", fps.tick());

        if (viewer.show(img))
            break;
    }

    return 0;
}
