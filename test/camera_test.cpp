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
    return argc > 1 ? argv[1] : "configs/camera.yaml";
}

// 配置：只保存参数，不碰硬件
static vision::CameraConfig load_config(const std::string &path)
{
    const YAML::Node yaml = YAML::LoadFile(path); // 缺少配置项或类型错误会抛异常

    vision::CameraConfig cfg;
    cfg.exposure_ms = yaml["exposure_ms"].as<double>();
    cfg.gain = yaml["gain"].as<double>();
    cfg.transport_ms = yaml["transport_ms"].as<double>();
    return cfg;
}

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

    auto last = camera.grab().timestamp; // 首帧只用来定帧率基准

    while (true)
    {
        const vision::Frame frame = camera.grab(); // 一帧 BGR + 曝光中点时刻

        const double dt = std::chrono::duration<double>(frame.timestamp - last).count(); // 用相机时间戳算帧率
        last = frame.timestamp;
        std::printf("%.2f fps\n", 1 / dt);

        if (viewer.show(frame.img))
            break;
    }

    return 0;
}
