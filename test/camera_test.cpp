// 真机集成测试：需要 MV-CS016-10UC 在场，按 q 退出

#include <chrono>
#include <cstdio>
#include <string>

#include <opencv2/opencv.hpp>

#include "camera.hpp"

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

int main(int, char *argv[])
{
    const std::string config_path = argv[1];

    vision::Camera camera(config_path); // 构造即打开并开始取流

    Viewer viewer;

    vision::Frame frame;

    camera.read(frame); // 首帧只用来定帧率基准
    auto last = frame.timestamp;

    while (true)
    {
        camera.read(frame);

        const double dt = std::chrono::duration<double>(frame.timestamp - last).count(); // 用相机时间戳算帧率
        last = frame.timestamp;
        std::printf("%.2f fps\n", 1 / dt);

        if (viewer.show(frame.img))
            break;
    }

    return 0;
}
