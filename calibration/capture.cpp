// 标定：采集棋盘格图像，识别到才允许存，按 s 存图，按 q 退出
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

#include "camera.hpp"

int main(int, char *argv[])
{
    const std::string config_path = argv[1];
    const std::string folder = argv[2];

    const YAML::Node yaml = YAML::LoadFile(config_path);
    const cv::Size pattern(yaml["pattern_cols"].as<int>(), yaml["pattern_rows"].as<int>());

    std::filesystem::create_directories(folder); // 已存在则什么都不做

    const std::string win = "capture: s 存图, q 退出";
    cv::namedWindow(win, cv::WINDOW_NORMAL);
    cv::resizeWindow(win, 960, 720);

    vision::Camera camera(config_path);
    vision::Frame frame;

    // 不加 FAST_CHECK：它会筛掉低对比或出画的板子
    const int flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;

    // 配置值优先，另一个是型号标法误读（标的是内角点而非格数）
    const std::vector<cv::Size> candidates = {pattern, {pattern.width + 1, pattern.height + 1}};

    int count = 0;
    while (true)
    {
        camera.read(frame);

        cv::Mat gray;
        cv::cvtColor(frame.img, gray, cv::COLOR_BGR2GRAY);

        cv::Mat view = frame.img.clone(); // 画在副本上，存下去的还是干净原图
        cv::Size hit(0, 0);               // 本帧匹配上的尺寸，0 表示没匹配
        std::vector<cv::Point2f> corners;
        for (const auto &size : candidates)
        {
            if (!cv::findChessboardCorners(gray, size, corners, flags))
                continue;
            hit = size;
            cv::drawChessboardCorners(view, size, corners, true);
            break;
        }

        const std::string size_text = std::to_string(hit.width) + "x" + std::to_string(hit.height);
        const std::string text = hit.width ? "match " + size_text : "no board";
        cv::putText(view, text, {20, 50}, cv::FONT_HERSHEY_SIMPLEX, 1.2,
                    hit == pattern ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
        cv::imshow(win, view);

        const int key = cv::waitKey(1);
        if (key == 'q')
            break;
        if (key != 's' || hit != pattern) // 只存配置尺寸的，其它尺寸存了也算不出内参
            continue;

        ++count;
        const std::string path = folder + "/" + std::to_string(count) + ".jpg";
        cv::imwrite(path, frame.img);
        std::printf("[%d] 已保存 %s\n", count, path.c_str());
    }

    return 0;
}
