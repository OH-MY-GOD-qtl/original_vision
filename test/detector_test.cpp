// 真机检测测试：绿框为检出的装甲板，按 q 退出
#include <cstdio>
#include <string>

#include <opencv2/opencv.hpp>

#include "camera.hpp"
#include "detector.hpp"
#include "viz.hpp"

int main(int, char *argv[])
{
    const std::string config_path = argv[1];

    vision::Camera camera(config_path);
    vision::Detector detector(config_path);

    vision::Frame frame;
    cv::namedWindow("detector_test", cv::WINDOW_NORMAL);

    while (true)
    {
        camera.read(frame);

        const auto armors = detector.detect(frame.img);

        for (const auto &armor : armors)
            vision::draw_quad(frame.img, armor.corners, vision::color_detected);

        std::printf("%zu armors\n", armors.size());

        cv::imshow("detector_test", frame.img);
        if (cv::waitKey(1) == 'q')
            break;
    }

    return 0;
}
