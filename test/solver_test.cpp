// 真机解算测试：需要 MV-CS016-10UC 在场，按 q 退出
#include <cmath>
#include <string>

#include <opencv2/opencv.hpp>

#include "camera.hpp"
#include "detector.hpp"
#include "solver.hpp"

int main(int, char *argv[])
{
    const std::string config_path = argv[1];

    vision::Camera camera(config_path);
    vision::Detector detector(config_path);
    vision::Solver solver(config_path);

    vision::Frame frame;
    cv::namedWindow("solver_test", cv::WINDOW_NORMAL);

    while (true)
    {
        camera.read(frame);

        auto armors = detector.detect(frame.img);
        for (auto &armor : armors) // 与总链路一致：检测完立刻解算
            solver.solve(armor);

        for (const auto &armor : armors)
        {
            if (!armor.solved)
                continue; // 解算失败的不画

            for (size_t i = 0; i < 4; ++i)
                cv::line(frame.img, armor.corners[i], armor.corners[(i + 1) % 4],
                         cv::Scalar(0, 255, 0), 2);

            const cv::Point2f center =
                (armor.corners[0] + armor.corners[1] + armor.corners[2] + armor.corners[3]) * 0.25f;
            const double yaw = std::atan2(armor.xyz.x(), armor.xyz.z()) * 180 / CV_PI;
            const double pitch = std::asin(armor.xyz.y() / armor.xyz.norm()) * 180 / CV_PI;

            cv::circle(frame.img, center, 4, cv::Scalar(0, 0, 255), -1);
            cv::putText(frame.img, cv::format("%.2fm y%.1f p%.1f", armor.xyz.norm(), yaw, pitch),
                        center, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
        }

        cv::imshow("solver_test", frame.img);
        if (cv::waitKey(1) == 'q')
            break;
    }

    return 0;
}
