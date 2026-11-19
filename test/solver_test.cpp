// 真机解算测试：绿框为解算成功的装甲板，洋红线为板面法线，按 q 退出
#include <cmath>
#include <string>

#include <opencv2/opencv.hpp>

#include "angle.hpp"
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
            const Eigen::Vector2d angles = vision::angles_of(armor.xyz);
            const double yaw = angles.x() * 180 / CV_PI;
            const double pitch = angles.y() * 180 / CV_PI;
            const double board = armor.yaw * 180 / CV_PI;

            // 洋红线：板面法线，指向相机方向；它和装甲板方位一致说明角度解算对
            const Eigen::Vector3d normal(std::sin(armor.yaw), 0.0, std::cos(armor.yaw));
            const auto tail = solver.project(armor.xyz + 0.15 * normal);
            const cv::Point2f tip = (tail[0] + tail[1] + tail[2] + tail[3]) * 0.25f;
            cv::line(frame.img, center, tip, cv::Scalar(255, 0, 255), 2);

            cv::circle(frame.img, center, 4, cv::Scalar(0, 255, 0), -1);
            const std::string text = cv::format("%.2fm yaw%.1f pitch%.1f board%.1f",
                                                armor.xyz.norm(), yaw, pitch, board);
            cv::putText(frame.img, text, center, cv::FONT_HERSHEY_SIMPLEX, 0.8,
                        cv::Scalar(0, 255, 255), 2);
        }

        cv::imshow("solver_test", frame.img);
        if (cv::waitKey(1) == 'q')
            break;
    }

    return 0;
}
