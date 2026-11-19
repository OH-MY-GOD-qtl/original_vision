// 真机跟踪测试：绿框检测，深绿框滤波后位置，红框红点击打位置，按 q 退出
#include <cmath>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "camera.hpp"
#include "detector.hpp"
#include "gimbal.hpp"
#include "solver.hpp"
#include "tracker.hpp"

int main(int, char *argv[])
{
    const std::string config_path = argv[1];

    vision::Camera camera(config_path);
    vision::Detector detector(config_path);
    vision::Solver solver(config_path);
    vision::Tracker tracker(config_path);
    vision::Gimbal gimbal(config_path);

    cv::namedWindow("tracker_test", cv::WINDOW_NORMAL);

    vision::Frame frame;
    while (true)
    {
        camera.read(frame);
        gimbal.read();
        solver.set_attitude(gimbal.yaw(), gimbal.pitch());

        auto armors = detector.detect(frame.img);
        for (auto &armor : armors)
            solver.solve(armor);

        const auto tracked = tracker.track(armors, frame.timestamp);

        for (const auto &armor : armors) // 绿框：本帧检测且解算成功
        {
            if (!armor.solved)
                continue;
            for (size_t i = 0; i < 4; ++i)
                cv::line(frame.img, armor.corners[i], armor.corners[(i + 1) % 4],
                         cv::Scalar(0, 255, 0), 2);
        }

        if (tracked)
        {
            // 蓝圈：装甲板绕的圆，白点圆心，一眼看出转速估计对不对
            std::vector<cv::Point> ring; // polylines 只收整数点
            for (int i = 0; i <= 36; ++i)
            {
                const double a = i * CV_PI / 18;
                const Eigen::Vector3d p = tracked->center + tracked->radius * Eigen::Vector3d(
                                                                std::sin(a), 0, std::cos(a));
                const auto corner = solver.project(p);
                const cv::Point2f point = (corner[0] + corner[1] + corner[2] + corner[3]) * 0.25f;
                ring.emplace_back(cvRound(point.x), cvRound(point.y));
            }
            cv::polylines(frame.img, ring, false, cv::Scalar(255, 128, 0), 1);
            const auto middle = solver.project(tracked->center);
            const cv::Point2f center = (middle[0] + middle[1] + middle[2] + middle[3]) * 0.25f;
            cv::circle(frame.img, center, 3, cv::Scalar(255, 255, 255), -1);

            const auto now = solver.project(tracked->armor(0.0)); // 深绿：滤波后的当前位置
            for (size_t i = 0; i < 4; ++i)
                cv::line(frame.img, now[i], now[(i + 1) % 4], cv::Scalar(0, 128, 0), 2);

            const double flight = tracked->armor(0.0).norm() / gimbal.bullet_speed(); // 飞行时间
            const auto future = solver.project(tracked->armor(flight));
            for (size_t i = 0; i < 4; ++i) // 红：击打位置
                cv::line(frame.img, future[i], future[(i + 1) % 4], cv::Scalar(0, 0, 255), 2);

            const cv::Point2f ahead = (future[0] + future[1] + future[2] + future[3]) * 0.25f;
            cv::circle(frame.img, ahead, 4, cv::Scalar(0, 0, 255), -1); // 红点：击打点

            cv::putText(
                frame.img,
                cv::format("%.2fm  v=%.2f  w=%.2frad/s  t=%.3fs", tracked->armor(0.0).norm(),
                           tracked->armor_velocity(0.0).norm(), tracked->omega, flight),
                ahead, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
        }

        cv::imshow("tracker_test", frame.img);
        if (cv::waitKey(1) == 'q')
            break;
    }

    return 0;
}
