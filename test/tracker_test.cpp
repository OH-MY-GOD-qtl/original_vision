// 真机跟踪测试：亮绿为本帧检测，深绿为滤波后位置，红框为飞行时间后的预瞄点，按 q 退出
#include <string>

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
            const auto now = solver.project(tracked->xyz); // 深绿：滤波后的当前位置
            for (size_t i = 0; i < 4; ++i)
                cv::line(frame.img, now[i], now[(i + 1) % 4], cv::Scalar(0, 128, 0), 2);

            const double flight = tracked->xyz.norm() / gimbal.bullet_speed(); // 弹丸飞行时间
            const auto future = solver.project(tracked->xyz + tracked->velocity * flight);
            for (size_t i = 0; i < 4; ++i) // 红：飞行时间后的预瞄点
                cv::line(frame.img, future[i], future[(i + 1) % 4], cv::Scalar(0, 0, 255), 2);

            const cv::Point2f ahead = (future[0] + future[1] + future[2] + future[3]) * 0.25f;
            cv::circle(frame.img, ahead, 6, cv::Scalar(0, 255, 255), -1); // 黄点：提前量的终点

            cv::putText(frame.img,
                        cv::format("%.2fm  v=%.2f  t=%.3fs  lead=%.2f", tracked->xyz.norm(),
                                   tracked->velocity.norm(), flight,
                                   tracked->velocity.norm() * flight),
                        ahead, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
        }

        cv::imshow("tracker_test", frame.img);
        if (cv::waitKey(1) == 'q')
            break;
    }

    return 0;
}
