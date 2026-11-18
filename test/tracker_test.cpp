// 真机跟踪测试：绿框为本帧检测，红框为滤波后的位置，黄点为速度外推，按 q 退出
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
            const auto corners = solver.project(tracked->xyz); // 红框：滤波后的位置
            for (size_t i = 0; i < 4; ++i)
                cv::line(frame.img, corners[i], corners[(i + 1) % 4], cv::Scalar(0, 0, 255), 2);

            const auto future = solver.project(tracked->xyz + tracked->velocity * 0.3);
            const cv::Point2f ahead = (future[0] + future[1] + future[2] + future[3]) * 0.25f;
            cv::circle(frame.img, ahead, 6, cv::Scalar(0, 255, 255), -1); // 黄点：0.3s 后

            cv::putText(frame.img, cv::format("%.2fm  v=%.2f", tracked->xyz.norm(),
                                              tracked->velocity.norm()),
                        ahead, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
        }

        cv::imshow("tracker_test", frame.img);
        if (cv::waitKey(1) == 'q')
            break;
    }

    return 0;
}
