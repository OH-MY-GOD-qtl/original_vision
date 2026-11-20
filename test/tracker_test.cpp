// 真机跟踪测试：绿框检测，深绿框滤波后位置，红框红点击打位置，按 q 退出
#include <cmath>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "aimer.hpp"
#include "angle.hpp"
#include "camera.hpp"
#include "detector.hpp"
#include "gimbal.hpp"
#include "solver.hpp"
#include "tracker.hpp"
#include "viz.hpp"

namespace
{

constexpr double lead_scale = 3.0; // 调试用：把击打点相对当前装甲板的位移放大，1 为真实值

} // namespace

int main(int, char *argv[])
{
    const std::string config_path = argv[1];

    vision::Camera camera(config_path);
    vision::Detector detector(config_path);
    vision::Solver solver(config_path);
    vision::Tracker tracker(config_path);
    vision::Aimer aimer(config_path);
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
            vision::draw_quad(frame.img, armor.corners, vision::color_detected);
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
                const auto corner = solver.project(p, a);
                const cv::Point2f point = vision::center_of(corner);
                ring.emplace_back(cvRound(point.x), cvRound(point.y));
            }
            cv::polylines(frame.img, ring, false, cv::Scalar(255, 128, 0), 1);
            const auto middle = solver.project(tracked->center, tracked->theta);
            const cv::Point2f center = vision::center_of(middle);
            cv::circle(frame.img, center, 3, cv::Scalar(255, 255, 255), -1);

            const auto now =
                solver.project(tracked->armor(0.0), tracked->armor_yaw(0.0));
            vision::draw_quad(frame.img, now, vision::color_tracked); // 深绿：滤波后的当前位置

            // 击打位置：交给 Aimer 算，工具不重复弹道
            const auto command =
                aimer.aim(tracked, gimbal.bullet_speed(), gimbal.yaw(), gimbal.pitch(),
                          frame.timestamp);
            const Eigen::Vector3d aim =
                vision::direction_of(command.yaw, command.pitch) * command.distance;
            const auto future = solver.project(
                tracked->armor(0.0) + lead_scale * (aim - tracked->armor(0.0)),
                tracked->armor_yaw(command.flight));
            vision::draw_quad(frame.img, future, vision::color_hit); // 红：击打位置

            const cv::Point2f ahead = vision::center_of(future);
            cv::circle(frame.img, ahead, 4, vision::color_hit, -1); // 红点：击打点

            cv::putText(
                frame.img,
                cv::format("%.2fm  v=%.2f  w=%.2frad/s", tracked->armor(0.0).norm(),
                           tracked->armor_velocity(0.0).norm(), tracked->omega),
                ahead, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
        }

        cv::imshow("tracker_test", frame.img);
        if (cv::waitKey(1) == 'q')
            break;
    }

    return 0;
}
