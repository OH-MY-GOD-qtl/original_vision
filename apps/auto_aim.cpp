// 自动瞄准总链路：时间戳从相机一路传到跟踪，全程不用 now()
#include <string>

#include "aimer.hpp"
#include "camera.hpp"
#include "detector.hpp"
#include "gimbal.hpp"
#include "solver.hpp"
#include "tracker.hpp"

int main(int, char **argv)
{
    const std::string config_path = argv[1];

    vision::Camera camera(config_path);
    vision::Solver solver(config_path);
    vision::Detector detector(config_path);
    vision::Tracker tracker(config_path);
    vision::Aimer aimer(config_path);
    vision::Gimbal gimbal(config_path);

    vision::Frame frame;

    for (;;)
    {
        camera.read(frame); // 图像与曝光中点时刻绑在一起
        gimbal.read();      // 取最新的姿态与弹速

        solver.set_attitude(gimbal.yaw(), gimbal.pitch());

        auto armors = detector.detect(frame.img);
        for (auto &armor : armors)
            solver.solve(armor); // 失败时 solved 保持 false，tracker 会跳过

        auto target = tracker.track(armors, frame.timestamp);
        gimbal.send(aimer.aim(target, gimbal.bullet_speed(), gimbal.yaw(), gimbal.pitch(),
                              frame.timestamp));
    }
}