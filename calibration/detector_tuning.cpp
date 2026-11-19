// 标定：检测参数，滑动条实时调，退出时打印可直接粘贴回 auto_aim.yaml 的配置
// 算法全部走 Detector，调出来的参数与生产链路一致
#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "camera.hpp"
#include "detector.hpp"

// 滑动条只能绑 int，小数参数用放大倍数存
struct Sliders
{
    int enemy_red;       // 0 蓝 / 1 红
    int color;           // 敌方通道差值阈值
    int brightness;      // 灰度阈值
    int morph_size;      // 闭运算核边长，像素
    int min_area;        // 灯条面积下限，像素²
    int lightbar_ratio;  // 灯条长/宽 下限 ×10
    int height_ratio;    // 两灯条 短/长 下限 ×100
    int tilt_diff;       // 两灯条倾角差 上限，度 ×10
    int center_offset;   // 中心偏移/灯条长 上限 ×100
    int armor_ratio_min; // 间距/灯条长 下限 ×100
    int armor_ratio_max; // 间距/灯条长 上限 ×100
};

static Sliders sliders; // 滑动条直接改这几个 int

struct Slider
{
    const char *name;
    int *value;
    int max;
};

static void add_sliders(const std::string &win, const std::vector<Slider> &list)
{
    for (const auto &slider : list)
    {
        cv::createTrackbar(slider.name, win, nullptr, slider.max); // 不用已废弃的 value 指针
        cv::setTrackbarPos(slider.name, win, *slider.value);
    }
}

static void pull_sliders(const std::string &win, const std::vector<Slider> &list)
{
    for (const auto &slider : list)
        *slider.value = cv::getTrackbarPos(slider.name, win);
}

// 滑动条的 int → Detector 用的真实参数
static vision::DetectorParams to_params()
{
    vision::DetectorParams params;
    params.is_red = sliders.enemy_red != 0;
    params.color_threshold = sliders.color;
    params.brightness_threshold = sliders.brightness;
    params.morph_size = std::max(1, sliders.morph_size); // 滑动条能拉到 0，核最小 1
    params.min_lightbar_area = sliders.min_area;
    params.min_lightbar_ratio = sliders.lightbar_ratio / 10.0;
    params.min_height_ratio = sliders.height_ratio / 100.0;
    params.max_tilt_diff_deg = sliders.tilt_diff / 10.0;
    params.max_center_offset = sliders.center_offset / 100.0;
    params.min_armor_ratio = sliders.armor_ratio_min / 100.0;
    params.max_armor_ratio = sliders.armor_ratio_max / 100.0;
    return params;
}

int main(int, char *argv[])
{
    const std::string path = argv[1];

    vision::Detector detector(path); // 算法与生产链路同一份
    const vision::DetectorParams initial = detector.params(); // 从现有配置起步

    sliders.enemy_red = initial.is_red ? 1 : 0;
    sliders.color = static_cast<int>(initial.color_threshold);
    sliders.brightness = static_cast<int>(initial.brightness_threshold);
    sliders.morph_size = initial.morph_size;
    sliders.min_area = static_cast<int>(initial.min_lightbar_area);
    sliders.lightbar_ratio = static_cast<int>(initial.min_lightbar_ratio * 10);
    sliders.height_ratio = static_cast<int>(initial.min_height_ratio * 100);
    sliders.tilt_diff = static_cast<int>(initial.max_tilt_diff_deg * 10);
    sliders.center_offset = static_cast<int>(initial.max_center_offset * 100);
    sliders.armor_ratio_min = static_cast<int>(initial.min_armor_ratio * 100);
    sliders.armor_ratio_max = static_cast<int>(initial.max_armor_ratio * 100);

    vision::Camera camera(path);

    const std::string view_win = "detector tuning";
    const std::string mask_win = "lightbar mask";

    const std::vector<Slider> list = {
        {"enemy_red 0blue 1red", &sliders.enemy_red, 1},
        {"color_threshold", &sliders.color, 255},
        {"brightness_threshold", &sliders.brightness, 255},
        {"morph_size", &sliders.morph_size, 21},
        {"min_lightbar_area", &sliders.min_area, 2000},
        {"min_lightbar_ratio x10", &sliders.lightbar_ratio, 50},
        {"min_height_ratio x100", &sliders.height_ratio, 100},
        {"max_tilt_diff_deg x10", &sliders.tilt_diff, 900},
        {"max_center_offset x100", &sliders.center_offset, 100},
        {"min_armor_ratio x100", &sliders.armor_ratio_min, 500},
        {"max_armor_ratio x100", &sliders.armor_ratio_max, 1000},
    };

    cv::namedWindow(view_win, cv::WINDOW_NORMAL);
    cv::resizeWindow(view_win, 960, 720);
    add_sliders(view_win, list);
    cv::namedWindow(mask_win, cv::WINDOW_NORMAL);
    cv::resizeWindow(mask_win, 960, 720);

    vision::Frame frame;
    int tick = 0;
    while (cv::waitKey(1) != 27) // ESC 退出
    {
        camera.read(frame);
        pull_sliders(view_win, list);
        detector.set_params(to_params());

        const cv::Mat mask = detector.lightbar_mask(frame.img);
        const cv::Mat thick = detector.close(mask);
        const std::vector<cv::RotatedRect> bars = detector.find_lightbars(thick);
        const std::vector<vision::Armor> armors = detector.pair(bars);

        for (const auto &bar : bars) // 绿框：认下的灯条
        {
            cv::Point2f p[4];
            bar.points(p);
            for (int i = 0; i < 4; ++i)
                cv::line(frame.img, p[i], p[(i + 1) % 4], {0, 255, 0}, 1);
        }
        for (const auto &armor : armors) // 蓝框：配上的装甲板
        {
            for (size_t i = 0; i < 4; ++i)
                cv::line(frame.img, armor.corners[i], armor.corners[(i + 1) % 4], {255, 128, 0}, 2);
        }

        cv::imshow(view_win, frame.img);
        cv::imshow(mask_win, thick); // 显示闭运算后的图，就是进 findContours 的那张

        if (++tick % 30 == 0) // 每 30 帧打一次，别刷屏
            std::printf("灯条 %zu  装甲板 %zu  mask 像素 %d\n", bars.size(), armors.size(),
                        cv::countNonZero(mask));
    }

    std::printf("\n# 复制到 auto_aim.yaml\n");
    std::printf("enemy_color: \"%s\"\n", sliders.enemy_red ? "red" : "blue");
    std::printf("color_threshold: %d\n", sliders.color);
    std::printf("brightness_threshold: %d\n", sliders.brightness);
    std::printf("morph_size: %d\n", sliders.morph_size);
    std::printf("min_lightbar_area: %d\n", sliders.min_area);
    std::printf("min_lightbar_ratio: %.1f\n", sliders.lightbar_ratio / 10.0);
    std::printf("min_height_ratio: %.2f\n", sliders.height_ratio / 100.0);
    std::printf("max_tilt_diff_deg: %.1f\n", sliders.tilt_diff / 10.0);
    std::printf("max_center_offset: %.2f\n", sliders.center_offset / 100.0);
    std::printf("min_armor_ratio: %.2f\n", sliders.armor_ratio_min / 100.0);
    std::printf("max_armor_ratio: %.2f\n", sliders.armor_ratio_max / 100.0);

    return 0;
}
