#include <cstdio>
#include <string>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "MvCameraControl.h"

// 参数：取配置文件路径，缺省用默认值
static std::string parse_config_path(int argc, char *argv[])
{
    const std::string keys = "{@config-path   | configs/camera.yaml | yaml配置文件路径 }";
    cv::CommandLineParser cli(argc, argv, keys);
    return cli.get<std::string>(0);
}

// 配置：只保存参数，不碰硬件
struct CameraConfig
{
    double exposure_ms = 0.0;
    double gain = 0.0;
};

static CameraConfig load_config(const std::string &path)
{
    const YAML::Node yaml = YAML::LoadFile(path); // 缺少配置项或类型错误会抛异常

    CameraConfig cfg;
    cfg.exposure_ms = yaml["exposure_ms"].as<double>();
    cfg.gain = yaml["gain"].as<double>();
    return cfg;
}

// 相机：枚举、打开、参数、取帧、释放
struct Camera
{
  private:
    void *handle = nullptr;

  public:
    Camera(const Camera &) = delete;
    Camera &operator=(const Camera &) = delete;

    Camera()
    {
        MV_CC_DEVICE_INFO_LIST list{};
        MV_CC_EnumDevices(MV_USB_DEVICE, &list);
        MV_CC_CreateHandle(&handle, list.pDeviceInfo[0]);
        MV_CC_OpenDevice(handle);
        MV_CC_StartGrabbing(handle);
    }

    ~Camera()
    {
        MV_CC_StopGrabbing(handle);
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        MV_CC_Finalize();
    }

    void set_exposure(double ms)
    {
        MV_CC_SetEnumValue(handle, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
        MV_CC_SetFloatValue(handle, "ExposureTime", ms * 1000.0); // 配置为 ms，SDK 为 us
    }

    void set_gain(double db)
    {
        MV_CC_SetEnumValue(handle, "GainAuto", MV_GAIN_MODE_OFF);
        MV_CC_SetFloatValue(handle, "Gain", db);
    }

    void set_white_balance_auto()
    {
        MV_CC_SetEnumValue(handle, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
    }

    MV_FRAME_OUT grab()
    {
        MV_FRAME_OUT frame{};
        MV_CC_GetImageBuffer(handle, &frame, 1000);
        return frame;
    }

    void release(MV_FRAME_OUT &frame) { MV_CC_FreeImageBuffer(handle, &frame); }
};

// 转码：BayerRG8 转三通道彩色图，写入 dst 复用内存
static void to_bgr(const MV_FRAME_OUT &frame, cv::Mat &dst)
{
    // 零拷贝视图，归还缓冲区后失效
    cv::Mat bayer(frame.stFrameInfo.nHeight, frame.stFrameInfo.nWidth, CV_8U, frame.pBufAddr);

    cv::cvtColor(bayer, dst, cv::COLOR_BayerRG2RGB);
}

// 帧率：每次 tick 返回距上次的瞬时值
struct FpsMeter
{
    std::chrono::steady_clock::time_point last = std::chrono::steady_clock::now();

    double tick()
    {
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> dt = now - last;
        last = now;
        return 1 / dt.count();
    }
};

// 显示：窗口、刷新、按键退出
struct Viewer
{
    Viewer() { cv::namedWindow("img", cv::WINDOW_NORMAL); }

    bool show(const cv::Mat &img) // 返回 true 表示用户按 q 退出
    {
        cv::imshow("img", img);
        return cv::waitKey(1) == 'q';
    }
};

int main(int argc, char *argv[])
{
    const std::string config_path = parse_config_path(argc, argv);

    const CameraConfig cfg = load_config(config_path);

    Camera camera; // 构造即打开并开始取流
    camera.set_exposure(cfg.exposure_ms);
    camera.set_gain(cfg.gain);
    camera.set_white_balance_auto();

    Viewer viewer;

    cv::Mat img;
    FpsMeter fps;

    while (true)
    {
        MV_FRAME_OUT frame = camera.grab();

        to_bgr(frame, img);

        camera.release(frame);

        std::printf("%.2f fps\n", fps.tick());

        if (viewer.show(img))
            break;
    }

    return 0;
}
