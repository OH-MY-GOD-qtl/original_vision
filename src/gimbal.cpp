#include "gimbal.hpp"

#include <array>
#include <cstring>
#include <stdexcept>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <yaml-cpp/yaml.h>

namespace
{

// 1.1 电控 → 视觉，29 字节，小端
struct __attribute__((packed)) FrameFromGimbal
{
    uint8_t head[2];
    uint8_t mode;
    float yaw;
    float yaw_vel;
    float pitch;
    float pitch_vel;
    float bullet_speed;
    int32_t id;
    uint16_t crc16;
};

// 1.2 视觉 → 电控，31 字节，小端
struct __attribute__((packed)) FrameToGimbal
{
    uint8_t head[2];
    uint8_t mode;
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc;
    uint8_t shoot;
    uint8_t is_empty;
    uint16_t crc16;
};

static_assert(sizeof(FrameFromGimbal) == 29, "接收帧必须是 29 字节");
static_assert(sizeof(FrameToGimbal) == 31, "发送帧必须是 31 字节");

// CRC16：查表右移，反射多项式 0x8408，初值 0xFFFF，无最终异或
uint16_t crc16(const uint8_t *data, size_t len)
{
    static const auto table = []
    {
        std::array<uint16_t, 256> t{};
        for (int i = 0; i < 256; ++i)
        {
            uint16_t crc = static_cast<uint16_t>(i);
            for (int bit = 0; bit < 8; ++bit)
                crc = (crc & 1) ? static_cast<uint16_t>((crc >> 1) ^ 0x8408)
                                : static_cast<uint16_t>(crc >> 1);
            t[static_cast<size_t>(i)] = crc;
        }
        return t;
    }();

    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i)
        crc = static_cast<uint16_t>((crc >> 8) ^ table[(crc ^ data[i]) & 0xFF]);
    return crc;
}

// 只映射几档常用波特率，其余按 115200 处理
speed_t baud_speed(int baud)
{
    switch (baud)
    {
    case 921600:
        return B921600;
    case 460800:
        return B460800;
    case 230400:
        return B230400;
    default:
        return B115200;
    }
}

} // namespace

namespace vision
{

Gimbal::Gimbal(const std::string &config_path)
{
    const YAML::Node yaml = YAML::LoadFile(config_path); // 缺少配置项或类型错误会抛异常
    bullet_speed_ = yaml["bullet_speed"].as<double>();
    const std::string port = yaml["com_port"].as<std::string>();
    if (port.empty())
        return; // 留空则不接串口，姿态固定为 0，纯离线调试

    const int baud = yaml["baud_rate"].as<int>();

    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0)
        throw std::runtime_error("打不开串口 " + port);

    termios tio{};
    ::tcgetattr(fd_, &tio);
    ::cfmakeraw(&tio);                           // 关掉行缓冲、回显和特殊字符处理
    tio.c_cflag |= CLOCAL | CREAD;               // 忽略调制解调器线，允许接收
    tio.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS); // 8N1，无硬件流控
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cc[VMIN] = 0;  // 读不阻塞
    tio.c_cc[VTIME] = 0;
    ::cfsetispeed(&tio, baud_speed(baud));
    ::cfsetospeed(&tio, baud_speed(baud));
    ::tcsetattr(fd_, TCSANOW, &tio);
}

Gimbal::~Gimbal()
{
    if (fd_ >= 0)
        ::close(fd_);
}

void Gimbal::read()
{
    if (fd_ < 0)
        return; // 离线模式，没有串口可读

    uint8_t chunk[256];
    const ssize_t n = ::read(fd_, chunk, sizeof chunk);
    if (n > 0)
        rx_.append(reinterpret_cast<char *>(chunk), static_cast<size_t>(n));

    while (!rx_.empty())
    {
        const size_t head = rx_.find("SP");
        if (head == std::string::npos)
        {
            const bool keep = rx_.back() == 'S'; // 末尾的 'S' 可能是下一帧的帧头
            rx_.erase(0, rx_.size() - (keep ? 1 : 0));
            return;
        }
        if (rx_.size() < head + sizeof(FrameFromGimbal))
        {
            rx_.erase(0, head);
            return;
        }

        FrameFromGimbal frame;
        std::memcpy(&frame, rx_.data() + head, sizeof frame);
        rx_.erase(0, head + sizeof frame); // 校验不过也吃掉，从下一个字节重新找帧头
        if (crc16(reinterpret_cast<const uint8_t *>(&frame), sizeof frame - 2) != frame.crc16)
            continue;

        mode_ = frame.mode;
        yaw_ = frame.yaw;
        pitch_ = frame.pitch;
        bullet_speed_ = frame.bullet_speed;
    }
}

void Gimbal::send(const Command &command)
{
    if (fd_ < 0)
        return; // 离线模式，无处可发

    FrameToGimbal frame{};
    frame.head[0] = 'S';
    frame.head[1] = 'P';
    frame.mode = command.shoot ? 2 : 1; // 视觉始终接管：1 控制云台不开火，2 控制并开火
    frame.yaw = static_cast<float>(command.yaw);
    frame.yaw_vel = static_cast<float>(command.yaw_vel);
    frame.yaw_acc = static_cast<float>(command.yaw_acc);
    frame.pitch = static_cast<float>(command.pitch);
    frame.pitch_vel = static_cast<float>(command.pitch_vel);
    frame.pitch_acc = static_cast<float>(command.pitch_acc);
    frame.shoot = command.shoot;
    frame.is_empty = command.is_empty;
    frame.crc16 =
        crc16(reinterpret_cast<const uint8_t *>(&frame), sizeof frame - 2); // 从帧头算到 is_empty

    ::write(fd_, &frame, sizeof frame);
}

} // namespace vision
