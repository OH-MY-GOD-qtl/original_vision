# 鼎行·双创战队 自瞄骨架

机器人视觉自瞄的工程骨架：**相机模块已实现，其余模块为空壳**。

链路、接口、配置、构建都已就位，按模块逐个填入实现即可。

## 当前状态

| 模块 | 状态 | 职责 |
|---|---|---|
| `Camera` | ✅ 已实现 | 海康相机取流，输出图像 + 曝光中点时刻 |
| `Detector` | ⬜ 空 | 图像 → 装甲板像素角点 |
| `Solver` | ⬜ 空 | 角点 + 云台姿态 → 云台系坐标 |
| `Tracker` | ⬜ 空 | 多帧装甲板 → 唯一目标（滤波预测） |
| `Aimer` | ⬜ 空 | 目标 + 弹速 → 云台指令 |
| `Gimbal` | ⬜ 空 | 指令 → 下位机串口 |
| `Imu` | ⬜ 未建 | 姿态历史查询（`imu.at(t)`） |

## 数据流

```
Camera ──Frame{img, t}──► Detector ──Armor──► Solver ──► Tracker ──Target──► Aimer ──Command──► Gimbal
```

`t` 是**曝光中点**，从相机一路传到姿态查询与跟踪，全链路不使用 `now()`。

## 依赖

- C++17、CMake ≥ 3.16
- OpenCV 4.x、Eigen3、yaml-cpp
- 海康 MVS SDK（已放在 `third_party/`，无需额外安装）
- 相机：MV-CS016-10UC（USB3，1440×1080，BayerRG8）

## 构建

```bash
cmake -S . -B build
cmake --build build -j
```

## 运行

```bash
./build/camera_test configs/camera.yaml                  # 相机取流并显示，按 q 退出
./build/apps/auto_aim configs/auto_aim.yaml              # 总链路（检测为空，静默空转）
./build/calibration/transport_delay configs/camera.yaml  # 传输延迟标定
```

## 目录

```
apps/          总链路
calibration/   标定工具，一个文件一个标定
configs/       配置，各模块读自己那一段
include/       头文件
src/           实现，与 include 同名
test/          真机测试
third_party/   海康 MVS SDK
```

## 模块约定

- 构造函数只收 `config_path`，模块自己读配置
- 一个模块一个静态库，**target 名 = 文件名**
- 模块之间不互相 include，只依赖共享类型头：
  - `armor.hpp` —— Detector 产、Solver 消费
  - `target.hpp` —— Tracker 产、Aimer 消费
  - `command.hpp` —— Aimer 产、Gimbal 消费
- 图像与时间戳用 `Frame` 强绑定，不会配错

## 相机模块

唯一已实现的部分，几个关键点：

- **设备时间戳**：`nDevTimeStamp` 单位 10ns、标记曝光开始。首帧对时一次，之后只用设备时间差推导，帧间隔恒定 6.047ms（165.37fps），不受主机调度影响
- **曝光中点**：设备时刻 + 曝光时长 / 2
- **传输补偿**：`transport_ms` 抵消 USB 传输延迟，实测低于 0.3ms 噪声，故为 0
- **像素格式**：显式固定 `BayerRG8`，转码用 `COLOR_BayerRG2RGB`（本机相位与 OpenCV 命名相反，用 `BayerRG2BGR` 会红蓝互换）
- **不做防御**：相机断连、取帧失败的返回码一律不检查（理想环境前提）

标定 `transport_ms`：跑 `transport_delay`，把建议值写回 `camera.yaml`。

## 配置

`configs/auto_aim.yaml` 是总配置，各段已有 TODO 占位；`configs/camera.yaml` 供相机测试单独使用。

## 待办

按顺序补齐：`Detector` → `Solver` → `Tracker` → `Aimer` → `Gimbal` → `Imu`

`Imu` 最后接入，在此之前 `apps/auto_aim.cpp` 用单位四元数顶替姿态。

