# 鼎行·双创战队 自瞄骨架

机器人视觉自瞄的工程骨架：相机、检测、解算、串口已实现，剩下跟踪与瞄准。

链路、接口、配置、构建都已就位，按模块逐个填入实现即可。

## 当前状态

| 模块 | 状态 | 职责 |
|---|---|---|
| `Camera` | ✅ 已实现 | 海康相机取流，输出图像 + 曝光中点时刻 |
| `Detector` | ✅ 已实现 | 图像 → 装甲板像素角点（传统色差法，参数待真机标定） |
| `Solver` | ✅ 已实现 | 像素角点 → 世界系坐标（IPPE PnP + 手眼变换） |
| `Tracker` | ⬜ 空 | 多帧装甲板 → 唯一目标（滤波预测） |
| `Aimer` | 🟡 部分 | 无目标时停留后缓慢归零；弹道解算未实现 |
| `Gimbal` | ✅ 已实现 | 指令 ↔ 下位机串口（115200 8N1，CRC16） |

## 数据流

```
Camera ──Frame{img, t}──► Detector ──Armor──► Solver ──► Tracker ──Target──► Aimer ──Command──► Gimbal
```

`t` 是**曝光中点**，从相机一路传到跟踪，全链路不使用 `now()`。

云台姿态（相机光轴的世界 yaw/pitch）由 `Gimbal` 从串口回传，不再单独建 `Imu` 模块。

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
./build/camera_test configs/camera.yaml                   # 相机取流并显示，按 q 退出
./build/detector_test configs/auto_aim.yaml               # 检测结果叠加到画面，按 q 退出
./build/solver_test configs/auto_aim.yaml                 # 解算结果：距离 / yaw / pitch，按 q 退出
./build/apps/auto_aim configs/auto_aim.yaml               # 总链路（Tracker 未实现，静默空转）
./build/calibration/transport_delay configs/camera.yaml   # 传输延迟标定
./build/calibration/detector_tuning configs/auto_aim.yaml # 检测参数标定，ESC 退出
./build/calibration/capture configs/calibration.yaml assets/calib      # 采集棋盘格，s 存图 q 退出
./build/calibration/camera_calib configs/calibration.yaml assets/calib # 算内参，打印可粘贴的 yaml
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

## 坐标与角度约定

云台系取 $x$ 右、$y$ 下、$z$ 前，与相机系同构，机械对准时手眼矩阵就是单位阵。

- `yaw = atan2(x, z)`，正为向右
- `pitch = atan2(y, z)`，正为向下
- 装甲板模型点取灯条中轴线四端点：$(\pm 65, \pm 27.5, 0)$ mm，顺序左上→右上→右下→左下

`Armor::xyz` 与 `Target` 存**世界系**坐标，由 $R_{gimbal\to world} = R_y(\text{yaw})\,R_x(-\text{pitch})$ 变换得到，
云台转动时静止目标的坐标不变。

这套符号下全链路没有取负：目标在图像中心偏右对应 yaw 正，偏下对应 pitch 正。

## 相机模块

几个关键点：

- **设备时间戳**：`nDevTimeStamp` 单位 10ns、标记曝光开始。首帧对时一次，之后只用设备时间差推导，帧间隔恒定 6.047ms（165.37fps），不受主机调度影响
- **曝光中点**：设备时刻 + 曝光时长 / 2
- **传输补偿**：`transport_ms` 抵消 USB 传输延迟，实测低于 0.3ms 噪声，故为 0
- **像素格式**：显式固定 `BayerRG8`，转码用 `COLOR_BayerRG2RGB`（本机相位与 OpenCV 命名相反，用 `BayerRG2BGR` 会红蓝互换）
- **不做防御**：相机断连、取帧失败的返回码一律不检查（理想环境前提）

标定 `transport_ms`：跑 `transport_delay`，把建议值写回 `camera.yaml`。

## 检测参数标定

跑 `detector_tuning`：左侧窗口原图叠加绿框灯条与红线配对，右侧窗口是灯条二值图，
滑动条实时调全部检测参数，ESC 退出时打印可直接粘贴回 `auto_aim.yaml` 的配置。

灯条判定两层：`min_lightbar_area` 滤面积过小的碎点，`min_lightbar_ratio` 滤不够细长的。
配对再过四关：高度相近、倾角一致、中心对齐、间距像装甲板（倾角差阈值 `max_tilt_diff_deg`）。

## 相机内参标定

采集和计算分成两个工具：采集要举着板子摆姿势，计算会反复跑。

1. 跑 `capture`，实时预览上叠加识别到的角点，**识别到才允许存图**。按 `s` 存一张，按 `q` 退出。
   摆 15~20 个姿势：覆盖画面四角与中心、远近各一档、板面绕两个轴各倾斜 20~40 度；
   别让角点贴边，避开反光和过曝。
2. 跑 `camera_calib`，同一个目录，逐张打印重投影误差，最后打印可直接粘进 `auto_aim.yaml` 的
   `camera_matrix` 与 `distort_coeffs`。某张误差明显偏大就删掉重算。

标定板几何在 `configs/calibration.yaml`：`pattern_cols` / `pattern_rows` 是**内角点**数
（格子数减一），`square_mm` 是方格边长。平均重投影误差 0.5 px 以内可用。

## 下位机通信

`Gimbal` 独占一个串口（默认 `/dev/ttyACM0`，115200 8N1），双向定长帧，小端，结构体紧凑无填充。

| 方向 | 长度 | 内容 |
|---|---|---|
| 电控 → 视觉 | 29 字节 | mode、相机光轴的世界 yaw/pitch 及角速度、弹速、id |
| 视觉 → 电控 | 31 字节 | mode、目标绝对 yaw/pitch 及一二阶导、shoot、is_empty |

- 帧头 `'S' 'P'`，CRC16 覆盖帧头到校验字段之前，小端存放
- CRC16：查表右移、反射多项式 `0x8408`、初值 `0xFFFF`、无最终异或（`"123456789"` → `0x6F91`）
- 读是**非阻塞**的：每次 `read()` 排空缓冲区并保留最新一帧，不拖视觉循环
- 收发两个方向的 `yaw` 都指**相机光轴的世界航向**，大小 yaw 的分配由电控负责
- 前馈四项（vel / acc）目前填 0，等 `Tracker` 上线再补速度项

## 配置

`configs/auto_aim.yaml` 是总配置，各段已有 TODO 占位；`configs/camera.yaml` 供相机测试单独使用；
`configs/calibration.yaml` 供相机标定使用，相机段之外多一份棋盘格几何。

## 待办

按顺序补齐：`Tracker` → `Aimer`（弹道解算 + 前馈）

