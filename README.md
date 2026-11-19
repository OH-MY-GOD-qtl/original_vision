# 鼎行·双创战队 自瞄骨架

机器人视觉自瞄的工程骨架：相机、检测、解算、跟踪、串口已实现，剩下瞄准。

链路、接口、配置、构建都已就位，按模块逐个填入实现即可。

## 当前状态

| 模块 | 状态 | 职责 |
|---|---|---|
| `Camera` | ✅ 已实现 | 海康相机取流，输出图像 + 曝光中点时刻 |
| `Detector` | ✅ 已实现 | 图像 → 装甲板像素角点（传统色差法，参数待真机标定） |
| `Solver` | ✅ 已实现 | 像素角点 → 世界系坐标（IPPE PnP + 手眼变换） |
| `Tracker` | ✅ 已实现 | 多帧装甲板 → 唯一目标（8 维旋转卡尔曼：圆心 + 转角） |
| `Aimer` | ✅ 已实现 | 预瞄点外推 + 重力补偿；开火判定未接 |
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
./build/camera_test configs/auto_aim.yaml                 # 相机取流并显示，按 q 退出
./build/detector_test configs/auto_aim.yaml               # 检测结果叠加到画面，按 q 退出
./build/solver_test configs/auto_aim.yaml                 # 解算结果：距离 / yaw / pitch，按 q 退出
./build/plot_test configs/auto_aim.yaml                   # 整链路数据推 PlotJuggler，按 q 退出
./build/tracker_test configs/auto_aim.yaml                # 跟踪可视化：绿框检测 / 深绿框滤波 / 红框红点击打位置，按 q 退出
./build/auto_aim_test configs/auto_aim.yaml               # 全链路：画面 + 推 PlotJuggler，红框红点击打位置，不发指令
./build/apps/auto_aim configs/auto_aim.yaml               # 总链路（开火判定未接，其余已通）
./build/calibration/timestamp_check configs/auto_aim.yaml # 时间戳链路自检
./build/calibration/detector_tuning configs/auto_aim.yaml # 检测参数标定，ESC 退出
./build/calibration/capture configs/auto_aim.yaml configs/calibration.yaml assets/calib # 采集标定图
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
  - `angle.hpp` —— 角度约定 `direction_of` / `angles_of`，全项目共用
- 图像与时间戳用 `Frame` 强绑定，不会配错

## 坐标与角度约定

云台系取 $x$ 右、$y$ 下、$z$ 前，与相机系同构，机械对准时手眼矩阵就是单位阵。

- `yaw = atan2(x, z)`，正为向右
- `pitch = asin(y / ‖p‖)`，正为向下，也就是云台的仰角（不是 `atan2(y, z)`）
- 装甲板模型点取灯条中轴线四端点：$(\pm 65, \pm 27.5, 0)$ mm，顺序左上→右上→右下→左下

`Armor::xyz` 与 `Target` 存**世界系**坐标，由 $R_{gimbal\to world} = R_y(\text{yaw})\,R_x(-\text{pitch})$
变换得到，云台转动时静止目标的坐标不变。

这两个角是该复合旋转的欧拉角，前进轴为 $(\sin\psi\cos\theta,\ \sin\theta,\ \cos\psi\cos\theta)$，
所以 `pitch` 是仰角——用 `atan2(y, z)` 算会在 yaw 变大时偏小，6m 处 yaw 45°、仰角 10° 能差 420mm。

这套符号下全链路没有取负：目标在图像中心偏右对应 yaw 正，偏下对应 pitch 正。

`Armor::yaw` 是板面法线的世界系偏航角，指向相机那一侧，也就是装甲板在圆上的方位。

## 跟踪

装甲板固定在底盘四周的一个圆上，所以跟踪的不是一块自由运动的板，而是**圆心 + 转角**：

- 状态 8 维：`[cx cy cz vx vy vz theta omega]`，`theta` 是装甲板在圆上的方位，也是板面朝向
- 半径取配置里的 `armor_radius`，板子尺寸固定所以写死，不参与估计
- 观测 4 维：装甲板世界坐标（3）+ 板面角度（1），角度对位置有耦合，雅可比里 `∂h/∂theta = r`
- 换板：相邻两块夹角 `360 / armor_count`，看到另一块时位置和角度一起跳，转角先对齐再滤波，转速不受影响

预瞄不再按直线外推，而是让转角走 `theta + omega * t`，所以目标自转时预瞄点也在圆上跑：

```cpp
target.armor(t) // 圆心 + 速度 * t + 半径 * (sin(theta + omega t), 0, cos(theta + omega t))
```

## 相机模块

几个关键点：

- **设备时间戳**：`nDevTimeStamp` 单位 10ns、标记曝光开始。对时一次，之后只用设备时间差推导，帧间隔恒定 6.047ms（165.37fps），不受主机调度影响
- **对时基准**：构造时预热 100 帧，取到手延迟最小的那帧对齐时钟。首帧带着流建立开销，拿它对时会让之后每一帧都稳定偏晚数百微秒
- **曝光中点**：设备时刻 + 曝光时长 / 2
- **像素格式**：显式固定 `BayerRG8`，转码用 `COLOR_BayerRG2RGB`（本机相位与 OpenCV 命名相反，用 `BayerRG2BGR` 会红蓝互换）
- **不做防御**：相机断连、取帧失败的返回码一律不检查（理想环境前提）

自检时间戳链路：跑 `timestamp_check`，看帧间隔是否恒为 6.047ms、到手延迟是否稳定、设备时钟相对主机漂多少 ppm。
传输延迟不从软件里单独标定 —— 它和曝光时长共用一个观测量（帧到手时刻），拆不开；而 `frame.timestamp` 只参与帧间求差，常数偏移自己抵消，所以不设补偿参数。

## 检测参数标定

跑 `detector_tuning`：左侧窗口原图叠加绿框灯条与红线配对，右侧窗口是闭运算后的灯条二值图，
滑动条实时调全部检测参数，ESC 退出时打印可直接粘贴回 `auto_aim.yaml` 的配置。

像素判定：敌方色差 + 灰度亮度 → 二值图，再按 `morph_size` 做一次闭运算，把点阵屏灯条的断点连成一片
（闭运算不撑大外框；膨胀会把灯条两端各撑开半核，越远误差占比越大）。
灯条判定两层：`min_lightbar_area` 滤面积过小的碎点，`min_lightbar_ratio` 滤不够细长的。
配对再过四关：高度相近、倾角一致、中心对齐、间距像装甲板（倾角差阈值 `max_tilt_diff_deg`）。

## 相机内参标定

采集和计算分成两个工具：采集要举着板子摆姿势，计算会反复跑。

1. 跑 `capture`，实时预览上叠加识别到的角点，**识别到才允许存图**。按 `s` 存一张，按 `q` 退出。
   第一个参数是相机参数（与总链路共用 `auto_aim.yaml`），第二个是棋盘格几何。
   摆 15~20 个姿势：覆盖画面四角与中心、远近各一档、板面绕两个轴各倾斜 20~40 度；
   别让角点贴边，避开反光和过曝。
2. 跑 `camera_calib`，同一个目录，逐张打印重投影误差，最后打印可直接粘进 `auto_aim.yaml` 的
   `camera_matrix` 与 `distort_coeffs`。某张误差明显偏大就删掉重算。

棋盘格几何只放在 `configs/calibration.yaml`：`pattern_cols` / `pattern_rows` 是**内角点**数
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
- 前馈四项（vel / acc）目前填 0；预瞄已经用上 `Target.velocity`，但发出去的角速度前馈还没算
- **`com_port` 留空即离线模式**：不打开串口，`read()` 不更新姿态（yaw/pitch 恒为 0）、
  `send()` 直接丢弃指令。没接电控时也能跑 `detector_test` / `solver_test` / `plot_test`

## 实时曲线

`plot_test` 把整条链路的量用 UDP JSON 推到 PlotJuggler，**只收不发，不会指挥云台**。

1. PlotJuggler 里选 **Streaming → UDP Server**，端口填 `9870`，格式选 **JSON**，点开始
2. 另开终端跑 `./build/plot_test configs/auto_aim.yaml`

键名里的 `/` 会被 PlotJuggler 自动分组：

| 键 | 含义 |
|---|---|
| `loop/fps` | 主循环帧率 |
| `detector/armors` | 检测到的装甲板数 |
| `solver/solved` | 是否有解算成功的板（0/1），用它判断目标相关字段该段是否有效 |
| `target/x` `y` `z` `dist` | 目标世界坐标与距离，m |
| `target/yaw_deg` `pitch_deg` | 目标方向，度 |
| `predict/dist` | 滤波后的距离，与 `target/dist` 对比可看出滤波效果 |
| `predict/theta_deg` | 装甲板在圆上的方位角，换板时会跳 `360 / armor_count` |
| `predict/omega_deg` | 估计的转速，度/秒 |
| `gimbal/yaw_deg` `pitch_deg` | 云台当前姿态，度 |
| `gimbal/yaw_vel_deg` `pitch_vel_deg` | 云台回传的角速度，度/秒 |
| `err/yaw_deg` `pitch_deg` | 目标减云台，度 |
| `gimbal/bullet_speed` | 回传弹速 |

没有解算结果时，目标相关字段填 0。

`auto_aim_test` 推同一个端口，键名多一组 `tx/`——**即将发给云台的帧内容**：

| 键 | 含义 |
|---|---|
| `tx/mode` / `tx/shoot` / `tx/is_empty` | 帧里的模式位与标志位 |
| `tx/yaw_deg` `tx/pitch_deg` `tx/dist` | 目标绝对角与距离 |
| `tx/yaw_vel_deg` `tx/yaw_acc_deg` `tx/pitch_vel_deg` `tx/pitch_acc_deg` | 前馈项，度制，目前恒 0 |

串口上下行都是弧度，**推出去的所有角量都折成度**：带 `_deg` 的是度，角速度是度/秒，
角加速度是度/秒²。

`err/*` 是 `tx` 减 `gimbal`，也就是云台还差多少。两个工具共用 9870 端口，不要同时开。

## 配置

`configs/auto_aim.yaml` 是**唯一一份相机参数**，也是总配置，各模块的参数都在里面；
`configs/calibration.yaml` 只放棋盘格几何，相机参数从总配置读。

## 待办

按顺序补齐：`Aimer` 的速度前馈（`yaw_vel` / `pitch_vel`）与开火判定

